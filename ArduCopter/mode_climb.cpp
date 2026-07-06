#include "Copter.h"
#include "mode.h"
#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Math/quaternion.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_RPM/AP_RPM.h>
#include <RC_Channel/RC_Channel.h>

#if MODE_CLIMB_ENABLED

#ifndef constrain
#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

// ==============================
// 설정값
// ==============================

// SERVO7: theta3용 서보
static constexpr uint8_t  SERVO_CH = 7;
static constexpr float    PWM_CENTER = 1500.0f;
static constexpr float    PWM_MIN = 1000.0f;
static constexpr float    PWM_MAX = 2000.0f;

// 서보 물리 제한 ±60deg
static constexpr float THETA3_MAX = M_PI / 3.0f;
static constexpr float THETA3_MIN = -M_PI / 3.0f;

// rad → pwm 스케일
static constexpr float SERVO_GAIN = 1.0f / (M_PI / 1500.0f);

// 5차 다항식 계수 (deg 기준)
static constexpr float A0 = -1.65870202e+00f;
static constexpr float A1 = -3.15286298e+01f;
static constexpr float A2 =  1.58815657e+00f;
static constexpr float A3 = -7.70787407e+00f;
static constexpr float A4 = -3.55817027e-01f;
static constexpr float A5 =  2.75112288e+00f;

// ==============================
// RPM 폐루프 제어 설정값
// ==============================

// SERVO4 = hal.rcout->write(3, ...)
static constexpr uint8_t RPM_OUT_CH_ZERO_BASE = 3;

// CH2IN = rc_channel(1) (0-based)
static constexpr uint8_t RPM_RC_CH_INDEX = 1;

// RPM1 instance
static constexpr uint8_t RPM_INSTANCE = 0;

// 조종기 명령 기반 기본 PWM 범위
static constexpr float RPM_PWM_BASE_MIN = 1500.0f;
static constexpr float RPM_PWM_BASE_MAX = 1850.0f;

// 최종 출력 허용 범위
static constexpr float RPM_PWM_OUT_MIN = 1500.0f;
static constexpr float RPM_PWM_OUT_MAX = 1875.0f;

// 제어 피드백 최대 ±25us
static constexpr float RPM_FEEDBACK_MIN = -10.0f;
static constexpr float RPM_FEEDBACK_MAX =  10.0f;

static constexpr float RCIN_MIN = 1000.0f;
static constexpr float RCIN_MAX = 2000.0f;

// 목표 RPM 범위
static constexpr float RPM_TARGET_MIN = 0.0f;
static constexpr float RPM_TARGET_MAX = 11700.0f;

// PI gains
static constexpr float RPM_KP = 0.02f;
static constexpr float RPM_KI = 0.005f;

// 적분항 제한
static constexpr float RPM_I_MIN = -5000.0f;
static constexpr float RPM_I_MAX =  5000.0f;

// 상태변수
static float    rpm_i_term = 0.0f;
static uint64_t rpm_last_update_us = 0;

// ==============================
// 보조 함수
// ==============================

static inline float map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
    x = constrain(x, in_min, in_max);
    return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
}

static float theta2_between(const Quaternion& q_ref, const Quaternion& q_now, const Vector3f& u_axis)
{
    const Quaternion q_rel = q_ref.inverse() * q_now;

    const float w = q_rel.q1;
    const float x = q_rel.q2;
    const float y = q_rel.q3;
    const float z = q_rel.q4;

    const float sin_half = sqrtf(x*x + y*y + z*z);

    Vector3f rotvec;
    if (sin_half > 1e-6f) {
        const float angle = 2.0f * atan2f(sin_half, w);
        const float scale = angle / sin_half;
        rotvec.x = x * scale;
        rotvec.y = y * scale;
        rotvec.z = z * scale;
    } else {
        rotvec.zero();
    }

    return rotvec.dot(u_axis);   // [rad]
}

// ==============================
// 초기 진입
// ==============================

ModeClimb::ModeClimb() : Mode() {}

bool ModeClimb::init(bool ignore_checks)
{
    gcs().send_text(MAV_SEVERITY_INFO, "ModeClimb: init");

    // SERVO7 점유
    SRV_Channels::set_output_pwm_chan_timeout(SERVO_CH, PWM_CENTER, 100);

    log_enable = true;
    AP::logger().Write("UHD2", "event", "s", "START");

    rpm_i_term = 0.0f;
    rpm_last_update_us = AP_HAL::micros64();

    return true;
}

// ==============================
// 모드 종료
// ==============================

void ModeClimb::exit()
{
    // override 해제
    SRV_Channels::set_output_pwm_chan_timeout(SERVO_CH, 0, 0);

    gcs().send_text(MAV_SEVERITY_INFO, "ModeClimb: exit");

    AP::logger().Write("UHD2", "event", "s", "END");
    log_enable = false;

    rpm_i_term = 0.0f;
    rpm_last_update_us = 0;
}

void ModeClimb::write_drive_motors(int16_t left_pwm, int16_t right_pwm)
{
    hal.rcout->write(0, left_pwm);  // Motor1: 좌측 바퀴
    hal.rcout->write(2, right_pwm); // Motor2: 우측 바퀴
}

void ModeClimb::run() // double loop + RPM loop
{
    if (!interpolate_mode(0)) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        return;
    }

    /* =====================================================
     * 1) RC3 입력 사용 (전/후진)
     * ===================================================== */
    const float rc3 = -1.0f * channel_pitch->norm_input();   // -1.0 ~ +1.0
    const int16_t forward = constrain(rc3 * 100.0f, -100, 100);

    /* =====================================================
     * 2) 현재 자세 quaternion 취득
     * ===================================================== */
    Quaternion q_now;
    if (!copter.ahrs.get_quaternion(q_now)) {
        return;
    }

    /* =====================================================
     * 3) 목표 quaternion (직진 기준) + RC4(roll)로 u-axis 추가 회전
     * ===================================================== */
    const Quaternion q_target_base(
        0.5f,
        0.0f,
        0.8660254f,
        0.0f
    );

    const float rc4 = -1.0f * channel_roll->norm_input();
    float theta2_cmd_deg = rc4 * 40.0f;
    theta2_cmd_deg = constrain(theta2_cmd_deg, -40.0f, 40.0f);

    Vector3f u_axis(0.5f, 0.0f, -0.8660254f);
    u_axis.normalize();

    const float theta2_cmd_rad = radians(theta2_cmd_deg);

    Quaternion q_delta;
    q_delta.from_axis_angle(u_axis, theta2_cmd_rad);

    Quaternion q_target = q_target_base * q_delta;
    q_target.normalize();

    /* =====================================================
     * 4) theta2 / theta2_base 계산
     * ===================================================== */
    const float theta2      = theta2_between(q_target,      q_now, u_axis);
    const float theta2_base = theta2_between(q_target_base, q_now, u_axis);

    /* =====================================================
     * 5) theta3 polynomial mapping -> servo pwm
     * ===================================================== */
    float theta3 =
        (A0 +
         A1 * theta2_base +
         A2 * sq(theta2_base) +
         A3 * powf(theta2_base, 3) +
         A4 * powf(theta2_base, 4) +
         A5 * powf(theta2_base, 5)) / (180.0f / M_PI);

    theta3 = constrain(theta3, THETA3_MIN, THETA3_MAX);

    const float pwm_f = PWM_CENTER - SERVO_GAIN * theta3;
    const int16_t pwm_servo = (int16_t)constrain(pwm_f, PWM_MIN, PWM_MAX);

    /* =====================================================
     * 6) Outer loop: theta2 -> theta2_dot_cmd
     * ===================================================== */
    const float Kp_theta = 2.5f;
    float theta2_dot_cmd = -Kp_theta * theta2;
    theta2_dot_cmd = constrain(theta2_dot_cmd, -1.0f, 1.0f);

    /* =====================================================
     * 7) Inner loop: gyro projection
     * ===================================================== */
    const Vector3f gyro = ahrs.get_gyro();
    const float theta2_dot = gyro.dot(u_axis);

    const float theta2_dot_err = theta2_dot_cmd - theta2_dot;

    const float Kp_rate = 80.0f;
    int16_t delta = (int16_t)(Kp_rate * theta2_dot_err);
    delta = constrain(delta, -50, 50);

    /* =====================================================
     * 8) 좌/우 바퀴 + SERVO7 출력
     * ===================================================== */
    const int16_t pwm_L = constrain(1500 + forward - delta, 1400, 1600);
    const int16_t pwm_R = constrain(1500 + forward + delta, 1400, 1600);

    write_drive_motors(pwm_L, pwm_R);
    hal.rcout->write(6, pwm_servo);

    /* =====================================================
     * 9) CH2IN 기반 RPM1 폐루프 -> SERVO4 출력
     * ===================================================== */
    RC_Channel *ch2 = RC_Channels::rc_channel(RPM_RC_CH_INDEX);
    if (ch2 != nullptr) {
        const float ch2in = (float)ch2->get_radio_in();

        // CH2IN -> 기본 PWM (1500~1850)
        const float pwm_base = map_float(ch2in,
                                         RCIN_MIN, RCIN_MAX,
                                         RPM_PWM_BASE_MIN, RPM_PWM_BASE_MAX);

        // CH2IN -> 목표 RPM
        const float rpm_target = map_float(ch2in,
                                           RCIN_MIN, RCIN_MAX,
                                           RPM_TARGET_MIN, RPM_TARGET_MAX);

        // RPM1 읽기
        float rpm_meas = 0.0f;
        bool rpm_valid = false;

        AP_RPM *rpm_backend = AP::rpm();
        if (rpm_backend != nullptr) {
            rpm_valid = rpm_backend->get_rpm(RPM_INSTANCE, rpm_meas);
        }
        if (!rpm_valid) {
            rpm_meas = 0.0f;
        }

        // dt 계산
        const uint64_t now_us64 = AP_HAL::micros64();
        float dt = 0.01f;

        if (rpm_last_update_us != 0 && now_us64 > rpm_last_update_us) {
            dt = (now_us64 - rpm_last_update_us) * 1.0e-6f;
        }
        rpm_last_update_us = now_us64;

        if (dt <= 0.0f || dt > 0.1f) {
            dt = 0.01f;
        }

        // PI 제어
        const float rpm_err = rpm_target - rpm_meas;

        float next_i_term = rpm_i_term + rpm_err * dt;
        next_i_term = constrain(next_i_term, RPM_I_MIN, RPM_I_MAX);

        float pwm_correction = RPM_KP * rpm_err + RPM_KI * next_i_term;
        pwm_correction = constrain(pwm_correction,
                                   RPM_FEEDBACK_MIN, RPM_FEEDBACK_MAX);

        float rpm_pwm_f = pwm_base + pwm_correction;
        rpm_pwm_f = constrain(rpm_pwm_f, RPM_PWM_OUT_MIN, RPM_PWM_OUT_MAX);

        // anti-windup
        const bool upper_saturated = (rpm_pwm_f >= RPM_PWM_OUT_MAX - 0.5f);
        const bool lower_saturated = (rpm_pwm_f <= RPM_PWM_OUT_MIN + 0.5f);

        if ((!upper_saturated && !lower_saturated) ||
            (upper_saturated && rpm_err < 0.0f) ||
            (lower_saturated && rpm_err > 0.0f)) {
            rpm_i_term = next_i_term;
        }

        const uint16_t rpm_pwm = (uint16_t)rpm_pwm_f;
        hal.rcout->write(RPM_OUT_CH_ZERO_BASE, rpm_pwm);
    }

    /* =====================================================
     * 10) 로그
     * ===================================================== */
    static uint32_t last_log_us = 0;
    const uint32_t now_us = AP_HAL::micros();

    if (log_enable && (now_us - last_log_us > 50000)) { // 20 Hz
        AP::logger().Write(
            "UHD2",
            "TimeUS,heading,th2,th2dot,pL,pR",
            "Qfffff",
            AP_HAL::micros64(),
            theta2_base,
            theta2,
            theta2_dot,
            (float)pwm_L,
            (float)pwm_R
        );

        last_log_us = now_us;
    }
}

#endif