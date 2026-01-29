#include "Copter.h"
#include "mode.h"
#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Math/quaternion.h>
#include <AP_Logger/AP_Logger.h>

#if MODE_CLIMB_ENABLED

#ifndef constrain
#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

// ==============================
// 설정값
// ==============================

static constexpr uint8_t  SERVO_CH = 7;
static constexpr float    PWM_CENTER = 1500.0f;
static constexpr float    PWM_MIN = 1000.0f;
static constexpr float    PWM_MAX = 2000.0f;

// 서보 물리 제한 ±60deg
static constexpr float THETA3_MAX = M_PI / 3.0f;
static constexpr float THETA3_MIN = -M_PI / 3.0f;

// rad → pwm 스케일
static constexpr float SERVO_GAIN = 1/(M_PI/1500.0f);

// u-axis = (1/2, 0, sqrt(3)/2)
static constexpr float UX = 0.5f;
static constexpr float UY = 0.0f;
static constexpr float UZ = -0.8660254f;

// 5차 다항식 계수 (deg 기준)
// theta3 = a0 + a1*theta2 + ... + a5*theta2^5
static constexpr float A0 = -3.68758925f;
static constexpr float A1 =  5.89591294f;
static constexpr float A2 =  6.55444525f;
static constexpr float A3 = -3.60187384e+01f;
static constexpr float A4 = -1.65223999f;
static constexpr float A5 =  9.58412665f;

// ==============================
// 초기 진입
// ==============================

ModeClimb::ModeClimb() : Mode() {}

bool ModeClimb::init(bool ignore_checks)
{
    gcs().send_text(MAV_SEVERITY_INFO, "ModeClimb: init");

    // SERVO7 점유 (timeout 길게)
    SRV_Channels::set_output_pwm_chan_timeout(SERVO_CH, PWM_CENTER, 100);
    
    log_enable = true;
    AP::logger().Write("UHD2", "event", "s", "START");

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
}

// ==============================
// Main Loop
// ==============================

void ModeClimb::run()
{
    // --------------------------------------------------
    // 1) Current quaternion (Body -> Earth)
    // --------------------------------------------------
    Quaternion q_now;
    if (!copter.ahrs.get_quaternion(q_now)) {
        return;
    }
    const uint32_t now_us = AP_HAL::micros();

    const float rc3 = -1 * channel_pitch->norm_input();   // -1.0 ~ +1.0
    const int16_t forward = constrain(rc3 * 100.0f, -100, 100); //-100 ~ +100
    
    // --------------------------------------------------
    // 2) Target quaternion
    // --------------------------------------------------
    Quaternion q_target( // pitch 120deg
        0.5,             // w
        0.0f,            // x
        0.8660254f,      // y
        0.0f             // z
    );

    // --------------------------------------------------
    // 3) Relative rotation quaternion
    //    q_rel = q_target^{-1} * q_now
    // --------------------------------------------------
    Quaternion q_rel = q_target.inverse() * q_now;

    // --------------------------------------------------
    // 4) Quaternion -> rotation vector (log map)
    // --------------------------------------------------
    Vector3f rotvec;

    const float w = q_rel.q1;
    const float x = q_rel.q2;
    const float y = q_rel.q3;
    const float z = q_rel.q4;

    const float sin_half = sqrtf(x*x + y*y + z*z);

    if (sin_half > 1e-6f) {
        const float angle = 2.0f * atan2f(sin_half, w);
        const float scale = angle / sin_half;

        rotvec.x = x * scale;
        rotvec.y = y * scale;
        rotvec.z = z * scale;
    } else {
        rotvec.zero();
    }

    // --------------------------------------------------
    // 5) u-axis (BODY frame)
    // --------------------------------------------------
    Vector3f u_axis(0.5f, 0.0f, -0.86602540f);
    u_axis.normalize();

    // --------------------------------------------------
    // 6) theta2 = rotation about u-axis
    // --------------------------------------------------
    float theta2 = rotvec.dot(u_axis);   // [rad]

    // --------------------------
    // 7) Polynomial mapping → theta3
    // --------------------------
    float theta3 =
        (( A0 +
        A1 * theta2 +
        A2 * sq(theta2) +
        A3 * powf(theta2, 3) +
        A4 * powf(theta2, 4) +
        A5 * powf(theta2, 5) )) / (180.0f / M_PI);

    theta3 = constrain(theta3, THETA3_MIN, THETA3_MAX);

    // --------------------------
    // 8) theta3 → PWM
    const float pwm_f = PWM_CENTER + SERVO_GAIN * theta3;
    const int16_t pwm = (int16_t)constrain(pwm_f, PWM_MIN, PWM_MAX);

    const float Kp_theta = 2.5f;
    float theta2_dot_cmd = -Kp_theta * theta2;
    theta2_dot_cmd = constrain(theta2_dot_cmd, -1.0f, 1.0f); // rad/s 제한

    const Vector3f gyro = ahrs.get_gyro();     // rad/s
    const float theta2_dot = gyro.dot(u_axis);

    const float theta2_dot_err = theta2_dot_cmd - theta2_dot;

    const float Kp_rate = 80.0f;               // PWM / (rad/s)
    int16_t delta = (int16_t)(Kp_rate * theta2_dot_err);

    delta = constrain(delta, -50, 50);

    const int16_t pwm_L = constrain(1500 + forward + delta, 1400, 1600);
    const int16_t pwm_R = constrain(1500 + forward - delta, 1400, 1600);

    hal.rcout->write(6, pwm);
    write_drive_motors(pwm_L, pwm_R);

    /* =====================================================
    * 12) .bin 로그 조건부 기록
    * ===================================================== */
    static uint32_t last_log_us = 0;

    if (log_enable && (now_us - last_log_us > 50000)) { // 20 Hz
        AP::logger().Write(
            "UHD2",
            "TimeUS,heading",
            "Qf",
            AP_HAL::micros64(),
            theta2
        );

        last_log_us = now_us;
    }

}

void ModeClimb::write_drive_motors(int16_t left_pwm, int16_t right_pwm)
{
    hal.rcout->write(0, left_pwm);  // Motor1: 좌측 바퀴
    hal.rcout->write(2, right_pwm); // Motor2: 우측 바퀴
}

#endif // MODE_CLIMB_ENABLED