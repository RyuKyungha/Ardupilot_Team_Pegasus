#include "Copter.h"
#include "mode.h"
#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Math/quaternion.h>

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
static constexpr float A0 = -4.86946108f;
static constexpr float A1 = -5.18465967e+01f;
static constexpr float A2 =  7.59618559e-03f;
static constexpr float A3 =  8.03983134f;
static constexpr float A4 = -2.28529105e-03f;
static constexpr float A5 = -9.51971564e-01f;

// ==============================
// 초기 진입
// ==============================

ModeClimb::ModeClimb() : Mode() {}

bool ModeClimb::init(bool ignore_checks)
{
    gcs().send_text(MAV_SEVERITY_INFO,
                    "ModeClimb: init");

    // SERVO7 점유 (timeout 길게)
    SRV_Channels::set_output_pwm_chan_timeout(
        SERVO_CH,
        PWM_CENTER,
        100
    );

    return true;
}

// ==============================
// 모드 종료
// ==============================

void ModeClimb::exit()
{
    // override 해제
    SRV_Channels::set_output_pwm_chan_timeout(
        SERVO_CH,
        0,
        0
    );

    gcs().send_text(MAV_SEVERITY_INFO,
                    "ModeClimb: exit");
}

// ==============================
// Main Loop
// ==============================

void ModeClimb::run()
{
    static uint32_t last_log_ms = 0;
    uint32_t now = AP_HAL::millis();

    // --------------------------------------------------
    // 1) Current quaternion (Body -> Earth)
    // --------------------------------------------------
    Quaternion q_now;
    if (!copter.ahrs.get_quaternion(q_now)) {
        return;
    }

    // --------------------------------------------------
    // 2) Target quaternion (example: yaw +30 deg)
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
    const float pwm_f =
        PWM_CENTER + SERVO_GAIN * theta3;

    const int16_t pwm =
        (int16_t)constrain(pwm_f, PWM_MIN, PWM_MAX);

    // 9) 출력
    //SRV_Channels::set_output_pwm_chan_timeout(SERVO_CH, pwm, 100);
    hal.rcout->write(6, pwm);

    // 7) 디버깅 출력
    if (now - last_log_ms > 1000) {  // 1초 간격으로만 출력
        gcs().send_text(
            MAV_SEVERITY_INFO,
            "q_now   = [%.4f %.4f %.4f %.4f]",
            q_now.q1, q_now.q2, q_now.q3, q_now.q4
        );

        gcs().send_text(
            MAV_SEVERITY_INFO,
            "q_tgt   = [%.4f %.4f %.4f %.4f]",
            q_target.q1, q_target.q2, q_target.q3, q_target.q4
        );
        gcs().send_text(
            MAV_SEVERITY_INFO,
            "CLIMB[u-tilt] | "
            "t2=%.1f deg | t3=%.1f deg | pwm=%d",
            theta2 * 180.0f / M_PI,
            theta3 * 180.0f / M_PI,
            pwm
        );
        last_log_ms = now;
    }
}

#endif // MODE_CLIMB_ENABLED