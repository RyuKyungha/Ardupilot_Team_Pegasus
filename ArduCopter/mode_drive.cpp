#include "Copter.h"
#include "mode.h"
#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Math/quaternion.h>
#include <AP_Logger/AP_Logger.h>

#if MODE_DRIVE_ENABLED

#ifndef constrain
#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

ModeDrive::ModeDrive() : Mode() {}

bool ModeDrive::init(bool ignore_checks)
{
    // 초기화: 특별한 검사는 생략
    gcs().send_text(MAV_SEVERITY_INFO, "Drive mode running");

    // tail 서보 PWM 1500us 고정, 무제한 시간 동안 override
    SRV_Channels::set_output_pwm_chan_timeout(6, 1500, 0xFFFF);

    log_enable = true;
    AP::logger().Write("UHD2", "event", "s", "START");

    return true;
}

void ModeDrive::exit()
{
    // tail override 해제: timeout=0이면 즉시 원래 기능으로 복귀
    SRV_Channels::set_output_pwm_chan_timeout(6, 0, 0);

    // 모드 종료 시 추가 동작 없음
    gcs().send_text(MAV_SEVERITY_INFO, "Drive mode exited");

    AP::logger().Write("UHD2", "event", "s", "END");
    log_enable = false;
}

void ModeDrive::run() // double loop
{
    static uint32_t last_log_ms = 0;
    const uint32_t now = AP_HAL::millis();

    if (!interpolate_mode(0)) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        return;
    }

    const uint32_t now_us = AP_HAL::micros();

    /* =====================================================
     * 1) RC3 입력 사용 (전/후진)
     *    RC3: 1500 중립, 1400~1600 사용
     * ===================================================== */
    const float rc3 = -1 * channel_pitch->norm_input();   // -1.0 ~ +1.0
    // PWM 기준 forward 값 (-100 ~ +100)
    const int16_t forward = constrain(rc3 * 100.0f, -100, 100);

    /* =====================================================
     * 2) 현재 자세 quaternion 취득
     * ===================================================== */
    Quaternion q_now;
    if (!copter.ahrs.get_quaternion(q_now)) {
        return;
    }

    /* =====================================================
     * 3) 목표 quaternion (직진 기준)
     * ===================================================== */
    Quaternion q_target( // pitch 30deg
        0.9659258f,      // w
        0.0f,            // x
        0.2588190f,      // y
        0.0f             // z
    );

    /* =====================================================
     * 4) 상대 회전 quaternion
     * ===================================================== */
    Quaternion q_rel = q_target.inverse() * q_now;

    /* =====================================================
     * 5) log map → rotation vector
     * ===================================================== */
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

    /* =====================================================
     * 6) u-axis (BODY frame)
     * ===================================================== */
    Vector3f u_axis(0.5f, 0.0f, -0.8660254f);
    u_axis.normalize();

    /* =====================================================
     * 7) θ₂ 계산 (커브 반영)
     * ===================================================== */
    const float theta2 = rotvec.dot(u_axis);   // [rad]

    /* =====================================================
     * 8) Outer loop: θ₂ → θ̇₂_cmd
     * ===================================================== */
    const float Kp_theta = 2.5f;
    float theta2_dot_cmd = -Kp_theta * theta2;
    theta2_dot_cmd = constrain(theta2_dot_cmd, -1.0f, 1.0f); // rad/s 제한

    /* =====================================================
     * 9) Inner loop: θ̇₂ (gyro 사영)
     * ===================================================== */
    const Vector3f gyro = ahrs.get_gyro();     // rad/s
    const float theta2_dot = gyro.dot(u_axis);

    const float theta2_dot_err = theta2_dot_cmd - theta2_dot;

    const float Kp_rate = 80.0f;               // PWM / (rad/s)
    int16_t delta = (int16_t)(Kp_rate * theta2_dot_err);

    // 좌/우 차등은 ±50 PWM까지만 허용
    delta = constrain(delta, -50, 50);

    /* =====================================================
     * 10) 좌/우 바퀴 PWM (1400~1600)
     * ===================================================== */
    //const int16_t pwm_L = constrain(1500 + forward, 1400, 1600);
    //const int16_t pwm_R = constrain(1500 + forward, 1400, 1600);
    const int16_t pwm_L = constrain(1500 + forward + delta, 1400, 1600);
    const int16_t pwm_R = constrain(1500 + forward - delta, 1400, 1600);
    write_drive_motors(pwm_L, pwm_R);

    /* =====================================================
     * 11) 디버그 로그
     * ===================================================== */
    if (now - last_log_ms > 1000) {
        gcs().send_text(MAV_SEVERITY_INFO,
            "RC3: %.2f | theta2: %.3f | theta2_dot: %.3f",
            rc3,theta2, theta2_dot);

        gcs().send_text(MAV_SEVERITY_INFO,
            "PWM L/R: %d / %d | delta: %d",
            pwm_L, pwm_R, delta);

        last_log_ms = now;
    }

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

void ModeDrive::write_drive_motors(int16_t left_pwm, int16_t right_pwm)
{
    hal.rcout->write(0, left_pwm);  // Motor1: 좌측 바퀴
    hal.rcout->write(2, right_pwm); // Motor2: 우측 바퀴
}


#endif