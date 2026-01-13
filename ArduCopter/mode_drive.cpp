#include "Copter.h"
#include "mode.h"
#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Math/quaternion.h>

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

    return true;
}

void ModeDrive::exit()
{
    // tail override 해제: timeout=0이면 즉시 원래 기능으로 복귀
    SRV_Channels::set_output_pwm_chan_timeout(6, 0, 0);

    // 모드 종료 시 추가 동작 없음
    gcs().send_text(MAV_SEVERITY_INFO, "Drive mode exited");
}

/*void ModeDrive::run()
{
    static uint32_t last_log_ms = 0;
    uint32_t now = AP_HAL::millis();

    if (!interpolate_mode(0)){
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        return;
    } 
    
    // 조종기 입력 정규화: [-1.0 ~ +1.0]
    const float V = channel_throttle->norm_input();  // 속도 크기 (-1.0~+1.0): 후진~전진
    const float F = channel_pitch->norm_input(); // 전/후 속도 결정 
    const float W = channel_roll->norm_input();  // 좌/우 회전 방향

    // 속도 스케일링: [-500 ~ +500]
    const int16_t forward = V * F * 500;
    const int16_t turn    = V * W * 500;

    // 좌/우 바퀴 속도 계산 및 제한
    const int16_t L = constrain(forward + turn, -500, 500);
    const int16_t R = constrain(forward - turn, -500, 500);

    // PWM 출력값 계산 (중앙값 1500 기준, ±500)
    const int16_t pwm_L = 1500 + L;  // 좌측 바퀴
    const int16_t pwm_R = 1500 + R;  // 우측 바퀴

    // 모터에 출력
    write_drive_motors(pwm_L, pwm_R);

    if (now - last_log_ms > 1000) {  // 1초 간격으로만 출력
        gcs().send_text(MAV_SEVERITY_INFO, "입력값 - Throttle: %.2f, Roll: %.2f", V, W);
        gcs().send_text(MAV_SEVERITY_INFO, "PWM 출력 - Left: %d, Right: %d", pwm_L, pwm_R);
        last_log_ms = now;
    }
}*/

void ModeDrive::run()
{
    static uint32_t last_log_ms = 0;
    const uint32_t now = AP_HAL::millis();

    if (!interpolate_mode(0)) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        return;
    }

    /* =====================================================
     * 1) RC3 입력만 사용 (전/후진)
     *    RC3: 1500 중립, 1400~1600 사용
     * ===================================================== */
    const float rc3 = channel_pitch->norm_input();   // -1.0 ~ +1.0

    // PWM 기준 forward 값 (-100 ~ +100)
    const int16_t forward = constrain(rc3 * 100.0f, -100, 100);

    /* =====================================================
     * 2) 현재 자세 quaternion
     * ===================================================== */
    Quaternion q_now;
    if (!copter.ahrs.get_quaternion(q_now)) {
        return;
    }

    /* =====================================================
     * 3) 목표 quaternion
     *    (예시: 고정, 실제로는 진입 시 latch 권장)
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
     * 7) θ₂ 계산
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
    const int16_t pwm_L = constrain(1500 + forward + delta, 1400, 1600);
    const int16_t pwm_R = constrain(1500 + forward - delta, 1400, 1600);

    write_drive_motors(pwm_L, pwm_R);

    /* =====================================================
     * 11) 디버그 로그
     * ===================================================== */
    if (now - last_log_ms > 1000) {
        gcs().send_text(MAV_SEVERITY_INFO,
            "RC3: %.2f | theta2: %.3f | theta2_dot: %.3f",
            rc3, theta2, theta2_dot);

        gcs().send_text(MAV_SEVERITY_INFO,
            "PWM L/R: %d / %d | delta: %d",
            pwm_L, pwm_R, delta);

        last_log_ms = now;
    }
}

void ModeDrive::write_drive_motors(int16_t left_pwm, int16_t right_pwm)
{
    hal.rcout->write(0, left_pwm);  // Motor1: 좌측 바퀴
    hal.rcout->write(2, right_pwm); // Motor2: 우측 바퀴
}


#endif