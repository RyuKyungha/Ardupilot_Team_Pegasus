#include "Copter.h"
#include "mode.h"
#include <AP_HAL/AP_HAL.h>

#if MODE_DRIVE_ENABLED

#ifndef constrain
#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

ModeDrive::ModeDrive() : Mode() {}

bool ModeDrive::init(bool ignore_checks)
{
    _interp_step  = 0;
    // 초기화: 특별한 검사는 생략
    gcs().send_text(MAV_SEVERITY_INFO, "Drive mode running");
    return true;
}

void ModeDrive::exit()
{
    // 모드 종료 시 추가 동작 없음
    gcs().send_text(MAV_SEVERITY_INFO, "Drive mode exited");
}

void ModeDrive::run()
{
    static uint32_t last_log_ms = 0;
    uint32_t now = AP_HAL::millis();
    if (!copter.motors->armed()) {
        return;
    }
    if (_interp_step <= 800) {
        int i = _interp_step++;
        hal.rcout->write(0, 1500);  // Motor1: 좌측 바퀴
        hal.rcout->write(2, 1500); // Motor2: 우측 바퀴
        hal.rcout->write(9, 1900 - i);
        hal.rcout->write(8, 1100 + i);
    }

    if (_interp_step < 800) {
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
}

void ModeDrive::write_drive_motors(int16_t left_pwm, int16_t right_pwm)
{
    hal.rcout->write(0, left_pwm);  // Motor1: 좌측 바퀴
    hal.rcout->write(2, right_pwm); // Motor2: 우측 바퀴
}


#endif
