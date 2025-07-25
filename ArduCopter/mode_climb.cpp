#include "Copter.h"
#include "mode.h"
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Math/quaternion.h>

#if MODE_CLIMB_ENABLED

#ifndef constrain
#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

// pitch 각도에 따른 PWM 출력 (역방향: pitch↑ → PWM↓)
static const uint8_t climbAngles[] = { 35, 45, 50, 60, 70, 80, 90 };
static const uint16_t climbPWM[]  = { 1500, 1450, 1400, 1300, 1200, 1100, 1000 };
static const uint8_t CLIMB_POINTS = sizeof(climbAngles) / sizeof(climbAngles[0]);

ModeClimb::ModeClimb() : Mode() {}

bool ModeClimb::init(bool ignore_checks)
{
    hal.rcout->write(9, 1100);
    hal.rcout->write(8, 1900);
    gcs().send_text(MAV_SEVERITY_INFO, "Climb mode running");
    
    // 1. 채널 번호 → 기능 저장
    tail_orig_fn = SRV_Channels::channel_function(6);

    // 2. 기능 제거
    SRV_Channels::set_default_function(6, SRV_Channel::Function::k_none);
    SRV_Channels::update_aux_servo_function();

    gcs().send_text(MAV_SEVERITY_INFO, "CH7 function: %d", SRV_Channels::channel_function(6));

    // 3. 원하는 PWM 출력 (예: tail servo 중립값)
    hal.rcout->write(6, 1500);

    return true;
}

void ModeClimb::exit()
{
    SRV_Channels::set_default_function(6, tail_orig_fn);
    SRV_Channels::update_aux_servo_function();

    gcs().send_text(MAV_SEVERITY_INFO, "CH7 function: %d", SRV_Channels::channel_function(6));

    gcs().send_text(MAV_SEVERITY_INFO, "Climb mode exited");
}

void ModeClimb::run()
{
    static uint32_t last_log_ms = 0;
    uint32_t now = AP_HAL::millis();

    // ─────────────── 1. pitch 각도 측정 ───────────────
    // 쿼터니언 변수 선언
    Quaternion quat;

    // pitch_deg를 먼저 선언하고 초기화
    float pitch_deg = 0.0f;

    // 쿼터니언 받아오기
    bool success = copter.ahrs.get_quaternion(quat);

    // 성공 시 계산
    if (success) {
        float sinp = 2.0f * (quat.q1 * quat.q3 - quat.q4 * quat.q2);
        float cosp = 1.0f - 2.0f * (quat.q3 * quat.q3 + quat.q4 * quat.q4);
        float pitch_rad = atan2f(sinp, cosp);

        pitch_deg = degrees(pitch_rad);
    }

    // ─────────────── 2. pitch → 추력 변환 ───────────────
    float pwm_us = get_thrust_from_pitch(pitch_deg);

    // PWM 직접 출력
    hal.rcout->write(3, pwm_us);  // 채널 4 출력

    if (now - last_log_ms > 1000) {  // 1초 간격으로만 출력
        gcs().send_text(MAV_SEVERITY_INFO, "Degree: %.2f, Pwm_us: %.2f", pitch_deg, pwm_us);
        last_log_ms = now;
    }

    // ─────────────── 4. 주행 제어 (Drive 방식) ───────────────
    // 조종기 입력 정규화: [-1.0 ~ +1.0]
    const float V = channel_throttle->norm_input();  // 속도 크기 (-1.0~+1.0): 후진~전진
    const float F = channel_pitch->norm_input(); // 전/후 속도 결정 
    const float W = channel_roll->norm_input();  // 좌/우 회전 방향

    // 속도 스케일링: [-500 ~ +500]
    const int16_t forward = V * F * 200;
    const int16_t turn    = V * W * 200;

    // 좌/우 바퀴 속도 계산 및 제한
    const int16_t L = constrain(forward + turn, -500, 500);
    const int16_t R = constrain(forward - turn, -500, 500);

    // PWM 출력값 계산 (중앙값 1500 기준, ±500)
    const int16_t pwm_L = 1500 + L;  // 좌측 바퀴
    const int16_t pwm_R = 1500 + R;  // 우측 바퀴

    // 모터에 출력
    write_drive_motors(pwm_L, pwm_R);
}

void ModeClimb::write_drive_motors(int16_t left_pwm, int16_t right_pwm)
{
    hal.rcout->write(0, left_pwm);  // Motor1: 좌측 바퀴
    hal.rcout->write(2, right_pwm); // Motor2: 우측 바퀴
}

// 두 점 (x0, y0), (x1, y1) 에서 기울기 0인 3차 함수 보간
float cubic_interpolate(float x, float x0, float x1, float y0, float y1) 
{
    float t = (x - x0) / (x1 - x0);  // 정규화: 0~1
    float t2 = t * t;
    float t3 = t2 * t;

    // Hermite basis functions (with m0 = m1 = 0 → 평평한)
    float h00 = 2 * t3 - 3 * t2 + 1;
    //float h10 = t3 - 2 * t2 + t;
    float h01 = -2 * t3 + 3 * t2;
    //float h11 = t3 - t2;

    // 여기선 기울기 m0, m1을 0으로 두면 각 점에서 평평함
    return h00 * y0 + h01 * y1;  // m0, m1 = 0 → h10, h11 생략
}

float get_thrust_from_pitch(float thetaDeg)
{
    if (thetaDeg <= climbAngles[0]) {
        return climbPWM[0];
    } else if (thetaDeg >= climbAngles[CLIMB_POINTS - 1]) {
        return climbPWM[CLIMB_POINTS - 1];
    }

    for (uint8_t i = 0; i < CLIMB_POINTS - 1; ++i) {
        float x0 = climbAngles[i];
        float x1 = climbAngles[i + 1];
        if (thetaDeg >= x0 && thetaDeg <= x1) {
            float y0 = climbPWM[i];
            float y1 = climbPWM[i + 1];
            return cubic_interpolate(thetaDeg, x0, x1, y0, y1);  // 기울기 0 보간
        }
    }
    return climbPWM[CLIMB_POINTS - 1];
}

#endif