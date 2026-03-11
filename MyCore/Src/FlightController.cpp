#include "FlightController.hpp"

PID::PID(float p, float i, float d, float lim_i, float lim_out, float a) {
    kp = p; ki = i; kd = d;
    limit_i = lim_i; limit_out = lim_out; alpha = a;
    reset();
}

void PID::reset() {
    integral = 0.0f;
    prev_meas = 0.0f;
    filt_d = 0.0f;
    first_run = true;
}

float PID::compute(float setpoint, float meas, float dt) {
    if (dt < 0.0001f) return 0.0f;
    float err = setpoint - meas;
    integral += err * ki * dt;

    // 1. 抗积分饱和 (Anti-Windup)
    if (integral > limit_i) integral = limit_i;
    else if (integral < -limit_i) integral = -limit_i;

    // 2. 仅对测量值求导，避免遥控器打杆突变时的 D-kick 冲击
    float d_meas = 0.0f;
    if (!first_run) {
        d_meas = -(meas - prev_meas) / dt;
    }
    prev_meas = meas;
    first_run = false;

    // 3. 一阶低通滤波 (PT1) 平滑微分高频噪声
    filt_d = alpha * d_meas + (1.0f - alpha) * filt_d;

    float out = kp * err + integral + kd * filt_d;
    
    // 4. 最终力矩输出限幅
    if (out > limit_out) out = limit_out;
    else if (out < -limit_out) out = -limit_out;

    return out;
}

FlightController::FlightController() :
    // P, I, D, LimI, LimOut, Alpha (滤波系数)
    // 角度外环 (纯P控制)，输出目标角速度 (rad/s)
    pid_roll_angle (4.5f, 0.0f, 0.0f, 0.0f, 3.0f, 1.0f),
    pid_pitch_angle(4.5f, 0.0f, 0.0f, 0.0f, 3.0f, 1.0f),
    
    // 角速度内环 (PID控制)，输出 PWM 偏移量 (-1.0 ~ 1.0)
    // 500Hz 下 alpha=0.15 对应约 13Hz 的低通滤波，能完美过滤机架震动
    pid_roll_rate  (0.12f, 0.06f, 0.003f, 0.2f, 0.5f, 0.15f),
    pid_pitch_rate (0.12f, 0.06f, 0.003f, 0.2f, 0.5f, 0.15f),
    pid_yaw_rate   (0.30f, 0.10f, 0.0f,   0.2f, 0.5f, 1.0f)
{}

void FlightController::reset_integrals() {
    pid_roll_angle.reset();
    pid_pitch_angle.reset();
    pid_roll_rate.reset();
    pid_pitch_rate.reset();
    pid_yaw_rate.reset();
}

void FlightController::update(const OuterSetpoint_t& out_sp, const InnerSetpoint_t& in_sp,
                               const VehicleState_t& state, const float rates[3], float dt, uint16_t pwms[4]) {
    
    // 1. 获取推力：地面站右扳机已经映射为 [0.0, 1.0]
    float throttle = in_sp.target_thrust;
    if (throttle < 0.0f) throttle = 0.0f;
    if (throttle > 1.0f) throttle = 1.0f;

    // 2. 解锁/待机与安全保护 (Dead-man's switch)
    // 地面防积分饱和：如果油门极低 (<5%) 强行清除积分，防止在地面不平导致积分累积起飞瞬间翻车
    if (out_sp.ctrl_mode != 0x01 || throttle < 0.05f) {
        reset_integrals();
        pwms[0] = 1000; pwms[1] = 1000; pwms[2] = 1000; pwms[3] = 1000;
        return;
    }

    // 3. 遥控器指令映射
    // Xbox右摇杆推满(vx > 0)，表示飞机前进，即对应低头(Pitch在FRD中为负)
    // Xbox右摇杆推满(vy > 0)，表示飞机右飞，即对应右倾(Roll在FRD中为正)
    float target_roll     = out_sp.target_vy * 0.6f;  // 最大倾角约 35 度
    float target_pitch    = -out_sp.target_vx * 0.6f; 
    float target_yaw_rate = out_sp.target_yaw * 1.5f; // 最大偏航角速度 90 度/s

    // 4. 外环控制 (角度误差 -> 目标角速度)
    float target_roll_rate  = pid_roll_angle.compute(target_roll, state.roll, dt);
    float target_pitch_rate = pid_pitch_angle.compute(target_pitch, state.pitch, dt);

    // 5. 内环控制 (角速度误差 -> 电机力矩控制量)
    float roll_out  = pid_roll_rate.compute(target_roll_rate, rates[0], dt);
    float pitch_out = pid_pitch_rate.compute(target_pitch_rate, rates[1], dt);
    float yaw_out   = pid_yaw_rate.compute(target_yaw_rate, rates[2], dt);

    // 6. X型四旋翼动力学混控 (Kinematic Mixer)
    float m1 = throttle - roll_out + pitch_out + yaw_out; // M1: 右前 (FR) CCW
    float m2 = throttle + roll_out - pitch_out + yaw_out; // M2: 左后 (BL) CCW
    float m3 = throttle + roll_out + pitch_out - yaw_out; // M3: 左前 (FL) CW
    float m4 = throttle - roll_out - pitch_out - yaw_out; // M4: 右后 (BR) CW

    auto clamp_pwm = [](float v) {
        v = 1000.0f + v * 1000.0f;
        if (v < 1050.0f) return 1050.0f; // 保证起飞后怠速不断转（防空中停机）
        if (v > 2000.0f) return 2000.0f;
        return v;
    };

    pwms[0] = (uint16_t)clamp_pwm(m1);
    pwms[1] = (uint16_t)clamp_pwm(m2);
    pwms[2] = (uint16_t)clamp_pwm(m3);
    pwms[3] = (uint16_t)clamp_pwm(m4);
}

FlightController g_FlightController;