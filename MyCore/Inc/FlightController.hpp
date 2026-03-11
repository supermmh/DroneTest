#pragma once
#include "DataStructConfig.hpp"
#include <stdint.h>

#ifdef __cplusplus

// 基础 PID 控制器类
class PID {
public:
    float kp, ki, kd;
    float limit_i, limit_out;
    float alpha; // D项一阶低通滤波系数
    float integral;
    float prev_meas;
    float filt_d;
    bool first_run;

    PID(float p, float i, float d, float lim_i, float lim_out, float a);
    void reset();
    float compute(float setpoint, float meas, float dt);
};

// 四旋翼飞控管理器
class FlightController {
public:
    PID pid_roll_angle;
    PID pid_pitch_angle;
    PID pid_roll_rate;
    PID pid_pitch_rate;
    PID pid_yaw_rate;

    FlightController();
    void reset_integrals();
    void update(const OuterSetpoint_t& out_sp, const InnerSetpoint_t& in_sp,
                const VehicleState_t& state, const float rates[3], float dt, uint16_t pwms[4]);
};

extern FlightController g_FlightController;

#endif