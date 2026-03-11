#pragma once

#ifdef __cplusplus
#include "DataStructConfig.hpp"

#ifndef ARM_MATH_CM7
#define ARM_MATH_CM7
#endif
#ifndef __FPU_PRESENT
#define __FPU_PRESENT 1U
#endif
#include "arm_math.h"

class ESKF
{
public:
    ESKF();
    void Init(float initial_yaw, float initial_alt);
    void AlignAndCalibrate(const float acc_mean[3], const float gyro_mean[3], float init_yaw);

    void PredictIMU(const float acc[3], const float gyro[3], float dt);
    void UpdateBaro(float alt, float r_var);
    void UpdateMag(const float mag[3]);
    void UpdateFlow(float flow_dx, float flow_dy, float dt);
    void UpdateAccel(const float acc[3]);
    void UpdateZUPT(void);
    VehicleState_t GetState(uint64_t timestamp);
    void SetStaticBias(const float accel_bias[3], const float gyro_bias[3]);
    float base_baro;
    bool is_inited;

private:
    float p[3], v[3], q[4], bg[3], ba[3];
    float last_gyro[3];

    float32_t P_data[225];
    arm_matrix_instance_f32 P_mat;

    void GetRotationMatrix(float Rmat[3][3]);
    // 🛡️ 新增卡方检验开关，防止正常修正被误杀
    void ScalarUpdate(const float H[15], float y, float R_var, bool use_chisq = true);
    void InjectError(const float dx[15]);
};
#endif