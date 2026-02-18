#pragma once
#include "main.h"
#include "DataStructConfig.hpp"

#ifdef __cplusplus
extern "C" {
#endif

// FreeRTOS 任务入口暴露给 C 环境
void AttitudeEstimator_Init(void);

#ifdef __cplusplus
}

class ESKF {
public:
    ESKF();
    void Init(float initial_yaw, float initial_alt);
    
    // 核心卡尔曼循环
    void PredictIMU(const float acc[3], const float gyro[3], float dt);
    void UpdateBaro(float alt);
    void UpdateMag(const float mag[3]);
    void UpdateFlow(float flow_dx, float flow_dy, float dt);
    void UpdateAccel(const float acc[3]);
    void UpdateZUPT(void);
    VehicleState_t GetState(uint64_t timestamp);

    float base_baro;
    bool is_inited;
    
private:
    float p[3];      // 名义位置 NED
    float v[3];      // 名义速度 NED
    float q[4];      // 名义四元数
    float bg[3];     // 陀螺仪零偏
    float ba[3];     // 加速度零偏
    float P[15][15]; // 误差状态协方差矩阵
    
    float last_gyro[3]; 

    void GetRotationMatrix(float Rmat[3][3]);
    void ScalarUpdate(const float H[15], float y, float R_var);
    void InjectError(const float dx[15]);
    
};
#endif