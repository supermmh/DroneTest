#include "AttitudeESKF.hpp"
#include "QueueConfig.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>
#include "DebugMonitor.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 放入 DTCM 极速内存，避免占用 FreeRTOS 任务栈
DTCM_DATA ESKF eskf;
DTCM_DATA static StackType_t ESKFTaskStack[1024];
DTCM_DATA static StaticTask_t ESKFTaskTCB;
TaskHandle_t ESKFTaskHandle = NULL;

ESKF::ESKF()
{
    is_inited = false;
    base_baro = 101325.0f;
}

void ESKF::Init(float initial_yaw, float initial_alt)
{
    memset(p, 0, sizeof(p));
    memset(v, 0, sizeof(v));
    memset(bg, 0, sizeof(bg));
    memset(ba, 0, sizeof(ba));

    p[2] = -initial_alt; // NED坐标系，Z向下为正

    q[0] = cosf(initial_yaw * 0.5f);
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = sinf(initial_yaw * 0.5f);

    memset(P, 0, sizeof(P));
    for (int i = 0; i < 15; i++) {
        if (i < 9)
            P[i][i] = 1e-2f; // 位置、速度、姿态初始方差
        else
            P[i][i] = 1e-4f; // 零偏初始方差较小
    }
    is_inited = true;
}

void ESKF::GetRotationMatrix(float R[3][3])
{
    float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    R[0][0] = 1.0f - 2.0f * (qy * qy + qz * qz);
    R[0][1] = 2.0f * (qx * qy - qw * qz);
    R[0][2] = 2.0f * (qx * qz + qw * qy);
    R[1][0] = 2.0f * (qx * qy + qw * qz);
    R[1][1] = 1.0f - 2.0f * (qx * qx + qz * qz);
    R[1][2] = 2.0f * (qy * qz - qw * qx);
    R[2][0] = 2.0f * (qx * qz - qw * qy);
    R[2][1] = 2.0f * (qy * qz + qw * qx);
    R[2][2] = 1.0f - 2.0f * (qx * qx + qy * qy);
}

void ESKF::PredictIMU(const float acc[3], const float gyro[3], float dt)
{
    // 1. 扣除零偏
    float true_acc[3]  = {acc[0] - ba[0], acc[1] - ba[1], acc[2] - ba[2]};
    float true_gyro[3] = {gyro[0] - bg[0], gyro[1] - bg[1], gyro[2] - bg[2]};

    last_gyro[0] = true_gyro[0];
    last_gyro[1] = true_gyro[1];
    last_gyro[2] = true_gyro[2];

    float Rmat[3][3];
    GetRotationMatrix(Rmat);

    // 2. 名义运动学积分
    float a_ned[3] = {
        Rmat[0][0] * true_acc[0] + Rmat[0][1] * true_acc[1] + Rmat[0][2] * true_acc[2],
        Rmat[1][0] * true_acc[0] + Rmat[1][1] * true_acc[1] + Rmat[1][2] * true_acc[2],
        Rmat[2][0] * true_acc[0] + Rmat[2][1] * true_acc[1] + Rmat[2][2] * true_acc[2] + 9.80665f // 重力补偿
    };

    for (int i = 0; i < 3; i++) {
        p[i] += v[i] * dt + 0.5f * a_ned[i] * dt * dt;
        v[i] += a_ned[i] * dt;
    }

    float theta = sqrtf(true_gyro[0] * true_gyro[0] + true_gyro[1] * true_gyro[1] + true_gyro[2] * true_gyro[2]) * dt;
    float dq[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    if (theta > 1e-6f) {
        float s = sinf(theta * 0.5f) / (theta / dt);
        dq[0]   = cosf(theta * 0.5f);
        dq[1]   = true_gyro[0] * s;
        dq[2]   = true_gyro[1] * s;
        dq[3]   = true_gyro[2] * s;
    }

    float q_new[4] = {
        q[0] * dq[0] - q[1] * dq[1] - q[2] * dq[2] - q[3] * dq[3],
        q[0] * dq[1] + q[1] * dq[0] + q[2] * dq[3] - q[3] * dq[2],
        q[0] * dq[2] - q[1] * dq[3] + q[2] * dq[0] + q[3] * dq[1],
        q[0] * dq[3] + q[1] * dq[2] - q[2] * dq[1] + q[3] * dq[0]};
    float norm = sqrtf(q_new[0] * q_new[0] + q_new[1] * q_new[1] + q_new[2] * q_new[2] + q_new[3] * q_new[3]);
    for (int i = 0; i < 4; i++) q[i] = q_new[i] / norm;

    // 3. 构造状态转移雅可比矩阵 F (15x15)
    float F[15][15] = {0};
    for (int i = 0; i < 15; i++) F[i][i] = 1.0f;
    for (int i = 0; i < 3; i++) F[i][i + 3] = dt;

    float skew_a[3][3] = {{0, -true_acc[2], true_acc[1]}, {true_acc[2], 0, -true_acc[0]}, {-true_acc[1], true_acc[0], 0}};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            F[3 + i][6 + j]  = -(Rmat[i][0] * skew_a[0][j] + Rmat[i][1] * skew_a[1][j] + Rmat[i][2] * skew_a[2][j]) * dt;
            F[3 + i][12 + j] = -Rmat[i][j] * dt;
        }
    }
    F[6][7]  = true_gyro[2] * dt;
    F[6][8]  = -true_gyro[1] * dt;
    F[7][6]  = -true_gyro[2] * dt;
    F[7][8]  = true_gyro[0] * dt;
    F[8][6]  = true_gyro[1] * dt;
    F[8][7]  = -true_gyro[0] * dt;
    F[6][9]  = -dt;
    F[7][10] = -dt;
    F[8][11] = -dt;

    // 4. 协方差推演: P = F * P * F^T + Q (矩阵展开优化算法)
    float FP[15][15] = {0};
    for (int i = 0; i < 15; i++) {
        for (int k = 0; k < 15; k++) {
            if (F[i][k] != 0.0f) {
                for (int j = 0; j < 15; j++) FP[i][j] += F[i][k] * P[k][j];
            }
        }
    }
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 15; k++)
                if (F[j][k] != 0.0f) sum += FP[i][k] * F[j][k];
            P[i][j] = sum;
        }
    }

    // 注入过程噪声 Q
    for (int i = 3; i < 6; i++) P[i][i] += 1e-2f * dt;   // 速度游走
    for (int i = 6; i < 9; i++) P[i][i] += 1e-4f * dt;   // 角度游走
    for (int i = 9; i < 12; i++) P[i][i] += 1e-6f * dt;  // 陀螺零偏游走
    for (int i = 12; i < 15; i++) P[i][i] += 1e-5f * dt; // 加速零偏游走
}

// =================== 极速降维：单向标量更新 ===================
void ESKF::ScalarUpdate(const float H[15], float y, float R_var)
{
    float PHt[15] = {0};
    float S       = R_var;
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++)
            if (H[j] != 0.0f) PHt[i] += P[i][j] * H[j];
        S += H[i] * PHt[i];
    }
    if (S < 1e-6f) return;

    float K[15], dx[15];
    for (int i = 0; i < 15; i++) {
        K[i]  = PHt[i] / S;
        dx[i] = K[i] * y;
    }

    InjectError(dx);

    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) P[i][j] -= K[i] * PHt[j];
    }
    for (int i = 0; i < 15; i++) { // 强制对称
        for (int j = i + 1; j < 15; j++) {
            float sym = 0.5f * (P[i][j] + P[j][i]);
            P[i][j] = P[j][i] = sym;
        }
    }
}

void ESKF::InjectError(const float dx[15])
{
    for (int i = 0; i < 3; i++) {
        p[i] += dx[i];
        v[i] += dx[i + 3];
        bg[i] += dx[i + 9];
        ba[i] += dx[i + 12];
    }
    float dq[4]    = {1.0f, 0.5f * dx[6], 0.5f * dx[7], 0.5f * dx[8]};
    float q_new[4] = {
        q[0] * dq[0] - q[1] * dq[1] - q[2] * dq[2] - q[3] * dq[3],
        q[0] * dq[1] + q[1] * dq[0] + q[2] * dq[3] - q[3] * dq[2],
        q[0] * dq[2] - q[1] * dq[3] + q[2] * dq[0] + q[3] * dq[1],
        q[0] * dq[3] + q[1] * dq[2] - q[2] * dq[1] + q[3] * dq[0]};
    float norm = sqrtf(q_new[0] * q_new[0] + q_new[1] * q_new[1] + q_new[2] * q_new[2] + q_new[3] * q_new[3]);
    for (int i = 0; i < 4; i++) q[i] = q_new[i] / norm;
}

// ---------------------- 观测接口 ----------------------
void ESKF::UpdateBaro(float alt)
{
    float H[15] = {0};
    H[2]        = -1.0f;                  // p[2] 是 NED 的 D (向下)，因此预测 alt 是 -p[2]
    ScalarUpdate(H, alt - (-p[2]), 0.5f); // 气压计方差 R=0.5
}

void ESKF::UpdateMag(const float mag[3])
{
    float Rmat[3][3];
    GetRotationMatrix(Rmat);
    float roll  = atan2f(Rmat[2][1], Rmat[2][2]);
    float pitch = asinf(-Rmat[2][0]);
    float yaw   = atan2f(Rmat[1][0], Rmat[0][0]);

    // 倾角补偿求绝对磁航向
    float cr = cosf(roll), sr = sinf(roll), cp = cosf(pitch), sp = sinf(pitch);
    float Xh       = mag[0] * cp + mag[1] * sr * sp + mag[2] * cr * sp;
    float Yh       = mag[1] * cr - mag[2] * sr;
    float meas_yaw = atan2f(-Yh, Xh);

    float y = meas_yaw - yaw;
    while (y > M_PI) y -= 2.0f * M_PI;
    while (y < -M_PI) y += 2.0f * M_PI;

    float H[15] = {0};
    H[6]        = Rmat[2][0];
    H[7]        = Rmat[2][1];
    H[8]        = Rmat[2][2];
    ScalarUpdate(H, y, 0.2f); // 磁力计方差 R=0.2
}

void ESKF::UpdateFlow(float dx, float dy, float dt)
{
    float z = -p[2]; // 对地高度
    if (z < 0.1f) z = 0.1f;

    // ★ 核心缩放系数: 像素与光流视场角换算率，需根据 PMW3901 镜头实测修改
    float OPT_FLOW_SCALER = 30.0f;
    float flow_wx         = (dx / dt) / OPT_FLOW_SCALER;
    float flow_wy         = (dy / dt) / OPT_FLOW_SCALER;

    // 抵消机身自转产生的虚假相对位移
    float vbx_obs = (flow_wy - last_gyro[1]) * z;
    float vby_obs = (flow_wx + last_gyro[0]) * z;

    float Rmat[3][3];
    GetRotationMatrix(Rmat);
    float vbx_pred = Rmat[0][0] * v[0] + Rmat[1][0] * v[1] + Rmat[2][0] * v[2];
    float vby_pred = Rmat[0][1] * v[0] + Rmat[1][1] * v[1] + Rmat[2][1] * v[2];
    float vbz_pred = Rmat[0][2] * v[0] + Rmat[1][2] * v[1] + Rmat[2][2] * v[2];

    // 分别进行 X 和 Y 轴的标量融合
    float Hx[15] = {0};
    Hx[3]        = Rmat[0][0];
    Hx[4]        = Rmat[1][0];
    Hx[5]        = Rmat[2][0];
    Hx[7]        = -vbz_pred;
    Hx[8]        = vby_pred;
    ScalarUpdate(Hx, vbx_obs - vbx_pred, 0.1f);

    float Hy[15] = {0};
    Hy[3]        = Rmat[0][1];
    Hy[4]        = Rmat[1][1];
    Hy[5]        = Rmat[2][1];
    Hy[6]        = vbz_pred;
    Hy[8]        = -vbx_pred;
    ScalarUpdate(Hy, vby_obs - vby_pred, 0.1f);
}

VehicleState_t ESKF::GetState(uint64_t timestamp)
{
    VehicleState_t s;
    s.timestamp = timestamp;
    float Rmat[3][3];
    GetRotationMatrix(Rmat);
    s.roll     = atan2f(Rmat[2][1], Rmat[2][2]);
    float sinp = -Rmat[2][0];
    if (sinp > 1.0f)
        sinp = 1.0f;
    else if (sinp < -1.0f)
        sinp = -1.0f;
    s.pitch = asinf(sinp);
    s.yaw   = atan2f(Rmat[1][0], Rmat[0][0]);

    for (int i = 0; i < 4; i++) s.q[i] = q[i];
    for (int i = 0; i < 3; i++) {
        s.position[i]   = p[i];
        s.velocity[i]   = v[i];
        s.accel_bias[i] = ba[i];
        s.gyro_bias[i]  = bg[i];
    }
    return s;
}

// ================= FreeRTOS 任务主干 =================
void AttitudeESKF_TaskEntry(void *argument)
{
    Sensor_Packet_t pkt;

    // --- 阶段1：静止开机校准，获取初始基准 ---
    int imu_cnt = 0, mag_cnt = 0, baro_cnt = 0;
    float sum_acc[3] = {0}, sum_mag[3] = {0}, sum_baro = 0;

    while (imu_cnt < 200) { // 等待约 0.2 秒收集基础环境数据
        if (xQueueReceive(SensorsDataHub, &pkt, portMAX_DELAY)) {
            if (pkt.tag == SENSOR_ID_ICM42688) {
                sum_acc[0] += pkt.data.ICM42688.acc[0];
                sum_acc[1] += pkt.data.ICM42688.acc[1];
                sum_acc[2] += pkt.data.ICM42688.acc[2];
                imu_cnt++;
            } else if (pkt.tag == SENSOR_ID_MMC5983) {
                sum_mag[0] += pkt.data.MMC5983.mag[0];
                sum_mag[1] += pkt.data.MMC5983.mag[1];
                sum_mag[2] += pkt.data.MMC5983.mag[2];
                mag_cnt++;
            } else if (pkt.tag == SENSOR_ID_DPS310) {
                sum_baro += pkt.data.DPS310.pressure;
                baro_cnt++;
            }
        }
    }

    float ax = sum_acc[0] / imu_cnt;
    if (ax > 1.0f)
        ax = 1.0f;
    else if (ax < -1.0f)
        ax = -1.0f;
    float pitch = asinf(ax);
    float roll  = atan2f(-(sum_acc[1] / imu_cnt), -(sum_acc[2] / imu_cnt));

    float initial_yaw = 0.0f;
    if (mag_cnt > 0) {
        float Xh    = (sum_mag[0] / mag_cnt) * cosf(pitch) + (sum_mag[1] / mag_cnt) * sinf(roll) * sinf(pitch) + (sum_mag[2] / mag_cnt) * cosf(roll) * sinf(pitch);
        float Yh    = (sum_mag[1] / mag_cnt) * cosf(roll) - (sum_mag[2] / mag_cnt) * sinf(roll);
        initial_yaw = atan2f(-Yh, Xh);
    }
    if (baro_cnt > 0) eskf.base_baro = sum_baro / baro_cnt;

    eskf.Init(initial_yaw, 0.0f);
    uint64_t last_imu_ts = 0, last_flow_ts = 0;

    // --- 阶段2：高速解算循环 ---
    while (1) {
        if (xQueueReceive(SensorsDataHub, &pkt, portMAX_DELAY)) {
            switch (pkt.tag) {
                case SENSOR_ID_ICM42688: { // IMU到达：触发预测并向外发布
                    if (last_imu_ts != 0) {
                        float dt = (pkt.timestamp - last_imu_ts) * 1e-9f;

                        float acc[3]  = {pkt.data.ICM42688.acc[0] * 9.80665f, pkt.data.ICM42688.acc[1] * 9.80665f, pkt.data.ICM42688.acc[2] * 9.80665f};
                        float gyro[3] = {pkt.data.ICM42688.gyro[0] * 0.01745329f, pkt.data.ICM42688.gyro[1] * 0.01745329f, pkt.data.ICM42688.gyro[2] * 0.01745329f};

                        // 1. 运动学预测 (状态推演)
                        eskf.PredictIMU(acc, gyro, dt);

                        // ★ 2. 加速度计姿态更新 (非常关键：锁定水平面，防止重力泄露)
                        eskf.UpdateAccel(acc);

                        // ★ 3. 桌面静态抗漂移 ZUPT (Zero Velocity Update) 逻辑
                        static int static_counter = 0;
                        float gyro_norm           = sqrtf(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);
                        float acc_norm            = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);

                        // 如果角速度极小 (<0.03 rad/s，约 1.7度/秒) 且加速度接近 1G，认为飞控板静止放在桌面上
                        if (gyro_norm < 0.03f && fabsf(acc_norm - 9.80665f) < 0.5f) {
                            static_counter++;
                            if (static_counter > 50) { // 连续约50ms静止，认定为完全静置
                                eskf.UpdateZUPT();
                            }
                        } else {
                            static_counter = 0; // 一旦有拿起来或晃动，立即解除静止锁定
                        }

                        // 打包当前解算结果并用 Overwrite 机制发布给控制器
                        VehicleState_t vs = eskf.GetState(pkt.timestamp);
                        if (VehicleStateQueue != NULL) xQueueOverwrite(VehicleStateQueue, &vs);

                        // 更新调试探针
                        DBG_MON_ATTITUDE_UPDATE(&vs);
                    }
                    last_imu_ts = pkt.timestamp;
                    break;
                }
                case SENSOR_ID_DPS310: { // 气压计到达：触发高度观测
                    float alt = 44330.0f * (1.0f - powf(pkt.data.DPS310.pressure / eskf.base_baro, 0.190295f));
                    eskf.UpdateBaro(alt);
                    break;
                }
                case SENSOR_ID_MMC5983: { // 磁力计到达：触发航向观测
                    eskf.UpdateMag(pkt.data.MMC5983.mag);
                    break;
                }
                case SENSOR_ID_PMW3901: { // 光流到达：触发XY速度观测
                    if (last_flow_ts != 0 && pkt.data.PMW3901.squal >= 20.0f) {
                        float dt = (pkt.timestamp - last_flow_ts) * 1e-9f;
                        if (dt > 0.0f && dt < 0.1f) {
                            eskf.UpdateFlow(pkt.data.PMW3901.delta_x, pkt.data.PMW3901.delta_y, dt);
                        }
                    }
                    last_flow_ts = pkt.timestamp;
                    break;
                }
                default:
                    break;
            }
        }
    }
}
void ESKF::UpdateAccel(const float acc[3])
{
    // 1. 扣除零偏求当前真实加速度的模长 (m/s^2)
    float true_acc[3] = {acc[0] - ba[0], acc[1] - ba[1], acc[2] - ba[2]};
    float norm        = sqrtf(true_acc[0] * true_acc[0] + true_acc[1] * true_acc[1] + true_acc[2] * true_acc[2]);

    // 2. 只有在无剧烈机动时才使用加速度计修正姿态 (约 0.8G ~ 1.2G)
    // 避免机动时的异常加速度干扰重力方向
    if (norm < 8.0f || norm > 11.5f) return;

    // 3. 归一化测量向量 (机体坐标系下的重力支持力方向向量)
    float v_meas[3] = {true_acc[0] / norm, true_acc[1] / norm, true_acc[2] / norm};

    // 4. 计算期望的归一化重力向量
    // (NED坐标系下重力向下为 [0, 0, 1]，转换到机体坐标系即为旋转矩阵的第三列转置。
    // 由于加速度计测到的是向上的支持力，方向与重力相反，所以取负)
    float Rmat[3][3];
    GetRotationMatrix(Rmat);
    float v_pred[3] = {-Rmat[0][2], -Rmat[1][2], -Rmat[2][2]};

    // 5. 分别对 X、Y、Z 三个轴进行雅可比标量降维更新
    float H[15] = {0};

    // X轴倾角更新
    H[6] = 0.0f;
    H[7] = v_pred[2];
    H[8] = -v_pred[1];
    ScalarUpdate(H, v_meas[0] - v_pred[0], 0.05f); // 0.05f 为观测方差

    // Y轴倾角更新
    memset(H, 0, sizeof(H));
    H[6] = -v_pred[2];
    H[7] = 0.0f;
    H[8] = v_pred[0];
    ScalarUpdate(H, v_meas[1] - v_pred[1], 0.05f);

    // Z轴倾角更新
    memset(H, 0, sizeof(H));
    H[6] = v_pred[1];
    H[7] = -v_pred[0];
    H[8] = 0.0f;
    ScalarUpdate(H, v_meas[2] - v_pred[2], 0.05f);
}

void ESKF::UpdateZUPT(void)
{
    float H[15] = {0};

    // 强制更新 X 轴速度为 0
    H[3] = 1.0f;
    ScalarUpdate(H, 0.0f - v[0], 0.005f); // 极小方差，强制收敛

    // 强制更新 Y 轴速度为 0
    memset(H, 0, sizeof(H));
    H[4] = 1.0f;
    ScalarUpdate(H, 0.0f - v[1], 0.005f);

    // 强制更新 Z 轴速度为 0
    memset(H, 0, sizeof(H));
    H[5] = 1.0f;
    ScalarUpdate(H, 0.0f - v[2], 0.005f);
}

void AttitudeEstimator_Init(void)
{
    ESKFTaskHandle = xTaskCreateStatic(
        AttitudeESKF_TaskEntry, "AttEst", 1024, NULL,
        configMAX_PRIORITIES - 3, // 优先级建议低于 Sensor 解析任务，高于未来其它控制逻辑
        ESKFTaskStack, &ESKFTaskTCB);
}