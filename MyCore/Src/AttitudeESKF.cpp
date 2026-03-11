#include "AttitudeESKF.hpp"
#include "FreeRTOS.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define IDX(row, col) ((row) * 15 + (col))

DTCM_BSS static float32_t F_data[225], FT_data[225], FP_data[225], P_next_data[225];
DTCM_BSS static float32_t HT_data[15], PHT_data[15], HPHT_data[1], K_data[15];
DTCM_BSS static float32_t KH_data[225], I_KH_data[225], I_KH_T_data[225];
DTCM_BSS static float32_t TMP_data[225], KT_data[15], KKT_data[225];

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
    p[2] = -initial_alt;
    q[0] = cosf(initial_yaw * 0.5f);
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = sinf(initial_yaw * 0.5f);

    memset(P_data, 0, sizeof(P_data));
    for (int i = 0; i < 15; i++) { P_data[IDX(i, i)] = (i < 9) ? 1e-2f : 1e-4f; }
    arm_mat_init_f32(&P_mat, 15, 15, P_data);
    is_inited = true;
}

void ESKF::AlignAndCalibrate(const float acc_mean[3], const float gyro_mean[3], float init_yaw)
{
    bg[0] = gyro_mean[0];
    bg[1] = gyro_mean[1];
    bg[2] = gyro_mean[2];
    ba[0] = 0.0f;
    ba[1] = 0.0f;
    ba[2] = 0.0f;

    float ax = acc_mean[0], ay = acc_mean[1], az = acc_mean[2];
    float norm_a = sqrtf(ax * ax + ay * ay + az * az);
    if (norm_a > 1e-3f) {
        ax /= norm_a;
        ay /= norm_a;
        az /= norm_a;
    }

    float pitch = asinf(ax), roll = atan2f(-ay, -az), yaw = init_yaw;
    float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f), sy = sinf(yaw * 0.5f);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;

    memset(v, 0, sizeof(v));
    p[0] = 0.0f;
    p[1] = 0.0f;
}

void ESKF::SetStaticBias(const float accel_bias[3], const float gyro_bias[3])
{
    ba[0] = accel_bias[0];
    ba[1] = accel_bias[1];
    ba[2] = accel_bias[2];
    bg[0] = gyro_bias[0];
    bg[1] = gyro_bias[1];
    bg[2] = gyro_bias[2];
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
    float true_acc[3]  = {acc[0] - ba[0], acc[1] - ba[1], acc[2] - ba[2]};
    float true_gyro[3] = {gyro[0] - bg[0], gyro[1] - bg[1], gyro[2] - bg[2]};
    last_gyro[0]       = true_gyro[0];
    last_gyro[1]       = true_gyro[1];
    last_gyro[2]       = true_gyro[2];

    float Rmat[3][3];
    GetRotationMatrix(Rmat);
    float a_ned[3] = {
        Rmat[0][0] * true_acc[0] + Rmat[0][1] * true_acc[1] + Rmat[0][2] * true_acc[2],
        Rmat[1][0] * true_acc[0] + Rmat[1][1] * true_acc[1] + Rmat[1][2] * true_acc[2],
        Rmat[2][0] * true_acc[0] + Rmat[2][1] * true_acc[1] + Rmat[2][2] * true_acc[2] + 9.80665f};

    for (int i = 0; i < 3; i++) {
        p[i] += v[i] * dt + 0.5f * a_ned[i] * dt * dt;
        v[i] += a_ned[i] * dt;
    }

    float theta = sqrtf(true_gyro[0] * true_gyro[0] + true_gyro[1] * true_gyro[1] + true_gyro[2] * true_gyro[2]) * dt;
    float dq[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    if (theta > 1e-6f && !isnan(theta) && !isinf(theta)) {
        float s = sinf(theta * 0.5f) / (theta / dt);
        dq[0]   = cosf(theta * 0.5f);
        dq[1]   = true_gyro[0] * s;
        dq[2]   = true_gyro[1] * s;
        dq[3]   = true_gyro[2] * s;
    } else {
        dq[1] = true_gyro[0] * dt * 0.5f;
        dq[2] = true_gyro[1] * dt * 0.5f;
        dq[3] = true_gyro[2] * dt * 0.5f;
    }
    float q_new[4] = {
        q[0] * dq[0] - q[1] * dq[1] - q[2] * dq[2] - q[3] * dq[3], q[0] * dq[1] + q[1] * dq[0] + q[2] * dq[3] - q[3] * dq[2],
        q[0] * dq[2] - q[1] * dq[3] + q[2] * dq[0] + q[3] * dq[1], q[0] * dq[3] + q[1] * dq[2] - q[2] * dq[1] + q[3] * dq[0]};
    float norm = sqrtf(q_new[0] * q_new[0] + q_new[1] * q_new[1] + q_new[2] * q_new[2] + q_new[3] * q_new[3]);
    if (!isnan(norm) && norm > 1e-6f) {
        for (int i = 0; i < 4; i++) q[i] = q_new[i] / norm;
    }

    memset(F_data, 0, sizeof(F_data));
    for (int i = 0; i < 15; i++) F_data[IDX(i, i)] = 1.0f;
    for (int i = 0; i < 3; i++) F_data[IDX(i, i + 3)] = dt;

    float skew_a[3][3] = {{0, -true_acc[2], true_acc[1]}, {true_acc[2], 0, -true_acc[0]}, {-true_acc[1], true_acc[0], 0}};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            F_data[IDX(3 + i, 6 + j)]  = -(Rmat[i][0] * skew_a[0][j] + Rmat[i][1] * skew_a[1][j] + Rmat[i][2] * skew_a[2][j]) * dt;
            F_data[IDX(3 + i, 12 + j)] = -Rmat[i][j] * dt;
        }
    }
    F_data[IDX(6, 7)]  = true_gyro[2] * dt;
    F_data[IDX(6, 8)]  = -true_gyro[1] * dt;
    F_data[IDX(7, 6)]  = -true_gyro[2] * dt;
    F_data[IDX(7, 8)]  = true_gyro[0] * dt;
    F_data[IDX(8, 6)]  = true_gyro[1] * dt;
    F_data[IDX(8, 7)]  = -true_gyro[0] * dt;
    F_data[IDX(6, 9)]  = -dt;
    F_data[IDX(7, 10)] = -dt;
    F_data[IDX(8, 11)] = -dt;

    arm_matrix_instance_f32 mat_F = {15, 15, F_data}, mat_FT = {15, 15, FT_data}, mat_FP = {15, 15, FP_data}, mat_P_next = {15, 15, P_next_data};
    arm_mat_trans_f32(&mat_F, &mat_FT);
    arm_mat_mult_f32(&mat_F, &P_mat, &mat_FP);
    arm_mat_mult_f32(&mat_FP, &mat_FT, &mat_P_next);
    memcpy(P_data, P_next_data, sizeof(P_data));

    for (int i = 0; i < 3; i++) P_data[IDX(i, i)] += 1e-6f * dt;
    for (int i = 3; i < 6; i++) P_data[IDX(i, i)] += 1e-2f * dt;
    for (int i = 6; i < 9; i++) P_data[IDX(i, i)] += 1e-4f * dt;
    for (int i = 9; i < 12; i++) P_data[IDX(i, i)] += 1e-6f * dt;
    for (int i = 12; i < 15; i++) P_data[IDX(i, i)] += 1e-5f * dt;

    for (int i = 0; i < 15; i++) {
        if (isnan(P_data[IDX(i, i)]) || P_data[IDX(i, i)] > 10.0f) {
            P_data[IDX(i, i)] = 10.0f;
            for (int j = 0; j < 15; j++) {
                if (i != j) {
                    P_data[IDX(i, j)] = 0.0f;
                    P_data[IDX(j, i)] = 0.0f;
                }
            }
        }
        if (P_data[IDX(i, i)] < 1e-7f) P_data[IDX(i, i)] = 1e-7f;

        for (int j = i + 1; j < 15; j++) {
            float sym = 0.5f * (P_data[IDX(i, j)] + P_data[IDX(j, i)]);
            if (isnan(sym) || isinf(sym)) sym = 0.0f;
            P_data[IDX(i, j)] = P_data[IDX(j, i)] = sym;
        }
    }
}

void ESKF::ScalarUpdate(const float H_in[15], float y, float R_var, bool use_chisq)
{
    if (isnan(y) || isinf(y)) return;
    float32_t H_data[15];
    memcpy(H_data, H_in, sizeof(H_data));
    arm_matrix_instance_f32 mat_H = {1, 15, H_data}, mat_HT = {15, 1, HT_data}, mat_PHT = {15, 1, PHT_data}, mat_HPHT = {1, 1, HPHT_data};

    arm_mat_trans_f32(&mat_H, &mat_HT);
    arm_mat_mult_f32(&P_mat, &mat_HT, &mat_PHT);
    arm_mat_mult_f32(&mat_H, &mat_PHT, &mat_HPHT);
    float S = HPHT_data[0] + R_var;
    if (S < 1e-6f || isnan(S) || isinf(S)) return;

    if (use_chisq && ((y * y) > 25.0f * S)) return;

    float inv_S = 1.0f / S, dx[15];
    for (int i = 0; i < 15; i++) {
        K_data[i] = PHT_data[i] * inv_S;
        dx[i]     = K_data[i] * y;
    }
    InjectError(dx);

    arm_matrix_instance_f32 mat_K = {15, 1, K_data}, mat_KH = {15, 15, KH_data};
    arm_mat_mult_f32(&mat_K, &mat_H, &mat_KH);

    memset(I_KH_data, 0, sizeof(I_KH_data));
    for (int i = 0; i < 225; i++) I_KH_data[i] = -KH_data[i];
    for (int i = 0; i < 15; i++) I_KH_data[IDX(i, i)] += 1.0f;

    arm_matrix_instance_f32 mat_I_KH = {15, 15, I_KH_data}, mat_I_KH_T = {15, 15, I_KH_T_data}, mat_TMP = {15, 15, TMP_data}, mat_P_new = {15, 15, P_next_data};
    arm_mat_trans_f32(&mat_I_KH, &mat_I_KH_T);
    arm_mat_mult_f32(&mat_I_KH, &P_mat, &mat_TMP);
    arm_mat_mult_f32(&mat_TMP, &mat_I_KH_T, &mat_P_new);

    arm_matrix_instance_f32 mat_KT = {1, 15, KT_data}, mat_KKT = {15, 15, KKT_data};
    arm_mat_trans_f32(&mat_K, &mat_KT);
    arm_mat_mult_f32(&mat_K, &mat_KT, &mat_KKT);

    for (int i = 0; i < 225; i++) P_data[i] = P_next_data[i] + KKT_data[i] * R_var;

    for (int i = 0; i < 15; i++) {
        if (isnan(P_data[IDX(i, i)]) || P_data[IDX(i, i)] > 10.0f) {
            P_data[IDX(i, i)] = 10.0f;
            for (int j = 0; j < 15; j++) {
                if (i != j) {
                    P_data[IDX(i, j)] = 0.0f;
                    P_data[IDX(j, i)] = 0.0f;
                }
            }
        }
        if (P_data[IDX(i, i)] < 1e-7f) P_data[IDX(i, i)] = 1e-7f;

        for (int j = i + 1; j < 15; j++) {
            float sym = 0.5f * (P_data[IDX(i, j)] + P_data[IDX(j, i)]);
            if (isnan(sym) || isinf(sym)) sym = 0.0f;
            P_data[IDX(i, j)] = P_data[IDX(j, i)] = sym;
        }
    }
}

void ESKF::InjectError(const float dx[15])
{
    for (int i = 0; i < 15; i++)
        if (isnan(dx[i]) || isinf(dx[i])) return;

    for (int i = 0; i < 3; i++) {
        p[i] += dx[i];
        v[i] += dx[i + 3];
        bg[i] += dx[i + 9];
        ba[i] += dx[i + 12];
        if (bg[i] > 0.15f)
            bg[i] = 0.15f;
        else if (bg[i] < -0.15f)
            bg[i] = -0.15f;
        if (ba[i] > 1.5f)
            ba[i] = 1.5f;
        else if (ba[i] < -1.5f)
            ba[i] = -1.5f;
    }
    float dq[4]    = {1.0f, 0.5f * dx[6], 0.5f * dx[7], 0.5f * dx[8]};
    float q_new[4] = {
        q[0] * dq[0] - q[1] * dq[1] - q[2] * dq[2] - q[3] * dq[3], q[0] * dq[1] + q[1] * dq[0] + q[2] * dq[3] - q[3] * dq[2],
        q[0] * dq[2] - q[1] * dq[3] + q[2] * dq[0] + q[3] * dq[1], q[0] * dq[3] + q[1] * dq[2] - q[2] * dq[1] + q[3] * dq[0]};
    float norm = sqrtf(q_new[0] * q_new[0] + q_new[1] * q_new[1] + q_new[2] * q_new[2] + q_new[3] * q_new[3]);
    if (!isnan(norm) && norm > 1e-6f) {
        for (int i = 0; i < 4; i++) q[i] = q_new[i] / norm;
    }
}

void ESKF::UpdateBaro(float alt, float r_var)
{
    float H[15] = {0};
    H[2]        = -1.0f;
    ScalarUpdate(H, alt - (-p[2]), r_var, false);
}

void ESKF::UpdateMag(const float mag[3])
{
    float Rmat[3][3];
    GetRotationMatrix(Rmat);
    float roll = atan2f(Rmat[2][1], Rmat[2][2]), pitch = asinf(-Rmat[2][0]), yaw = atan2f(Rmat[1][0], Rmat[0][0]);
    float cr = cosf(roll), sr = sinf(roll), cp = cosf(pitch), sp = sinf(pitch);
    float Xh = mag[0] * cp + mag[1] * sr * sp + mag[2] * cr * sp, Yh = mag[1] * cr - mag[2] * sr;
    float meas_yaw = atan2f(-Yh, Xh);

    float y = meas_yaw - yaw;
    if (isnan(y) || isinf(y)) return;
    while (y > M_PI) y -= 2.0f * M_PI;
    while (y < -M_PI) y += 2.0f * M_PI;

    float H[15] = {0};
    H[6]        = Rmat[2][0];
    H[7]        = Rmat[2][1];
    H[8]        = Rmat[2][2];
    ScalarUpdate(H, y, 0.2f, false);
}

void ESKF::UpdateFlow(float dx, float dy, float dt)
{
    float z_cg = -p[2];
    if (z_cg < 0.1f) z_cg = 0.1f;
    if (z_cg > 35.0f) z_cg = 35.0f; // 🌟 保留修复：光流高空限幅保护

    // 🌟 保留修复：光流杆臂效应物理补偿 (目前为0，安装偏移时填入物理卷尺读数)
    const float offset_x = 0.0f;
    const float offset_y = 0.0f;
    const float offset_z = 0.0f;

    float z_sensor = z_cg + offset_z;
    if (z_sensor < 0.1f) z_sensor = 0.1f;
    if (z_sensor > 8.0f) z_sensor = 8.0f;

    float OPT_FLOW_SCALER = 30.0f;
    float raw_vbx_sensor  = ((dy / dt) / OPT_FLOW_SCALER - last_gyro[1]) * z_sensor;
    float raw_vby_sensor  = ((dx / dt) / OPT_FLOW_SCALER + last_gyro[0]) * z_sensor;

    float vbx_obs = raw_vbx_sensor - (last_gyro[1] * offset_z - last_gyro[2] * offset_y);
    float vby_obs = raw_vby_sensor - (last_gyro[2] * offset_x - last_gyro[0] * offset_z);

    float Rmat[3][3], H[15];

    GetRotationMatrix(Rmat);
    float vbx_pred = Rmat[0][0] * v[0] + Rmat[1][0] * v[1] + Rmat[2][0] * v[2];
    float vby_pred = Rmat[0][1] * v[0] + Rmat[1][1] * v[1] + Rmat[2][1] * v[2];
    float vbz_pred = Rmat[0][2] * v[0] + Rmat[1][2] * v[1] + Rmat[2][2] * v[2];

    memset(H, 0, sizeof(H));
    H[3] = Rmat[0][0];
    H[4] = Rmat[1][0];
    H[5] = Rmat[2][0];
    H[7] = -vbz_pred;
    H[8] = vby_pred;
    ScalarUpdate(H, vbx_obs - vbx_pred, 0.1f, true);

    GetRotationMatrix(Rmat);
    vbx_pred = Rmat[0][0] * v[0] + Rmat[1][0] * v[1] + Rmat[2][0] * v[2];
    vby_pred = Rmat[0][1] * v[0] + Rmat[1][1] * v[1] + Rmat[2][1] * v[2];
    vbz_pred = Rmat[0][2] * v[0] + Rmat[1][2] * v[1] + Rmat[2][2] * v[2];

    memset(H, 0, sizeof(H));
    H[3] = Rmat[0][1];
    H[4] = Rmat[1][1];
    H[5] = Rmat[2][1];
    H[6] = vbz_pred;
    H[8] = -vbx_pred;
    ScalarUpdate(H, vby_obs - vby_pred, 0.1f, true);
}

void ESKF::UpdateAccel(const float acc[3])
{
    float true_acc[3] = {acc[0] - ba[0], acc[1] - ba[1], acc[2] - ba[2]};
    float norm        = sqrtf(true_acc[0] * true_acc[0] + true_acc[1] * true_acc[1] + true_acc[2] * true_acc[2]);
    if (norm < 7.0f || norm > 13.0f || isnan(norm) || isinf(norm)) return;
    float v_meas[3] = {true_acc[0] / norm, true_acc[1] / norm, true_acc[2] / norm};

    float dyn_R = 0.05f + fabsf(norm - 9.80665f) * 2.0f;
    float Rmat[3][3], v_pred[3], H[15];

    GetRotationMatrix(Rmat);
    // 🌟 保留修复：将 NED 重力投影到机体，必须提取第三【列】(Column 2)
    v_pred[0] = -Rmat[0][2];
    v_pred[1] = -Rmat[1][2];
    v_pred[2] = -Rmat[2][2];

    memset(H, 0, sizeof(H));
    H[6] = 0.0f;
    H[7] = -v_pred[2];
    H[8] = v_pred[1];
    ScalarUpdate(H, v_meas[0] - v_pred[0], dyn_R, false);

    GetRotationMatrix(Rmat);
    v_pred[0] = -Rmat[0][2];
    v_pred[1] = -Rmat[1][2];
    v_pred[2] = -Rmat[2][2];
    memset(H, 0, sizeof(H));
    H[6] = v_pred[2];
    H[7] = 0.0f;
    H[8] = -v_pred[0];
    ScalarUpdate(H, v_meas[1] - v_pred[1], dyn_R, false);

    GetRotationMatrix(Rmat);
    v_pred[0] = -Rmat[0][2];
    v_pred[1] = -Rmat[1][2];
    v_pred[2] = -Rmat[2][2];
    memset(H, 0, sizeof(H));
    H[6] = -v_pred[1];
    H[7] = v_pred[0];
    H[8] = 0.0f;
    ScalarUpdate(H, v_meas[2] - v_pred[2], dyn_R, false);
}

void ESKF::UpdateZUPT(void)
{
    float H[15] = {0};
    H[3]        = 1.0f;
    ScalarUpdate(H, 0.0f - v[0], 0.05f, false);
    memset(H, 0, sizeof(H));
    H[4] = 1.0f;
    ScalarUpdate(H, 0.0f - v[1], 0.05f, false);
    memset(H, 0, sizeof(H));
    H[5] = 1.0f;
    ScalarUpdate(H, 0.0f - v[2], 0.05f, false);
}

VehicleState_t ESKF::GetState(uint64_t timestamp)
{
    VehicleState_t s;
    s.timestamp_ns = timestamp;
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