#include "TaskConfig.hpp"
#include "DataStructConfig.hpp"
#include "spi.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "Sensors.hpp"
#include "FlightBroker.hpp"
#include "AttitudeESKF.hpp"
#include "DebugMonitor.hpp"
#include "Mydelay.hpp"
#include <math.h>
#include <string.h>
#include "tim.h"
#include "FlightController.hpp"

#define ENABLE_OPT_FLOW  1
#define ENABLE_MAG       1
#define ENABLE_BARO      1

#define UART_RX_BUF_SIZE 1024
#define UART_TX_BUF_SIZE 64

AXI_SRAM uint8_t uart_rx_buf[UART_RX_BUF_SIZE] __attribute__((aligned(32)));
AXI_SRAM uint8_t uart_tx_buf[UART_TX_BUF_SIZE] __attribute__((aligned(32)));

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart4;

SPIBus spi1_bus(&hspi1);
SPIBus spi2_bus(&hspi2);
I2CBus i2c1_bus(&hi2c1);

AXI_SRAM uint8_t SPI1_Tx[32], SPI1_Rx[32];
AXI_SRAM uint8_t SPI2_Tx[32], SPI2_Rx[32];
AXI_SRAM uint8_t I2C1_Tx[32], I2C1_Rx[32];

DPS310 dps310(SENSOR_ID_DPS310, &spi1_bus, SPI1_Tx, SPI1_Rx, GPIOA, GPIO_PIN_4);
PMW3901 pmw3901(SENSOR_ID_PMW3901, &spi2_bus, SPI2_Tx, SPI2_Rx, GPIOB, GPIO_PIN_12);
MMC5983 mmc5983(SENSOR_ID_MMC5983, &i2c1_bus, I2C1_Tx, I2C1_Rx, 0x30);
ICM42688 icm42688(&hspi3, GPIOD, GPIO_PIN_2);

FAST_BSS FlightBroker g_FlightBroker;
DTCM_BSS ESKF eskf;

TaskHandle_t T0_FastLoop_Handle;
void *T0_FastLoop_Handle_Ptr = NULL;
TaskHandle_t T1_NavLoop_Handle;
TaskHandle_t T2_Comms_Handle;
void *T2_Comms_Handle_Ptr = NULL;
TaskHandle_t T3_Monitor_Handle;

DTCM_BSS StackType_t T0_Stack[1024];
DTCM_BSS StaticTask_t T0_TCB;
DTCM_BSS StackType_t T1_Stack[1024];
DTCM_BSS StaticTask_t T1_TCB;
DTCM_BSS StackType_t T2_Stack[1024];
DTCM_BSS StaticTask_t T2_TCB;
DTCM_BSS StackType_t T3_Stack[1024];
DTCM_BSS StaticTask_t T3_TCB;

static uint16_t calc_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc & 0xFFFF;
}

void T0_FastLoop_Entry(void *argument)
{
    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int calib_cnt = 0;

    icm42688.flush_fifo();
    while (calib_cnt < 100) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
        if (icm42688.read_and_parse_blocking()) calib_cnt++;
    }

    calib_cnt = 0;
    while (calib_cnt < 1000) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
        if (icm42688.read_and_parse_blocking()) {
            sum_ax += icm42688.acc[1];
            sum_ay += -icm42688.acc[0];
            sum_az += icm42688.acc[2];
            sum_gx += icm42688.gyro[1];
            sum_gy += -icm42688.gyro[0];
            sum_gz += icm42688.gyro[2];
            calib_cnt++;
        }
    }

    if (calib_cnt > 0) {
        float acc_mean[3]  = {sum_ax / calib_cnt, sum_ay / calib_cnt, sum_az / calib_cnt};
        float gyro_mean[3] = {sum_gx / calib_cnt, sum_gy / calib_cnt, sum_gz / calib_cnt};
        taskENTER_CRITICAL();
        eskf.AlignAndCalibrate(acc_mean, gyro_mean, 0.0f);
        taskEXIT_CRITICAL();
    }

    uint64_t last_time = Get_System_Time_ns();
    uint32_t accel_div = 0, stationary_counter = 0;
    uint32_t imu_death_cnt = 0;

    icm42688.flush_fifo();

    while (1) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2));
        uint64_t now = Get_System_Time_ns();

        DBG_MON_ATTEMPT(SENSOR_ID_ICM42688);
        if (!icm42688.read_and_parse_blocking()) {
            DBG_MON_ERROR(SENSOR_ID_ICM42688);
            if (++imu_death_cnt > 5) {
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);
                __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1000);
                __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 1000);
                g_FlightController.reset_integrals();
            }
            continue;
        }
        imu_death_cnt = 0;
        DBG_MON_SUCCESS(SENSOR_ID_ICM42688);

        float dt  = (now - last_time) * 1e-9f;
        last_time = now;
        if (dt > 0.02f || dt <= 0.0f) dt = 0.001f;

        float a_frd[3] = {icm42688.acc[1], -icm42688.acc[0], icm42688.acc[2]};    // 注意修改
        float g_frd[3] = {icm42688.gyro[1], -icm42688.gyro[0], icm42688.gyro[2]}; // 注意修改

        taskENTER_CRITICAL();
        VehicleState_t tmp_state = eskf.GetState(now);
        float true_gx            = g_frd[0] - tmp_state.gyro_bias[0];
        float true_gy            = g_frd[1] - tmp_state.gyro_bias[1];
        float true_gz            = g_frd[2] - tmp_state.gyro_bias[2];

        float norm_a = sqrtf(a_frd[0] * a_frd[0] + a_frd[1] * a_frd[1] + a_frd[2] * a_frd[2]);
        float norm_g = sqrtf(true_gx * true_gx + true_gy * true_gy + true_gz * true_gz);

        if (fabsf(norm_a - 9.80665f) < 0.4f && norm_g < 0.02f)
            stationary_counter++;
        else
            stationary_counter = 0;

        eskf.PredictIMU(a_frd, g_frd, dt);

        if (++accel_div >= 10) {
            accel_div = 0;
            eskf.UpdateAccel(a_frd);
            if (stationary_counter > 500) {
                eskf.UpdateZUPT();
                if (stationary_counter > 10000) stationary_counter = 1000;
            }
        }
        VehicleState_t state = eskf.GetState(now);
        taskEXIT_CRITICAL();

        g_FlightBroker.vehicle_state.publish(state);
        DBG_MON_ATTITUDE_UPDATE(&state);

        OuterSetpoint_t out_sp;
        uint32_t out_ver;
        uint64_t out_time;
        g_FlightBroker.outer_setpoint.copy_to(out_sp, out_time, out_ver, 0);

        InnerSetpoint_t in_sp;
        uint32_t in_ver;
        uint64_t in_time;
        g_FlightBroker.inner_setpoint.copy_to(in_sp, in_time, in_ver, 0);

        if (now - out_time > 150000000ULL) {
            out_sp.ctrl_mode = 0x02;
        }

        float rates[3]   = {true_gx, true_gy, true_gz};
        uint16_t pwms[4] = {1000, 1000, 1000, 1000};

        g_FlightController.update(out_sp, in_sp, state, rates, dt, pwms);

        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwms[0]);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwms[1]);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwms[2]);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwms[3]);
    }
}

void T1_NavLoop_Entry(void *argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t loop_cnt        = 0;

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
        loop_cnt++;

        bool p_req = false, d_req = false, m_req = false;

#if ENABLE_MAG
        if (loop_cnt % 2 == 0) mmc5983.trigger_measure();//连续读取有bug，单次读取
#endif

#if ENABLE_OPT_FLOW
        DBG_MON_ATTEMPT(SENSOR_ID_PMW3901); // 💡 压入探针
        p_req = pmw3901.trigger_read();
        if (!p_req) DBG_MON_ERROR(SENSOR_ID_PMW3901);
#endif

        if (loop_cnt % 2 == 0) {
#if ENABLE_BARO
            DBG_MON_ATTEMPT(SENSOR_ID_DPS310);
            d_req = dps310.trigger_read();
            if (!d_req) DBG_MON_ERROR(SENSOR_ID_DPS310);
#endif
#if ENABLE_MAG
            DBG_MON_ATTEMPT(SENSOR_ID_MMC5983);
            m_req = mmc5983.trigger_read();
            if (!m_req) DBG_MON_ERROR(SENSOR_ID_MMC5983);
#endif
        }

#if ENABLE_OPT_FLOW
        if (p_req) {
            if (pmw3901.wait_and_parse()) {
                DBG_MON_SUCCESS(SENSOR_ID_PMW3901);
                // 🌟 推入调试器观察位
                g_DebugMonitor.raw_sensors.pmw_dx    = pmw3901.dx;
                g_DebugMonitor.raw_sensors.pmw_dy    = pmw3901.dy;
                g_DebugMonitor.raw_sensors.pmw_squal = pmw3901.squal;

                if (pmw3901.squal > 20.0f) {
                    taskENTER_CRITICAL();
                    eskf.UpdateFlow(pmw3901.dx, pmw3901.dy, 0.02f);
                    taskEXIT_CRITICAL();
                }
            } else {
                DBG_MON_ERROR(SENSOR_ID_PMW3901);
            }
        }
#endif

        if (loop_cnt % 2 == 0) {
#if ENABLE_BARO
            if (d_req) {
                if (dps310.wait_and_parse()) {
                    DBG_MON_SUCCESS(SENSOR_ID_DPS310);
                    // 🌟 推入调试器观察位
                    g_DebugMonitor.raw_sensors.dps_pressure = dps310.pressure;
                    taskENTER_CRITICAL();
                    eskf.UpdateBaro(dps310.alt, 1.0f);
                    taskEXIT_CRITICAL();
                } else {
                    DBG_MON_ERROR(SENSOR_ID_DPS310);
                }
            }
#endif
#if ENABLE_MAG
            if (m_req) {
                if (mmc5983.wait_and_parse()) {
                    DBG_MON_SUCCESS(SENSOR_ID_MMC5983);
                    // 🌟 推入调试器观察位
                    g_DebugMonitor.raw_sensors.mmc_mag_x = mmc5983.mag[0];
                    g_DebugMonitor.raw_sensors.mmc_mag_y = mmc5983.mag[1];
                    g_DebugMonitor.raw_sensors.mmc_mag_z = mmc5983.mag[2];
                    taskENTER_CRITICAL();
                    eskf.UpdateMag(mmc5983.mag);
                    taskEXIT_CRITICAL();
                } else {
                    DBG_MON_ERROR(SENSOR_ID_MMC5983);
                }
            }
#endif
        }
    }
}

void T2_Comms_Entry(void *argument)
{
    T2_Comms_Handle_Ptr = (void *)xTaskGetCurrentTaskHandle();
    __HAL_UART_CLEAR_OREFLAG(&huart4);
    __HAL_UART_CLEAR_FEFLAG(&huart4);
    __HAL_UART_CLEAR_NEFLAG(&huart4);
    __HAL_UART_CLEAR_PEFLAG(&huart4);
    __HAL_UART_CLEAR_IDLEFLAG(&huart4);
    volatile uint32_t dummy = huart4.Instance->RDR;
    (void)dummy;
    huart4.ErrorCode = HAL_UART_ERROR_NONE;

    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uart_rx_buf, UART_RX_BUF_SIZE);

    uint16_t rx_read_pos = 0;
    uint8_t frame_buf[sizeof(CommControlFrame)];
    uint16_t frame_idx    = 0;
    int sync_state        = 0;
    uint64_t last_tx_time = Get_System_Time_ns();

    static uint8_t current_seq = 0; // 🌟 记住 PC 发来的序号

    while (1) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2));
        uint64_t now = Get_System_Time_ns();

        if (now - last_tx_time >= 1900000ULL) {
            last_tx_time = now;
            if (huart4.gState != HAL_UART_STATE_READY) {
                HAL_UART_AbortTransmit(&huart4);
                huart4.gState = HAL_UART_STATE_READY;
            }

            if (huart4.gState == HAL_UART_STATE_READY) {
                VehicleState_t state;
                uint64_t ts;
                uint32_t ver;
                g_FlightBroker.vehicle_state.copy_to(state, ts, ver, 0);

                CommTelemetryFrame tx_frame;
                tx_frame.head[0] = 0xBB;
                tx_frame.head[1] = 0x66;
                tx_frame.msg_id  = 0x10;
                tx_frame.ack_seq = current_seq; // 🌟 瞬间将收到的序列号回弹给 PC
                tx_frame.roll    = state.roll;
                tx_frame.pitch   = state.pitch;
                tx_frame.yaw     = state.yaw;
                tx_frame.pos_z   = state.position[2];
                // 🌟 msg_id 依旧是本帧起点，crc16 计算完美兼容
                tx_frame.crc16 = calc_crc16((uint8_t *)&tx_frame.msg_id, sizeof(CommTelemetryFrame) - 4);

                memcpy(uart_tx_buf, &tx_frame, sizeof(CommTelemetryFrame));
                SCB_CleanDCache_by_Addr((uint32_t *)uart_tx_buf, 32);

                if (HAL_UART_Transmit_DMA(&huart4, uart_tx_buf, sizeof(CommTelemetryFrame)) == HAL_OK) {
                    g_DebugMonitor.uart4.tx_frames++;
                }
            }
        }

        if (huart4.ErrorCode != HAL_UART_ERROR_NONE || huart4.RxState != HAL_UART_STATE_BUSY_RX) {
            g_DebugMonitor.uart4.rx_restarts++;
            __HAL_UART_CLEAR_OREFLAG(&huart4);
            __HAL_UART_CLEAR_FEFLAG(&huart4);
            __HAL_UART_CLEAR_NEFLAG(&huart4);
            __HAL_UART_CLEAR_PEFLAG(&huart4);
            HAL_UART_AbortReceive(&huart4);
            huart4.ErrorCode = HAL_UART_ERROR_NONE;

            rx_read_pos = 0;
            frame_idx   = 0;
            sync_state  = 0;
            vTaskDelay(pdMS_TO_TICKS(2));
            HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uart_rx_buf, UART_RX_BUF_SIZE);
            continue;
        }

        SCB_InvalidateDCache_by_Addr((uint32_t *)uart_rx_buf, UART_RX_BUF_SIZE);
        uint16_t rx_write_pos = UART_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart4.hdmarx);
        if (rx_write_pos >= UART_RX_BUF_SIZE) rx_write_pos = 0;

        while (rx_read_pos != rx_write_pos) {
            uint8_t b = uart_rx_buf[rx_read_pos];
            if (++rx_read_pos >= UART_RX_BUF_SIZE) rx_read_pos = 0;

            if (sync_state == 0) {
                if (b == 0xAA) {
                    frame_buf[frame_idx++] = b;
                    sync_state             = 1;
                }
            } else if (sync_state == 1) {
                if (b == 0x55) {
                    frame_buf[frame_idx++] = b;
                    sync_state             = 2;
                } else if (b == 0xAA) {
                    frame_idx = 1;
                } else {
                    frame_idx  = 0;
                    sync_state = 0;
                }
            } else if (sync_state == 2) {
                frame_buf[frame_idx++] = b;
                if (frame_idx == sizeof(CommControlFrame)) {
                    CommControlFrame *rx = (CommControlFrame *)frame_buf;
                    // 🌟 注意：校验的起点已经从 msg_id 变为了 rx->seq
                    if (calc_crc16((uint8_t *)&rx->seq, sizeof(CommControlFrame) - 4) == rx->crc16) {
                        g_DebugMonitor.uart4.rx_valid_frames++;

                        current_seq = rx->seq; // 🌟 截获序列号，用于反弹

                        OuterSetpoint_t sp = {0};
                        sp.ctrl_mode       = rx->ctrl_mode;
                        sp.target_vx       = rx->target_vx;
                        sp.target_vy       = rx->target_vy;
                        sp.target_yaw      = rx->target_yaw;
                        g_FlightBroker.outer_setpoint.publish(sp);

                        InnerSetpoint_t in_sp = {0};
                        uint32_t ver;
                        uint64_t ts;
                        g_FlightBroker.inner_setpoint.copy_to(in_sp, ts, ver, 0);
                        in_sp.target_thrust = rx->target_thrust;
                        g_FlightBroker.inner_setpoint.publish(in_sp);
                    }
                    frame_idx  = 0;
                    sync_state = 0;
                }
            }
        }
    }
}

void T3_Monitor_Entry(void *argument)
{
    while (1) { vTaskDelay(pdMS_TO_TICKS(100)); }
}

void System_Init_Task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(500));
    spi1_bus.init();
    spi2_bus.init();
    i2c1_bus.init();
    dps310.init_regs();
#if ENABLE_OPT_FLOW
    pmw3901.init_regs();
#endif
    icm42688.init_regs();
#if ENABLE_MAG
    mmc5983.init_regs();
#endif
    eskf.Init(0.0f, 0.0f);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1000);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 1000);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    vTaskDelay(pdMS_TO_TICKS(1500));

    T0_FastLoop_Handle     = xTaskCreateStatic(T0_FastLoop_Entry, "T0", 1024, NULL, configMAX_PRIORITIES - 1, T0_Stack, &T0_TCB);
    T0_FastLoop_Handle_Ptr = (void *)T0_FastLoop_Handle;
    T1_NavLoop_Handle      = xTaskCreateStatic(T1_NavLoop_Entry, "T1", 1024, NULL, configMAX_PRIORITIES - 2, T1_Stack, &T1_TCB);
    T2_Comms_Handle        = xTaskCreateStatic(T2_Comms_Entry, "T2", 1024, NULL, configMAX_PRIORITIES - 3, T2_Stack, &T2_TCB);
    T2_Comms_Handle_Ptr    = (void *)T2_Comms_Handle;
    T3_Monitor_Handle      = xTaskCreateStatic(T3_Monitor_Entry, "T3", 1024, NULL, configMAX_PRIORITIES - 4, T3_Stack, &T3_TCB);

    DBG_MON_INIT();
    vTaskDelete(NULL);
}

void Bridge_System_Init(void)
{
    xTaskCreate(System_Init_Task, "SysInit", 512, NULL, configMAX_PRIORITIES - 1, NULL);
}