#pragma once
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define AXI_SRAM  __attribute__((aligned(32)))
#define DTCM_DATA __attribute__((section(".dtcm_data"))) __attribute__((aligned(4)))
#define DTCM_BSS  __attribute__((section(".dtcm_bss"))) __attribute__((aligned(4)))
#define FAST_BSS  __attribute__((section(".dtcm_bss"))) __attribute__((aligned(32)))

#ifdef __cplusplus
extern "C" {
#endif
#pragma pack(push, 1)

// PC -> Pi -> STM32 (控制帧, 严格 22 字节)
struct CommControlFrame {
    uint8_t  head[2];       // [0xAA, 0x55]
    uint8_t  seq;           // 🌟 序列号，替代原msg_id，用于测算延迟
    uint8_t  ctrl_mode;     // 0x01:正常飞行, 0x02:失控迫降
    float    target_vx;     
    float    target_vy;
    float    target_thrust; 
    float    target_yaw;
    uint16_t crc16;         // 涵盖 seq 到 target_yaw (18字节)
};

// STM32 -> Pi -> PC (遥测帧, 严格 22 字节)
struct CommTelemetryFrame {
    uint8_t  head[2];       // [0xBB, 0x66]
    uint8_t  msg_id;        // 0x10 (遥测帧特有的帧标识符，予以保留)
    uint8_t  ack_seq;       // 🌟 回弹确认号，替代原 _pad
    float    roll;
    float    pitch;
    float    yaw;
    float    pos_z;         
    uint16_t crc16;         // 涵盖 msg_id 到 pos_z (18字节)
};

#pragma pack(pop)
typedef enum {
    SENSOR_ID_NONE = 0,
    SENSOR_ID_DPS310,
    SENSOR_ID_PMW3901,
    SENSOR_ID_ICM42688,
    SENSOR_ID_MMC5983,
    SENSOR_COUNT
} SensorID_e;

#pragma pack(push, 4)
typedef struct {
    uint64_t timestamp_ns;
    float roll, pitch, yaw;
    float q[4];
    float position[3];
    float velocity[3];
    float accel_bias[3];
    float gyro_bias[3];
} VehicleState_t;

typedef struct {
    float target_roll;
    float target_pitch;
    float target_yaw_rate;
    float target_thrust;
} InnerSetpoint_t;

typedef struct {
    uint8_t ctrl_mode;
    uint8_t _padding[3];
    float target_vx;
    float target_vy;
    float target_vz;
    float target_yaw;
} OuterSetpoint_t;
#pragma pack(pop)

struct DeviceConfig {
    union {
        struct {
            GPIO_TypeDef *port;
            uint16_t pin;
            bool read_sets_bit;
        } spi;
        struct {
            uint16_t addr;
        } i2c;
    };
};

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#include "FreeRTOS.h"
#include "semphr.h"
#include "cmsis_os2.h"

class BusDriver
{
public:
    SemaphoreHandle_t lock;
    SemaphoreHandle_t dma_cplt_sem;

    void init();
    virtual bool transfer_blocking(const DeviceConfig &cfg, uint8_t reg, uint8_t *tx, uint8_t *rx, uint16_t len, bool is_read) = 0;
    virtual bool trigger_dma(const DeviceConfig &cfg, uint8_t reg, uint8_t *tx, uint8_t *rx, uint16_t len, bool is_read)       = 0;
    virtual void irq_handler()                                                                                                 = 0;
    bool wait_dma(uint32_t timeout_ms);
    virtual ~BusDriver() {}
};

class SPIBus : public BusDriver
{
private:
    SPI_HandleTypeDef *hspi;
    DeviceConfig current_cfg;

public:
    SPIBus(SPI_HandleTypeDef *h);
    bool transfer_blocking(const DeviceConfig &cfg, uint8_t reg, uint8_t *tx, uint8_t *rx, uint16_t len, bool is_read) override;
    bool trigger_dma(const DeviceConfig &cfg, uint8_t reg, uint8_t *tx, uint8_t *rx, uint16_t len, bool is_read) override;
    void irq_handler() override;
};

class I2CBus : public BusDriver
{
private:
    I2C_HandleTypeDef *hi2c;

public:
    I2CBus(I2C_HandleTypeDef *h);
    bool transfer_blocking(const DeviceConfig &cfg, uint8_t reg, uint8_t *tx, uint8_t *rx, uint16_t len, bool is_read) override;
    bool trigger_dma(const DeviceConfig &cfg, uint8_t reg, uint8_t *tx, uint8_t *rx, uint16_t len, bool is_read) override;
    void irq_handler() override;
};

class SensorBase
{
protected:
    SensorID_e _id;
    BusDriver *_bus;
    uint8_t *_tx_buf;
    uint8_t *_rx_buf;
    DeviceConfig _config;

public:
    SensorBase(SensorID_e id, BusDriver *bus, uint8_t *tx, uint8_t *rx);
    virtual ~SensorBase() {}
    bool write_reg_blocking(uint8_t reg, uint8_t val);
    bool read_regs_blocking(uint8_t reg, uint16_t len);
    bool write_regs_blocking(uint8_t reg, uint8_t *data, uint16_t len);
    bool trigger_read_dma(uint8_t reg, uint16_t len);
    bool wait_and_release(uint32_t timeout_ms);
};
#endif