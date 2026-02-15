#pragma once
#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "cmsis_os2.h"
#include "string.h"
#include "stdbool.h"

#define AXI_SRAM  __attribute__((aligned(32)))
#define DTCM_DATA __attribute__((section(".dtcm_data"))) alignas(4)
// 传感器 ID 枚举
typedef enum {
    SENSOR_ID_NONE = 0,
    SENSOR_ID_DPS310,
    SENSOR_ID_PMW3901,
    SENSOR_ID_ICM42688,
    SENSOR_ID_MMC5983,
    SENSOR_COUNT // 用于数组计数
} SensorID_e;

// 传感器数据包结构 (推送到队列的数据)
typedef struct {
    SensorID_e tag;
    uint64_t timestamp;
    union {
        struct {
            float pressure;
            float temperature;
        } DPS310;
        struct {
            float delta_x; // X轴相对位移计数值
            float delta_y; // Y轴相对位移计数值
            float squal;   // 表面质量指标 (用于数据有效性过滤)
        } PMW3901;
        struct {
            float acc[3];  // 20-bit 扩展后
            float gyro[3]; // 20-bit 扩展后
            float temp_raw;
        } ICM42688;
        struct {
            float mag[3];
        } MMC5983;
    } data;
} Sensor_Packet_t;

struct DeviceConfig {
    union {
        struct {
            GPIO_TypeDef *port;
            uint16_t pin;
            bool read_sets_bit; // 新增标志: true = 读置1/写清0 (标准); false = 读清0/写置1 (PMW3901)
        } spi;
        struct {
            uint16_t addr;
        } i2c;
    };
};

class SensorBase; // 前向声明

class BusDriver
{
public:
    SemaphoreHandle_t lock;
    SensorBase *active_device;
    bool is_read_op;
    BusDriver();
    virtual void init(); // 创建互斥量

    // 纯虚函数：统一传输接口
    virtual bool transfer(const DeviceConfig &cfg, uint8_t reg,
                          uint8_t *tx, uint8_t *rx,
                          uint16_t len, bool is_read) = 0;

    // 纯虚函数：中断处理
    virtual void irq_handler() = 0;
    virtual ~BusDriver()
    {
    }
};

// ==========================================
// 传感器基类
// ==========================================
class SensorBase
{
protected:
    // 家族遗产：子类直接使用
    SensorID_e _id;
    BusDriver *_bus;
    uint8_t *_tx_buf;
    uint8_t *_rx_buf;
    DeviceConfig _config;
    TaskHandle_t _data_task_handle = nullptr; // 存储对应的处理任务句柄

public:
    SensorBase(SensorID_e id, BusDriver *bus, uint8_t *tx, uint8_t *rx);
    virtual ~SensorBase()
    {
    }

    bool read_regs(uint8_t reg, uint16_t len);
    bool write_reg(uint8_t reg, uint8_t val);
    bool write_regs(uint8_t reg, uint8_t *data, uint16_t len);
    void set_task_handle(TaskHandle_t handle)
    {
        _data_task_handle = handle;
    } // 绑定唤醒任务句柄
    virtual void notify_task_ISR(); //
    virtual void process_in_task() = 0;
};

// ==========================================
// 具体总线类声明
// ==========================================
class SPIBus : public BusDriver
{
private:
    SPI_HandleTypeDef *hspi;
    DeviceConfig current_cfg; // 中断里用
public:
    SPIBus(SPI_HandleTypeDef *h);
    bool transfer(const DeviceConfig &cfg, uint8_t reg, uint8_t *tx, uint8_t *rx, uint16_t len, bool is_read) override;
    void irq_handler() override;
};

class I2CBus : public BusDriver
{
private:
    I2C_HandleTypeDef *hi2c;

public:
    I2CBus(I2C_HandleTypeDef *h);
    bool transfer(const DeviceConfig &cfg, uint8_t reg, uint8_t *tx, uint8_t *rx, uint16_t len, bool is_read) override;
    void irq_handler() override;
};
