#pragma once


#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "DataStructConfig.hpp"

#define ICM42688_DEVICE_CONFIG_ADDR 0x11 // 软复位与 SPI 模式配置
#define ICM42688_INT_CONFIG_ADDR    0x14 // INT1/INT2 引脚电平与模式配置
#define ICM42688_FIFO_DATA_ADDR     0x30 // FIFO 数据读出口 (DMA 突发读取地址)
#define ICM42688_PWR_MGMTO_ADDR     0x4E // 电源管理 (开启陀螺仪与加速计)
#define ICM42688_GYRO_CONFIG0_ADDR  0x4F // 陀螺仪 ODR 与量程
#define ICM42688_ACCEL_CONFIG0_ADDR 0x50 // 加速计 ODR 与量程
#define ICM42688_FIFO_CONFIG1_ADDR  0x5F // 启用 20-bit 高分辨率模式 (FIFO_HIRES_EN)
#define ICM42688_INT_SOURCE0_ADDR   0x65 // 中断源映射 (Data Ready 映射至 INT1)
#define ICM42688_WHO_AM_I_ADDR      0x75 // 器件ID验证 (应返回 0x47)
#define ICM42688_INTF_CONFIG5_ADDR  0x7B // 配置引脚 9 为 CLKIN (外部时钟输入)
#define ICM42688_REG_BANK_SEL_ADDR  0x76 // 切换 Bank 0/1/2/3/4

/*--------------------------------------*ICM42688regmap*--------------------------------------*/

#define MMC5983_REG_XOUT0      0x00
#define MMC5983_REG_STATUS     0x08
#define MMC5983_REG_CTRL0      0x09
#define MMC5983_REG_CTRL1      0x0A
#define MMC5983_REG_CTRL2      0x0B
#define MMC5983_REG_CTRL3      0x0C
#define MMC5983_REG_PRODUCT_ID 0x2F
/*--------------------------------------*MMC5983regmap*--------------------------------------*/

#define DPS310_PRS_B2     0x00
#define DPS310_TMP_B2     0x03
#define DPS310_PRS_CFG    0x06
#define DPS310_TMP_CFG    0x07
#define DPS310_MEAS_CFG   0x08
#define DPS310_CFG_REG    0x09
#define DPS310_RESET      0x0C
#define DPS310_COEF_START 0x10
#define DPS310_COEF_SRCE  0x28
/*--------------------------------------*DPS310regmap*--------------------------------------*/

#define PMW3901_REG_ID             0x00
#define PMW3901_REG_MOTION         0x02
#define PMW3901_REG_DELTA_X_L      0x03
#define PMW3901_REG_MOTION_BURST   0x16
#define PMW3901_REG_POWER_UP_RESET 0x3A

#define W(reg) ((reg) | 0x80)//PMW辅助写入宏
/*--------------------------------------*PMW3901regmap*--------------------------------------*/

class ICM42688 : public SensorBase
{
public:
    ICM42688(SensorID_e id, BusDriver *bus, uint8_t *tx, uint8_t *rx,
             GPIO_TypeDef *cs_port, uint16_t cs_pin);
    void init_regs();
    void read_fifo();
    void process_in_task() override;
private:
    int32_t parse_20bit(uint8_t h, uint8_t l, uint8_t ext, uint8_t shift);
};

// --- MMC5983 (I2C) ---
class MMC5983 : public SensorBase
{
public:
    MMC5983(SensorID_e id, BusDriver *bus, uint8_t *tx, uint8_t *rx, uint16_t i2c_addr);
    void init_regs();
    void read_mag();
    void process_in_task() override;
};

class DPS310 : public SensorBase
{
public:
    DPS310(SensorID_e id, BusDriver *bus, uint8_t *tx, uint8_t *rx,
           GPIO_TypeDef *cs_port, uint16_t cs_pin);
    void init_regs(); // 初始化任务中调用
    void read_data(); // 周期性调用 (如 32Hz)
    void process_in_task() override;
private:
    struct Coeffs {
        int32_t c0, c1;
        int32_t c00, c10, c01, c11, c20, c21, c30;
    } m_coeffs;
    float m_kp;
    float m_kt;
    int32_t get_twos_complement(uint32_t val, uint8_t length);
    void parse_coeffs();
    float compensate_pressure(int32_t raw_p, int32_t raw_t);
    float compensate_temperature(int32_t raw_t);
};

class PMW3901 : public SensorBase
{
public:
    // 构造函数
    PMW3901(SensorID_e id, BusDriver *bus, uint8_t *tx, uint8_t *rx,
            GPIO_TypeDef *cs_port, uint16_t cs_pin);

    // 接口函数
    void init_regs();           // 初始化序列
    void read_data();           // 触发 Burst 读取
    void process_in_task() override; // 数据解析

private:
    // 内部使用的寄存器配置结构
    struct RegCfg {
        uint8_t addr;
        uint8_t val;
    };

    // 静态优化参数序列 (存储在 Flash 中)
    static const RegCfg opt_seq_part1[];
    static const RegCfg opt_seq_part2[];

    // 批量写入辅助函数
    void write_block(const RegCfg* seq, size_t len);
};