#pragma once
#include "main.h"

#ifdef __cplusplus
#include "DataStructConfig.hpp"

#define DPS310_PRS_B2               0x00
#define DPS310_MEAS_CFG             0x08
#define DPS310_CFG_REG              0x09
#define DPS310_RESET                0x0C
#define DPS310_COEF_START           0x10
#define DPS310_COEF_SRCE            0x28
#define DPS310_PRS_CFG              0x06
#define DPS310_TMP_CFG              0x07

#define PMW3901_REG_ID              0x00
#define PMW3901_REG_MOTION          0x02
#define PMW3901_REG_MOTION_BURST    0x16

#define ICM42688_DEVICE_CONFIG_ADDR 0x11
#define ICM42688_INT_CONFIG_ADDR    0x14
#define ICM42688_FIFO_DATA_ADDR     0x30
#define ICM42688_PWR_MGMTO_ADDR     0x4E
#define ICM42688_GYRO_CONFIG0_ADDR  0x4F
#define ICM42688_ACCEL_CONFIG0_ADDR 0x50
#define ICM42688_FIFO_CONFIG1_ADDR  0x5F
#define ICM42688_INT_SOURCE0_ADDR   0x65
#define ICM42688_REG_BANK_SEL_ADDR  0x76
#define ICM42688_FIFO_CONFIG_ADDR   0x16
#define ICM42688_INT_CONFIG1_ADDR   0x64

#define MMC5983_REG_XOUT0           0x00
#define MMC5983_REG_CTRL0           0x09
#define MMC5983_REG_CTRL1           0x0A
#define MMC5983_REG_CTRL3           0x0C

class DPS310 : public SensorBase
{
public:
    float alt;
    float pressure; // 🌟 暴露原生气压值
    DPS310(SensorID_e id, BusDriver *bus, uint8_t *tx, uint8_t *rx, GPIO_TypeDef *cs_port, uint16_t cs_pin);
    void init_regs();
    bool trigger_read();
    bool wait_and_parse();

private:
    struct {
        int32_t c0, c1, c00, c10, c01, c11, c20, c21, c30;
    } m_coeffs;
    float m_kp, m_kt, base_baro;
    int32_t get_twos_complement(uint32_t val, uint8_t length);
    void parse_coeffs();
    float compensate_pressure(int32_t raw_p, int32_t raw_t);
};

class PMW3901 : public SensorBase
{
public:
    float dx, dy, squal;
    PMW3901(SensorID_e id, BusDriver *bus, uint8_t *tx, uint8_t *rx, GPIO_TypeDef *cs_port, uint16_t cs_pin);
    void init_regs();
    bool trigger_read();
    bool wait_and_parse();

private:
    struct RegCfg {
        uint8_t addr;
        uint8_t val;
    };
    static const RegCfg opt_seq_part1[];
    static const RegCfg opt_seq_part2[];
    void write_block(const RegCfg *seq, size_t len);
};

class MMC5983 : public SensorBase
{
public:
    float mag[3];
    MMC5983(SensorID_e id, BusDriver *bus, uint8_t *tx, uint8_t *rx, uint16_t i2c_addr);
    void init_regs();
    void trigger_measure();
    bool trigger_read();
    bool wait_and_parse();
};

class ICM42688
{
private:
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    int32_t parse_20bit(uint8_t h, uint8_t l, uint8_t ext, uint8_t shift);
    void write_reg(uint8_t reg, uint8_t val);

public:
    float acc[3];//当前安装失误，标准状态为前X右Y下Z，目前为前Y右-X下Z
    float gyro[3];//当前安装失误，标准状态为前X右Y下Z，目前为前Y右-X下Z
    float temp;
    ICM42688(SPI_HandleTypeDef *spi, GPIO_TypeDef *port, uint16_t pin);
    void init_regs();
    void flush_fifo();
    bool read_and_parse_blocking();
};
#endif