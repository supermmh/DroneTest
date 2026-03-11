#include "Sensors.hpp"
#include "Mydelay.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

ICM42688::ICM42688(SPI_HandleTypeDef *spi, GPIO_TypeDef *port, uint16_t pin)
    : hspi(spi), cs_port(port), cs_pin(pin)
{
}

void ICM42688::write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = {(uint8_t)(reg & 0x7F), val};
    uint8_t rx[2] = {0};
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi, tx, rx, 2, 10);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
}

void ICM42688::flush_fifo()
{
    write_reg(0x16, 0x00);
    write_reg(0x4B, 0x02);
    delay_us(50);
    write_reg(0x16, 0x40);
}

void ICM42688::init_regs()
{
    write_reg(0x76, 0x00);
    delay_us(40000);
    write_reg(0x11, 0x01);
    delay_us(40000);
    write_reg(0x76, 0x00);
    delay_us(40000);
    write_reg(0x4F, 0x06);
    delay_us(40000);
    write_reg(0x50, 0x06);
    delay_us(40000);
    write_reg(0x14, 0x03);
    delay_us(40000);
    write_reg(0x64, 0x60);
    delay_us(40000);
    write_reg(0x65, 0x08);
    delay_us(40000);
    write_reg(0x5F, 0x17);
    delay_us(40000);
    write_reg(0x16, 0x40);
    delay_us(40000);
    write_reg(0x4E, 0x0F);
    delay_us(40000);
    flush_fifo();
}

int32_t ICM42688::parse_20bit(uint8_t h, uint8_t l, uint8_t ext, uint8_t shift)
{
    int32_t val = (int32_t)((h << 12) | (l << 4) | ((ext >> shift) & 0x0F));
    if (val & 0x80000) val |= 0xFFF00000;
    return val;
}

bool ICM42688::read_and_parse_blocking()
{
    uint8_t tx_buf[21] = {0};
    uint8_t rx_buf[21] = {0};
    tx_buf[0]          = ICM42688_FIFO_DATA_ADDR | 0x80;

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef res = HAL_SPI_TransmitReceive(hspi, tx_buf, rx_buf, 21, 2);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    if (res != HAL_OK || (rx_buf[1] & 0x80)) return false;

    uint8_t *p = &rx_buf[1];
    acc[0]     = (float)parse_20bit(p[1], p[2], p[17], 4) * 0.000299275f;
    acc[1]     = (float)parse_20bit(p[3], p[4], p[17], 0) * 0.000299275f;
    acc[2]     = (float)parse_20bit(p[5], p[6], p[18], 4) * 0.000299275f;
    gyro[0]    = (float)parse_20bit(p[7], p[8], p[18], 0) * 0.000066514f;
    gyro[1]    = (float)parse_20bit(p[9], p[10], p[19], 4) * 0.000066514f;
    gyro[2]    = (float)parse_20bit(p[11], p[12], p[19], 0) * 0.000066514f;
    temp       = ((float)(int16_t)((p[13] << 8) | p[14]) * 0.0075483f) + 25.0f;
    return true;
}

DPS310::DPS310(SensorID_e id, BusDriver *bus, uint8_t *tx, uint8_t *rx, GPIO_TypeDef *cs_port, uint16_t cs_pin) : SensorBase(id, bus, tx, rx)
{
    _config.spi.port          = cs_port;
    _config.spi.pin           = cs_pin;
    _config.spi.read_sets_bit = true;
    base_baro                 = 101325.0f;
}

int32_t DPS310::get_twos_complement(uint32_t val, uint8_t length)
{
    if (val & ((uint32_t)1 << (length - 1))) return (int32_t)(val - ((uint32_t)1 << length));
    return (int32_t)val;
}

void DPS310::parse_coeffs()
{
    uint8_t *b   = &_rx_buf[1];
    m_coeffs.c0  = get_twos_complement((b[0] << 4) | (b[1] >> 4), 12);
    m_coeffs.c1  = get_twos_complement(((b[1] & 0x0F) << 8) | b[2], 12);
    m_coeffs.c00 = get_twos_complement(((uint32_t)b[3] << 12) | ((uint32_t)b[4] << 4) | (b[5] >> 4), 20);
    m_coeffs.c10 = get_twos_complement(((uint32_t)(b[5] & 0x0F) << 16) | ((uint32_t)b[6] << 8) | b[7], 20);
    m_coeffs.c01 = get_twos_complement((b[8] << 8) | b[9], 16);
    m_coeffs.c11 = get_twos_complement((b[10] << 8) | b[11], 16);
    m_coeffs.c20 = get_twos_complement((b[12] << 8) | b[13], 16);
    m_coeffs.c21 = get_twos_complement((b[14] << 8) | b[15], 16);
    m_coeffs.c30 = get_twos_complement((b[16] << 8) | b[17], 16);
}

void DPS310::init_regs()
{
    write_reg_blocking(DPS310_RESET, 0x09);
    delay_us(40000);
    uint8_t ready = 0;
    int timeout   = 100;
    while ((ready & 0x80) == 0 && timeout > 0) {
        read_regs_blocking(DPS310_MEAS_CFG, 1);
        delay_us(10000);
        ready = _rx_buf[1];
        timeout--;
    }
    read_regs_blocking(DPS310_COEF_SRCE, 1);
    delay_us(10000);
    bool use_ext_temp = (_rx_buf[1] & 0x80) != 0;
    read_regs_blocking(DPS310_COEF_START, 18);
    delay_us(20000);
    parse_coeffs();
    write_reg_blocking(0x0E, 0xA5);
    write_reg_blocking(0x0F, 0x96);
    write_reg_blocking(0x62, 0x02);
    write_reg_blocking(0x0E, 0x00);
    write_reg_blocking(0x0F, 0x00);
    delay_us(10000);
    m_kp = 253952.0f;
    write_reg_blocking(DPS310_PRS_CFG, (0x05 << 4) | 0x04);
    delay_us(10000);
    m_kt = 1572864.0f;
    write_reg_blocking(DPS310_TMP_CFG, (0x05 << 4) | 0x01 | (use_ext_temp ? 0x80 : 0));
    delay_us(10000);
    write_reg_blocking(DPS310_CFG_REG, (1 << 2) | (1 << 4));
    delay_us(10000);
    write_reg_blocking(DPS310_MEAS_CFG, 0x07);
    delay_us(10000);
}

float DPS310::compensate_pressure(int32_t raw_p, int32_t raw_t)
{
    float p_sc = (float)raw_p / m_kp;
    float t_sc = (float)raw_t / m_kt;
    return m_coeffs.c00 + p_sc * (m_coeffs.c10 + p_sc * (m_coeffs.c20 + p_sc * m_coeffs.c30)) + t_sc * m_coeffs.c01 + t_sc * p_sc * (m_coeffs.c11 + p_sc * m_coeffs.c21);
}

bool DPS310::trigger_read()
{
    return trigger_read_dma(DPS310_PRS_B2, 6);
}

bool DPS310::wait_and_parse()
{
    if (!wait_and_release(5)) return false;
    SCB_InvalidateDCache_by_Addr((uint32_t *)_rx_buf, 32);
    int32_t raw_p = get_twos_complement(((_rx_buf[1] << 16) | (_rx_buf[2] << 8) | _rx_buf[3]), 24);
    int32_t raw_t = get_twos_complement(((_rx_buf[4] << 16) | (_rx_buf[5] << 8) | _rx_buf[6]), 24);
    if (raw_p == 0 || raw_p == 0x800000) return false;

    // 🌟 将气压赋值给成员变量，供外部 IDE 读取
    this->pressure = compensate_pressure(raw_p, raw_t);

    static int calib_cnt = 0;
    static float p_sum   = 0.0f;
    if (calib_cnt < 50) {
        p_sum += this->pressure;
        calib_cnt++;
        if (calib_cnt == 50) base_baro = p_sum / 50.0f;
        alt = 0.0f;
        return true;
    }

    alt = 44330.0f * (1.0f - powf(this->pressure / base_baro, 0.190295f));
    return true;
}

const PMW3901::RegCfg PMW3901::opt_seq_part1[] = {{0x7F, 0x00}, {0x61, 0xAD}, {0x7F, 0x03}, {0x40, 0x00}, {0x7F, 0x05}, {0x41, 0xB3}, {0x43, 0xF1}, {0x45, 0x14}, {0x5B, 0x32}, {0x5F, 0x34}, {0x7B, 0x08}, {0x7F, 0x06}, {0x44, 0x1B}, {0x40, 0xBF}, {0x4E, 0x3F}, {0x7F, 0x08}, {0x65, 0x20}, {0x6A, 0x18}, {0x7F, 0x09}, {0x4F, 0xAF}, {0x5F, 0x40}, {0x48, 0x80}, {0x49, 0x80}, {0x57, 0x77}, {0x60, 0x78}, {0x61, 0x78}, {0x62, 0x08}, {0x63, 0x50}, {0x7F, 0x0A}, {0x45, 0x60}, {0x7F, 0x00}, {0x4D, 0x11}, {0x55, 0x80}, {0x74, 0x1F}, {0x75, 0x1F}, {0x4A, 0x78}, {0x4B, 0x78}, {0x44, 0x08}, {0x45, 0x50}, {0x64, 0xFF}, {0x65, 0x1F}, {0x7F, 0x14}, {0x65, 0x67}, {0x66, 0x08}, {0x63, 0x70}, {0x7F, 0x15}, {0x48, 0x48}, {0x7F, 0x07}, {0x41, 0x0D}, {0x43, 0x14}, {0x4B, 0x0E}, {0x45, 0x0F}, {0x44, 0x42}, {0x4C, 0x80}, {0x7F, 0x10}, {0x5B, 0x02}, {0x7F, 0x07}, {0x40, 0x41}, {0x70, 0x00}};
const PMW3901::RegCfg PMW3901::opt_seq_part2[] = {{0x32, 0x44}, {0x7F, 0x07}, {0x40, 0x40}, {0x7F, 0x06}, {0x62, 0xF0}, {0x63, 0x00}, {0x7F, 0x0D}, {0x48, 0xC0}, {0x6F, 0xD5}, {0x7F, 0x00}, {0x5B, 0xA0}, {0x4E, 0xA8}, {0x5A, 0x50}, {0x40, 0x80}};
PMW3901::PMW3901(SensorID_e id, BusDriver *bus, uint8_t *tx, uint8_t *rx, GPIO_TypeDef *cs_port, uint16_t cs_pin) : SensorBase(id, bus, tx, rx)
{
    _config.spi.port          = cs_port;
    _config.spi.pin           = cs_pin;
    _config.spi.read_sets_bit = false;
}
void PMW3901::write_block(const RegCfg *seq, size_t len)
{
    for (size_t i = 0; i < len; i++) write_reg_blocking(seq[i].addr, seq[i].val);
}
void PMW3901::init_regs()
{
    delay_us(5000);
    read_regs_blocking(PMW3901_REG_ID, 1);
    read_regs_blocking(PMW3901_REG_MOTION, 5);
    write_reg_blocking(0x7F, 0x00);
    write_reg_blocking(0x55, 0x01);
    write_reg_blocking(0x50, 0x07);
    write_reg_blocking(0x7F, 0x0E);
    write_reg_blocking(0x43, 0x10);
    read_regs_blocking(0x67, 1);
    if (_rx_buf[1] & 0x80)
        write_reg_blocking(0x48, 0x04);
    else
        write_reg_blocking(0x48, 0x02);
    write_reg_blocking(0x7F, 0x00);
    write_reg_blocking(0x51, 0x7B);
    write_reg_blocking(0x50, 0x00);
    write_reg_blocking(0x55, 0x00);
    write_reg_blocking(0x7F, 0x0E);
    read_regs_blocking(0x73, 1);
    if (_rx_buf[1] == 0x00) {
        read_regs_blocking(0x70, 1);
        uint8_t c1 = _rx_buf[1];
        if (c1 <= 28)
            c1 += 14;
        else
            c1 += 11;
        if (c1 > 0x3F) c1 = 0x3F;
        read_regs_blocking(0x71, 1);
        uint8_t c2 = _rx_buf[1];
        c2         = (c2 * 45) / 100;
        write_reg_blocking(0x7F, 0x00);
        write_reg_blocking(0x61, 0xAD);
        write_reg_blocking(0x51, 0x70);
        write_reg_blocking(0x7F, 0x0E);
        write_reg_blocking(0x70, c1);
        write_reg_blocking(0x71, c2);
    }
    write_block(opt_seq_part1, sizeof(opt_seq_part1) / sizeof(RegCfg));
    delay_us(10000);
    write_block(opt_seq_part2, sizeof(opt_seq_part2) / sizeof(RegCfg));
}
bool PMW3901::trigger_read()
{
    return trigger_read_dma(PMW3901_REG_MOTION_BURST, 12);
}
bool PMW3901::wait_and_parse()
{
    if (!wait_and_release(5)) return false;
    SCB_InvalidateDCache_by_Addr((uint32_t *)_rx_buf, 32);
    uint8_t *p = &_rx_buf[1];
    dx         = (float)(int16_t)(p[2] | (p[3] << 8));
    dy         = (float)(int16_t)(p[4] | (p[5] << 8));
    squal      = (float)p[6];
    return true;
}

MMC5983::MMC5983(SensorID_e id, BusDriver *bus, uint8_t *tx, uint8_t *rx, uint16_t i2c_addr) : SensorBase(id, bus, tx, rx)
{
    _config.i2c.addr = (i2c_addr << 1);
}
void MMC5983::init_regs()
{
    write_reg_blocking(MMC5983_REG_CTRL1, 0x80);
    delay_us(15000);
    write_reg_blocking(MMC5983_REG_CTRL3, 0x00);
    delay_us(5000);
    write_reg_blocking(MMC5983_REG_CTRL0, 0x08);
    delay_us(10000);
}
void MMC5983::trigger_measure()
{
    write_reg_blocking(MMC5983_REG_CTRL0, 0x01);
}
bool MMC5983::trigger_read()
{
    return trigger_read_dma(MMC5983_REG_XOUT0, 7);
}
bool MMC5983::wait_and_parse()
{
    if (!wait_and_release(5)) return false;
    SCB_InvalidateDCache_by_Addr((uint32_t *)_rx_buf, 32);
    uint32_t x_raw = ((uint32_t)_rx_buf[0] << 10) | ((uint32_t)_rx_buf[1] << 2) | ((_rx_buf[6] & 0xC0) >> 6);
    uint32_t y_raw = ((uint32_t)_rx_buf[2] << 10) | ((uint32_t)_rx_buf[3] << 2) | ((_rx_buf[6] & 0x30) >> 4);
    uint32_t z_raw = ((uint32_t)_rx_buf[4] << 10) | ((uint32_t)_rx_buf[5] << 2) | ((_rx_buf[6] & 0x0C) >> 2);
    // 🌟 保留修复：左手系转右手系，强行适应飞控矩阵解算
    mag[0] = ((float)x_raw - 131072.0f) / 16384.0f;
    mag[1] = -(((float)y_raw - 131072.0f) / 16384.0f);
    mag[2] = ((float)z_raw - 131072.0f) / 16384.0f;
    return true;
}