#include "Sensors.hpp"
#include "QueueConfig.hpp"
#include "DebugMonitor.hpp"
#include "Mydelay.hpp"
DPS310::DPS310(SensorID_e id, BusDriver *bus, uint8_t *tx, uint8_t *rx,
               GPIO_TypeDef *cs_port, uint16_t cs_pin)
    : SensorBase(id, bus, tx, rx)
{
    _config.spi.port          = cs_port;
    _config.spi.pin           = cs_pin;
    _config.spi.read_sets_bit = true; // 标准模式: 读置1/写清0
}
void DPS310::init_regs()
{
    write_reg(DPS310_RESET, 0x09);
    vTaskDelay(pdMS_TO_TICKS(40));

    uint8_t ready = 0;
    while ((ready & 0x80) == 0) {
        read_regs(DPS310_MEAS_CFG, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
        ready = _rx_buf[1]; // SPI 必须跳过第 0 字节(Dummy)
    }
    read_regs(DPS310_COEF_SRCE, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    uint8_t coef_srce = _rx_buf[1]; // 必须是 _rx_buf[1]
    bool use_ext_temp = (coef_srce & 0x80) != 0;

    // 读取系数并解析
    read_regs(DPS310_COEF_START, 18);
    vTaskDelay(pdMS_TO_TICKS(20));
    parse_coeffs();

    // 3. 【新增】解决 Infineon 芯片温度读取异常的官方隐藏序列 (Erratum)
    write_reg(0x0E, 0xA5);
    write_reg(0x0F, 0x96);
    write_reg(0x62, 0x02);
    write_reg(0x0E, 0x00);
    write_reg(0x0F, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 后续的 PRS_CFG 等保持你的原样即可
    uint8_t prs_cfg = (0x05 << 4) | 0x04;
    m_kp            = 253952.0f;
    write_reg(DPS310_PRS_CFG, prs_cfg);
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t tmp_cfg = (0x05 << 4) | 0x01;
    if (use_ext_temp) tmp_cfg |= 0x80;
    m_kt = 1572864.0f;
    write_reg(DPS310_TMP_CFG, tmp_cfg);
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t cfg_reg = (1 << 2) | (1 << 4);
    write_reg(DPS310_CFG_REG, cfg_reg);
    vTaskDelay(pdMS_TO_TICKS(10));

    write_reg(DPS310_MEAS_CFG, 0x07);
    vTaskDelay(pdMS_TO_TICKS(10));
}
bool DPS310::read_data()
{
    return read_regs(DPS310_PRS_B2, 6);
}
void DPS310::process_in_task()
{
    Sensor_Packet_t packet;
    // 1. 解析原始数据 (24-bit 补码)
    int32_t raw_p = get_twos_complement(
        ((uint32_t)_rx_buf[1] << 16) | ((uint32_t)_rx_buf[2] << 8) | _rx_buf[3], 24);

    int32_t raw_t = get_twos_complement(
        ((uint32_t)_rx_buf[4] << 16) | ((uint32_t)_rx_buf[5] << 8) | _rx_buf[6], 24);

    if (raw_p == 0 || raw_p == 0x800000) {
        DBG_MON_ERROR(_id);
        return;
    }

    // 2. 执行补偿计算 (转为 float)
    float pressure    = compensate_pressure(raw_p, raw_t);
    float temperature = compensate_temperature(raw_t);

    packet.tag                     = _id;
    packet.timestamp               = Get_System_Time_ns(); // 或者使用微秒级时间戳
    packet.data.DPS310.pressure    = pressure;             // 单位: Pa
    packet.data.DPS310.temperature = temperature;          // 单位: Celsius

    // 4. 推送到队列 (非阻塞发送，保持实时性)
    if (SensorsDataHub != NULL) {
        xQueueSend(SensorsDataHub, &packet, 0);
        DBG_MON_SUCCESS(_id, &packet);
    }
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

int32_t DPS310::get_twos_complement(uint32_t val, uint8_t length)
{
    if (val & ((uint32_t)1 << (length - 1))) {
        return (int32_t)(val - ((uint32_t)1 << length));
    }
    return (int32_t)val;
}
// 压力补偿公式 [cite: 358]
float DPS310::compensate_pressure(int32_t raw_p, int32_t raw_t)
{
    float p_sc = (float)raw_p / m_kp;
    float t_sc = (float)raw_t / m_kt;

    // Pcomp = c00 + P_sc*(c10 + P_sc*(c20 + P_sc*c30)) + T_sc*c01 + T_sc*P_sc*(c11 + P_sc*c21)
    float term_p  = m_coeffs.c10 + p_sc * (m_coeffs.c20 + p_sc * m_coeffs.c30);
    float term_t  = t_sc * m_coeffs.c01;
    float term_tp = t_sc * p_sc * (m_coeffs.c11 + p_sc * m_coeffs.c21);

    return m_coeffs.c00 + p_sc * term_p + term_t + term_tp;
}

// 温度补偿公式 [cite: 377]
float DPS310::compensate_temperature(int32_t raw_t)
{
    float t_sc = (float)raw_t / m_kt;
    // Tcomp = c0 * 0.5 + c1 * T_sc
    return (float)m_coeffs.c0 * 0.5f + (float)m_coeffs.c1 * t_sc;
}

// 第一阶段序列 (在 10ms 延时之前)
const PMW3901::RegCfg PMW3901::opt_seq_part1[] = {
    {0x7F, 0x00}, {0x61, 0xAD}, {0x7F, 0x03}, {0x40, 0x00}, {0x7F, 0x05}, {0x41, 0xB3}, {0x43, 0xF1}, {0x45, 0x14}, {0x5B, 0x32}, {0x5F, 0x34}, {0x7B, 0x08}, {0x7F, 0x06}, {0x44, 0x1B}, {0x40, 0xBF}, {0x4E, 0x3F}, {0x7F, 0x08}, {0x65, 0x20}, {0x6A, 0x18}, {0x7F, 0x09}, {0x4F, 0xAF}, {0x5F, 0x40}, {0x48, 0x80}, {0x49, 0x80}, {0x57, 0x77}, {0x60, 0x78}, {0x61, 0x78}, {0x62, 0x08}, {0x63, 0x50}, {0x7F, 0x0A}, {0x45, 0x60}, {0x7F, 0x00}, {0x4D, 0x11}, {0x55, 0x80}, {0x74, 0x1F}, {0x75, 0x1F}, {0x4A, 0x78}, {0x4B, 0x78}, {0x44, 0x08}, {0x45, 0x50}, {0x64, 0xFF}, {0x65, 0x1F}, {0x7F, 0x14}, {0x65, 0x67}, {0x66, 0x08}, {0x63, 0x70}, {0x7F, 0x15}, {0x48, 0x48}, {0x7F, 0x07}, {0x41, 0x0D}, {0x43, 0x14}, {0x4B, 0x0E}, {0x45, 0x0F}, {0x44, 0x42}, {0x4C, 0x80}, {0x7F, 0x10}, {0x5B, 0x02}, {0x7F, 0x07}, {0x40, 0x41}, {0x70, 0x00}};

// 第二阶段序列 (在 10ms 延时之后)
const PMW3901::RegCfg PMW3901::opt_seq_part2[] = {
    {0x32, 0x44}, {0x7F, 0x07}, {0x40, 0x40}, {0x7F, 0x06}, {0x62, 0xF0}, {0x63, 0x00}, {0x7F, 0x0D}, {0x48, 0xC0}, {0x6F, 0xD5}, {0x7F, 0x00}, {0x5B, 0xA0}, {0x4E, 0xA8}, {0x5A, 0x50}, {0x40, 0x80}};

// =================================================================================
// 2. 函数实现
// =================================================================================

PMW3901::PMW3901(SensorID_e id, BusDriver *bus, uint8_t *tx, uint8_t *rx,
                 GPIO_TypeDef *cs_port, uint16_t cs_pin)
    : SensorBase(id, bus, tx, rx)
{
    _config.spi.port          = cs_port;
    _config.spi.pin           = cs_pin;
    _config.spi.read_sets_bit = false;
}

void PMW3901::write_block(const RegCfg *seq, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        // 直接写入，底层驱动会根据 read_sets_bit=false 自动处理 MSB
        write_reg(seq[i].addr, seq[i].val);
    }
}

void PMW3901::init_regs()
{
    vTaskDelay(pdMS_TO_TICKS(5)); // 等待复位完成

    // 读取 Product ID (0x00) 验证 SPI 通讯 (应返回 0x49)
    read_regs(PMW3901_REG_ID, 1);

    // 读取寄存器 0x02-0x06 一次 (Datasheet 要求初始化必须读一次)
    read_regs(PMW3901_REG_MOTION, 5);

    write_reg(0x7F, 0x00);
    write_reg(0x55, 0x01);
    write_reg(0x50, 0x07);
    write_reg(0x7F, 0x0E);
    write_reg(0x43, 0x10);

    // 检查 0x67 位 7 (Datasheet 分支逻辑)
    read_regs(0x67, 1);
    if (_rx_buf[1] & 0x80) {
        write_reg(0x48, 0x04);
    } else {
        write_reg(0x48, 0x02);
    }

    write_reg(0x7F, 0x00);
    write_reg(0x51, 0x7B);
    write_reg(0x50, 0x00);
    write_reg(0x55, 0x00);
    write_reg(0x7F, 0x0E);

    // 检查 0x73 (Datasheet 分支逻辑)
    read_regs(0x73, 1);
    if (_rx_buf[1] == 0x00) {
        // C1/C2 计算逻辑
        read_regs(0x70, 1);
        uint8_t c1 = _rx_buf[1];
        if (c1 <= 28)
            c1 += 14;
        else
            c1 += 11;
        if (c1 > 0x3F) c1 = 0x3F; // Cap at 0x3F

        read_regs(0x71, 1);
        uint8_t c2 = _rx_buf[1];
        c2         = (c2 * 45) / 100;

        write_reg(0x7F, 0x00);
        write_reg(0x61, 0xAD);
        write_reg(0x51, 0x70);
        write_reg(0x7F, 0x0E);
        write_reg(0x70, c1);
        write_reg(0x71, c2);
    }

    // --- 3. 性能优化序列 (静态部分 1) ---
    write_block(opt_seq_part1, sizeof(opt_seq_part1) / sizeof(RegCfg));

    vTaskDelay(pdMS_TO_TICKS(10));

    // --- 4. 性能优化序列 (静态部分 2) ---
    write_block(opt_seq_part2, sizeof(opt_seq_part2) / sizeof(RegCfg));

    // 初始化完成
}

bool PMW3901::read_data()
{
    return read_regs(PMW3901_REG_MOTION_BURST, 12);
}

void PMW3901::process_in_task()
{
    Sensor_Packet_t packet;
    uint8_t *p                  = &_rx_buf[1];
    int16_t raw_dx              = (int16_t)(p[2] | (p[3] << 8));
    int16_t raw_dy              = (int16_t)(p[4] | (p[5] << 8));
    uint8_t squal               = p[6];
    packet.tag                  = _id;
    packet.timestamp            = Get_System_Time_ns();
    packet.data.PMW3901.delta_x = (float)raw_dx;
    packet.data.PMW3901.delta_y = (float)raw_dy;
    packet.data.PMW3901.squal   = (float)squal; // 表面纹理质量
    if (SensorsDataHub != NULL) {
        xQueueSend(SensorsDataHub, &packet, 0);
        DBG_MON_SUCCESS(_id, &packet);
    }
}

ICM42688::ICM42688(SensorID_e id, BusDriver *bus, uint8_t *tx, uint8_t *rx,
                   GPIO_TypeDef *cs_port, uint16_t cs_pin)
    : SensorBase(id, bus, tx, rx)
{
    _config.spi.port          = cs_port;
    _config.spi.pin           = cs_pin;
    _config.spi.read_sets_bit = true; // 标准模式: 读置1/写清0
}
void ICM42688::init_regs()
{
    write_reg(ICM42688_REG_BANK_SEL_ADDR, 0x00);
    vTaskDelay(pdMS_TO_TICKS(40));

    write_reg(ICM42688_DEVICE_CONFIG_ADDR, 0x01); // 触发软复位
    vTaskDelay(pdMS_TO_TICKS(40));

    // 3. 返回 Bank 0 进行主配置
    write_reg(ICM42688_REG_BANK_SEL_ADDR, 0x00); // 切换回 Bank 0
    vTaskDelay(pdMS_TO_TICKS(40));

    // 4. 配置 ODR 和量程
    // 陀螺仪: 2000dps, 1kHz (0x01)
    write_reg(ICM42688_GYRO_CONFIG0_ADDR, 0x06);
    vTaskDelay(pdMS_TO_TICKS(40));

    // 加速计: 16g, 1kHz (0x01)
    write_reg(ICM42688_ACCEL_CONFIG0_ADDR, 0x06);
    vTaskDelay(pdMS_TO_TICKS(40));

    write_reg(ICM42688_INT_CONFIG_ADDR, 0x03);
    vTaskDelay(pdMS_TO_TICKS(40));

    write_reg(ICM42688_INT_CONFIG1_ADDR, 0x60);
    vTaskDelay(pdMS_TO_TICKS(40));

    // 5. 配置中断引脚：将 UI Data Ready 路由至 INT1
    write_reg(ICM42688_INT_SOURCE0_ADDR, 0x08);
    vTaskDelay(pdMS_TO_TICKS(40));

    // 6. 启用 20-bit 高分辨率模式及传感器 FIFO 写入
    write_reg(ICM42688_FIFO_CONFIG1_ADDR, 0x17);
    vTaskDelay(pdMS_TO_TICKS(40));

    write_reg(ICM42688_FIFO_CONFIG_ADDR, 0x40);
    vTaskDelay(pdMS_TO_TICKS(40));

    // 7. 开启 6 轴低噪声模式 (LN Mode)
    write_reg(ICM42688_PWR_MGMTO_ADDR, 0x0F);
    vTaskDelay(pdMS_TO_TICKS(40));

    // 启动通常需要 30ms，等待传感器稳定
}
bool ICM42688::read_fifo()
{
    // 在 EXTI 触发的 FreeRTOS 任务中调用此函数
    return read_regs(ICM42688_FIFO_DATA_ADDR, 20);
}

void ICM42688::process_in_task()
{
    // _rx_buf[0] 是 SPI 指令回显，真实数据从 _rx_buf[1] 开始 [cite: 1363, 1364]
    uint8_t *p = &_rx_buf[1];

    // 检查 FIFO Header (Packet 4 结构)
    // 如果 Bit 7 为 1，代表 FIFO 为空 [cite: 923]
    if (p[0] & 0x80) {
        DBG_MON_ERROR(_id);
        return;
    }

    Sensor_Packet_t packet;

    /* --- 20-bit 数据解析与物理量转换 --- */
    int32_t raw_ax = parse_20bit(p[1], p[2], p[17], 4);
    int32_t raw_ay = parse_20bit(p[3], p[4], p[17], 0);
    int32_t raw_az = parse_20bit(p[5], p[6], p[18], 4);

    int32_t raw_gx = parse_20bit(p[7], p[8], p[18], 0);
    int32_t raw_gy = parse_20bit(p[9], p[10], p[19], 4);
    int32_t raw_gz = parse_20bit(p[11], p[12], p[19], 0);

    int16_t raw_temp = (int16_t)((p[13] << 8) | p[14]);

    /* 物理量转换逻辑 */
    packet.tag       = _id;
    packet.timestamp = Get_System_Time_ns(); // 使用 DWT 周期计数器作为高精度时间戳
    // 加速度计：20-bit 模式下，±16g 量程的灵敏度为 8192 LSB/g
    packet.data.ICM42688.acc[0] = (float)raw_ax / 32768.0f;
    packet.data.ICM42688.acc[1] = (float)raw_ay / 32768.0f;
    packet.data.ICM42688.acc[2] = (float)raw_az / 32768.0f;

    // 陀螺仪：16-bit 时为 131 LSB/dps，20-bit 模式数据左移了 4 位，故灵敏度为 131 * 16 = 2096 LSB/dps [cite: 881, 165]
    packet.data.ICM42688.gyro[0] = (float)raw_gx / 262.4f;
    packet.data.ICM42688.gyro[1] = (float)raw_gy / 262.4f;
    packet.data.ICM42688.gyro[2] = (float)raw_gz / 262.4f;

    // 温度：公式为 (TEMP_DATA / 132.48) + 25

    packet.data.ICM42688.temp_raw = ((float)raw_temp / 132.48f) + 25.0f;

    // 发送到传感器数据总线
    if (SensorsDataHub != NULL) {
        xQueueSend(SensorsDataHub, &packet, 0);
        DBG_MON_SUCCESS(_id, &packet);
    }
}

int32_t ICM42688::parse_20bit(uint8_t h, uint8_t l, uint8_t ext, uint8_t shift)
{
    // 1. 拼接数据：[19:12]高位 | [11:4]中位 | [3:0]扩展位
    // 逻辑：将高位左移 12 位，中位左移 4 位，并提取扩展字节中对应的 4 位低位
    int32_t val = (int32_t)((h << 12) | (l << 4) | ((ext >> shift) & 0x0F));

    // 2. 符号扩展 (Sign Extension)
    // ICM42688 的 20 位补码符号位在第 19 位 (即 0x80000)
    // 如果该位为 1，说明是负数，需要将 32 位整型的高 12 位全部补 1
    if (val & 0x80000) {
        val |= 0xFFF00000;
    }

    return val;
}

MMC5983::MMC5983(SensorID_e id, BusDriver *bus, uint8_t *tx, uint8_t *rx, uint16_t i2c_addr)
    : SensorBase(id, bus, tx, rx)
{
    // 配置 I2C 地址
    // Datasheet Page 9: 7-bit address 0110000 (0x30)
    // HAL库需要左移1位: 0x30 << 1 = 0x60
    _config.i2c.addr = (i2c_addr << 1);
}

void MMC5983::init_regs()
{
    write_reg(MMC5983_REG_CTRL1, 0x80); // SW Reset 软件复位
    vTaskDelay(pdMS_TO_TICKS(15));

    write_reg(MMC5983_REG_CTRL3, 0x00);
    vTaskDelay(pdMS_TO_TICKS(5));

    write_reg(MMC5983_REG_CTRL0, 0x08); // SET
    vTaskDelay(pdMS_TO_TICKS(10));
}
bool MMC5983::read_mag()
{
    if (!write_reg(MMC5983_REG_CTRL0, 0x01)) {
        return false;
    }//触发单次测量

    vTaskDelay(pdMS_TO_TICKS(10));

    return read_regs(MMC5983_REG_XOUT0, 7);
}
void MMC5983::process_in_task()
{

    uint32_t x_raw = ((uint32_t)_rx_buf[0] << 10) |
                     ((uint32_t)_rx_buf[1] << 2) |
                     ((_rx_buf[6] & 0xC0) >> 6);

    uint32_t y_raw = ((uint32_t)_rx_buf[2] << 10) |
                     ((uint32_t)_rx_buf[3] << 2) |
                     ((_rx_buf[6] & 0x30) >> 4);

    uint32_t z_raw = ((uint32_t)_rx_buf[4] << 10) |
                     ((uint32_t)_rx_buf[5] << 2) |
                     ((_rx_buf[6] & 0x0C) >> 2);

    // 注意：这里必须转为带符号浮点数
    float x_gauss = ((float)x_raw - 131072.0f) / 16384.0f;
    float y_gauss = ((float)y_raw - 131072.0f) / 16384.0f;
    float z_gauss = ((float)z_raw - 131072.0f) / 16384.0f;

    // 3. 数据打包 (填充父类定义的结构体)
    Sensor_Packet_t packet;
    packet.tag = _id;
    // 获取当前时间戳 (假设 FreeRTOS tick)
    packet.timestamp = Get_System_Time_ns();

    packet.data.MMC5983.mag[0] = x_gauss;
    packet.data.MMC5983.mag[1] = y_gauss;
    packet.data.MMC5983.mag[2] = z_gauss;
    Mag_Gauss_x                = x_gauss;
    Mag_Gauss_y                = y_gauss;
    Mag_Gauss_z                = z_gauss;

    if (SensorsDataHub != NULL) {
        xQueueSend(SensorsDataHub, &packet, 0);
        DBG_MON_SUCCESS(_id, &packet);
    }
}