#include "DataStructConfig.hpp"

// BusDriver 实现
BusDriver::BusDriver() : lock(NULL), active_device(nullptr)
{
}

void BusDriver::init()
{
    lock = xSemaphoreCreateMutex();
}

// SensorBase 实现
SensorBase::SensorBase(SensorID_e id, BusDriver *bus, uint8_t *tx, uint8_t *rx)
    : _id(id), _bus(bus), _tx_buf(tx), _rx_buf(rx)
{
}

void SensorBase::notify_task_ISR()
{
    // 1. 针对 H7 的 AXI SRAM，失效 Cache 确保 CPU 读到最新 DMA 数据
    SCB_InvalidateDCache_by_Addr((uint32_t *)_rx_buf, 32);

    // 2. 发送任务通知唤醒处理任务
    if (_data_task_handle != nullptr) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(_data_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

bool SensorBase::read_regs(uint8_t reg, uint16_t len)
{
    _bus->active_device = this; // 👈 告知总线当前使用者
    return _bus->transfer(_config, reg, _tx_buf, _rx_buf, len, true);
}

bool SensorBase::write_regs(uint8_t reg, uint8_t *data, uint16_t len)
{
    _bus->active_device = this;
    if (len > 30) return false;

    // 借用 rx_buf 暂存数据，因为 DMA 需要稳定的地址
    memcpy(_rx_buf, data, len);
    return _bus->transfer(_config, reg, _tx_buf, _rx_buf, len, false);
}

bool SensorBase::write_reg(uint8_t reg, uint8_t val)
{
    return write_regs(reg, &val, 1);
}

SPIBus::SPIBus(SPI_HandleTypeDef *h) : hspi(h)
{
}

bool SPIBus::transfer(const DeviceConfig &cfg, uint8_t reg, uint8_t *tx, uint8_t *rx, uint16_t len, bool is_read)
{
    if (xSemaphoreTake(lock, pdMS_TO_TICKS(2)) != pdTRUE) return false;

    this->current_cfg = cfg;

if (is_read) {
        if (cfg.spi.read_sets_bit) {
            tx[0] = reg | 0x80;    // 标准模式 (ICM42688): Read = 1
        } else {
            tx[0] = reg & 0x7F;    // 反向模式 (PMW3901): Read = 0
        }
        memset(&tx[1], 0, len);
    } else {
        if (cfg.spi.read_sets_bit) {
            tx[0] = reg & 0x7F;    // 标准模式: Write = 0
        } else {
            tx[0] = reg | 0x80;    // 反向模式: Write = 1
        }
        memcpy(&tx[1], rx, len);
    }

    SCB_CleanDCache_by_Addr((uint32_t *)tx, 32);
    SCB_InvalidateDCache_by_Addr((uint32_t *)rx, 32);

    HAL_GPIO_WritePin(cfg.spi.port, cfg.spi.pin, GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive_DMA(hspi, tx, rx, len + 1) != HAL_OK) {
        HAL_GPIO_WritePin(cfg.spi.port, cfg.spi.pin, GPIO_PIN_SET);
        xSemaphoreGive(lock);
        return false;
    }
    return true;
}

void SPIBus::irq_handler()
{
    HAL_GPIO_WritePin(current_cfg.spi.port, current_cfg.spi.pin, GPIO_PIN_SET);
    if (active_device) active_device->notify_task_ISR();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(lock, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

I2CBus::I2CBus(I2C_HandleTypeDef *h) : hi2c(h)
{
}

bool I2CBus::transfer(const DeviceConfig &cfg, uint8_t reg, uint8_t *tx, uint8_t *rx, uint16_t len, bool is_read)
{
    if (xSemaphoreTake(lock, pdMS_TO_TICKS(5)) != pdTRUE) return false;

    if (is_read) {
        SCB_InvalidateDCache_by_Addr((uint32_t *)rx, 32);
        if (HAL_I2C_Mem_Read_DMA(hi2c, cfg.i2c.addr, reg, I2C_MEMADD_SIZE_8BIT, rx, len) != HAL_OK) {
            xSemaphoreGive(lock);
            return false;
        }
    } else {
        memcpy(tx, rx, len); // I2C写需要放到 tx
        SCB_CleanDCache_by_Addr((uint32_t *)tx, 32);
        if (HAL_I2C_Mem_Write_DMA(hi2c, cfg.i2c.addr, reg, I2C_MEMADD_SIZE_8BIT, tx, len) != HAL_OK) {
            xSemaphoreGive(lock);
            return false;
        }
    }
    return true;
}

void I2CBus::irq_handler()
{
    if (active_device) active_device->notify_task_ISR();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(lock, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}