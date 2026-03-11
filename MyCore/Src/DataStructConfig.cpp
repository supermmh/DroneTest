#include "DataStructConfig.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

void BusDriver::init()
{
    lock         = xSemaphoreCreateMutex();
    dma_cplt_sem = xSemaphoreCreateBinary();
}

bool BusDriver::wait_dma(uint32_t timeout_ms)
{
    return xSemaphoreTake(dma_cplt_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

SPIBus::SPIBus(SPI_HandleTypeDef *h) : hspi(h)
{
}

bool SPIBus::transfer_blocking(const DeviceConfig &cfg, uint8_t reg, uint8_t *tx, uint8_t *rx, uint16_t len, bool is_read)//阻塞式读取
{
    if (is_read) {
        tx[0] = cfg.spi.read_sets_bit ? (reg | 0x80) : (reg & 0x7F);
        memset(&tx[1], 0, len);
    } else {
        tx[0] = cfg.spi.read_sets_bit ? (reg & 0x7F) : (reg | 0x80);
        memcpy(&tx[1], rx, len);
    }
    HAL_GPIO_WritePin(cfg.spi.port, cfg.spi.pin, GPIO_PIN_RESET);
    bool ok = (HAL_SPI_TransmitReceive(hspi, tx, rx, len + 1, 10) == HAL_OK);
    HAL_GPIO_WritePin(cfg.spi.port, cfg.spi.pin, GPIO_PIN_SET);
    return ok;
}

bool SPIBus::trigger_dma(const DeviceConfig &cfg, uint8_t reg, uint8_t *tx, uint8_t *rx, uint16_t len, bool is_read)//DMA中断式读取
{
    current_cfg = cfg;
    if (is_read) {
        tx[0] = cfg.spi.read_sets_bit ? (reg | 0x80) : (reg & 0x7F);
        memset(&tx[1], 0, len);
    } else {
        tx[0] = cfg.spi.read_sets_bit ? (reg & 0x7F) : (reg | 0x80);
        memcpy(&tx[1], rx, len);
    }
    SCB_CleanDCache_by_Addr((uint32_t *)tx, 32);
    SCB_InvalidateDCache_by_Addr((uint32_t *)rx, 32);

    HAL_GPIO_WritePin(cfg.spi.port, cfg.spi.pin, GPIO_PIN_RESET);
    xSemaphoreTake(dma_cplt_sem, 0);

    if (HAL_SPI_TransmitReceive_DMA(hspi, tx, rx, len + 1) != HAL_OK) {
        HAL_GPIO_WritePin(cfg.spi.port, cfg.spi.pin, GPIO_PIN_SET);
        return false;
    }
    return true;
}

void SPIBus::irq_handler()
{
    HAL_GPIO_WritePin(current_cfg.spi.port, current_cfg.spi.pin, GPIO_PIN_SET);
    BaseType_t xWoken = pdFALSE;
    xSemaphoreGiveFromISR(dma_cplt_sem, &xWoken);
    portYIELD_FROM_ISR(xWoken);
}

I2CBus::I2CBus(I2C_HandleTypeDef *h) : hi2c(h)
{
}

bool I2CBus::transfer_blocking(const DeviceConfig &cfg, uint8_t reg, uint8_t *tx, uint8_t *rx, uint16_t len, bool is_read)
{
    if (is_read)
        return HAL_I2C_Mem_Read(hi2c, cfg.i2c.addr, reg, I2C_MEMADD_SIZE_8BIT, rx, len, 10) == HAL_OK;
    else {
        memcpy(tx, rx, len);
        return HAL_I2C_Mem_Write(hi2c, cfg.i2c.addr, reg, I2C_MEMADD_SIZE_8BIT, tx, len, 10) == HAL_OK;
    }
}

bool I2CBus::trigger_dma(const DeviceConfig &cfg, uint8_t reg, uint8_t *tx, uint8_t *rx, uint16_t len, bool is_read)
{
    xSemaphoreTake(dma_cplt_sem, 0);
    if (is_read) {
        SCB_InvalidateDCache_by_Addr((uint32_t *)rx, 32);
        return HAL_I2C_Mem_Read_DMA(hi2c, cfg.i2c.addr, reg, I2C_MEMADD_SIZE_8BIT, rx, len) == HAL_OK;
    } else {
        memcpy(tx, rx, len);
        SCB_CleanDCache_by_Addr((uint32_t *)tx, 32);
        return HAL_I2C_Mem_Write_DMA(hi2c, cfg.i2c.addr, reg, I2C_MEMADD_SIZE_8BIT, tx, len) == HAL_OK;
    }
}

void I2CBus::irq_handler()
{
    BaseType_t xWoken = pdFALSE;
    xSemaphoreGiveFromISR(dma_cplt_sem, &xWoken);
    portYIELD_FROM_ISR(xWoken);
}

SensorBase::SensorBase(SensorID_e id, BusDriver *bus, uint8_t *tx, uint8_t *rx)
    : _id(id), _bus(bus), _tx_buf(tx), _rx_buf(rx)
{
}

bool SensorBase::read_regs_blocking(uint8_t reg, uint16_t len)
{
    if (xSemaphoreTake(_bus->lock, portMAX_DELAY) != pdTRUE) return false;
    bool res = _bus->transfer_blocking(_config, reg, _tx_buf, _rx_buf, len, true);
    xSemaphoreGive(_bus->lock);
    return res;
}

bool SensorBase::write_regs_blocking(uint8_t reg, uint8_t *data, uint16_t len)
{
    if (len > 30) return false;
    if (xSemaphoreTake(_bus->lock, portMAX_DELAY) != pdTRUE) return false;
    memcpy(_rx_buf, data, len);
    bool res = _bus->transfer_blocking(_config, reg, _tx_buf, _rx_buf, len, false);
    xSemaphoreGive(_bus->lock);
    return res;
}

bool SensorBase::write_reg_blocking(uint8_t reg, uint8_t val)
{
    return write_regs_blocking(reg, &val, 1);
}

bool SensorBase::trigger_read_dma(uint8_t reg, uint16_t len)
{
    if (xSemaphoreTake(_bus->lock, pdMS_TO_TICKS(2)) != pdTRUE) return false;
    if (!_bus->trigger_dma(_config, reg, _tx_buf, _rx_buf, len, true)) {
        xSemaphoreGive(_bus->lock);
        return false;
    }
    return true;
}

bool SensorBase::wait_and_release(uint32_t timeout_ms)
{
    bool ok = _bus->wait_dma(timeout_ms);
    xSemaphoreGive(_bus->lock);
    return ok;
}