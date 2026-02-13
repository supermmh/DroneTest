#include "TaskConfig.hpp"
#include "main.h"
extern "C" {

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1) {
        spi1_bus.irq_handler();
    } else if (hspi == &hspi2) {
        spi2_bus.irq_handler();
    } else if (hspi == &hspi3) {
        spi3_bus.irq_handler();
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c1) {
        i2c1_bus.irq_handler();
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_8) {
        BaseType_t xWoken = pdFALSE;
        vTaskNotifyGiveFromISR(ICM42688TaskHandle, &xWoken);
        portYIELD_FROM_ISR(xWoken);
    }
}

} // extern "C" 结束
