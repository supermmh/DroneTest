#include "TaskConfig.hpp"
#include "DataStructConfig.hpp"
#include "Sensors.hpp"
#include "spi.h"
#include "i2c.h"
#include "usart.h"
#include "DebugMonitor.hpp"

extern SPIBus spi1_bus;
extern SPIBus spi2_bus;
extern I2CBus i2c1_bus;

extern "C" {

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1) spi1_bus.irq_handler();
    else if (hspi == &hspi2) spi2_bus.irq_handler();
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c1) i2c1_bus.irq_handler();
}
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c1) i2c1_bus.irq_handler();
}
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1) spi1_bus.irq_handler();
    else if (hspi == &hspi2) spi2_bus.irq_handler();
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c1) i2c1_bus.irq_handler();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ICM42688INT_Pin) {
        if (T0_FastLoop_Handle_Ptr != NULL) {
            BaseType_t xWoken = pdFALSE;
            vTaskNotifyGiveFromISR((TaskHandle_t)T0_FastLoop_Handle_Ptr, &xWoken);
            portYIELD_FROM_ISR(xWoken);
        }
    }
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart4) {
        if (T2_Comms_Handle_Ptr != NULL) {
            BaseType_t xWoken = pdFALSE;
            vTaskNotifyGiveFromISR((TaskHandle_t)T2_Comms_Handle_Ptr, &xWoken);
            portYIELD_FROM_ISR(xWoken);
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart4) {
        // === 案发现场取证：解剖具体的硬件级死因 ===
        uint32_t err = huart->ErrorCode;
        if (err & HAL_UART_ERROR_ORE) g_DebugMonitor.uart4.err_ore++;
        if (err & HAL_UART_ERROR_FE)  g_DebugMonitor.uart4.err_fe++;
        if (err & HAL_UART_ERROR_NE)  g_DebugMonitor.uart4.err_ne++;
        if (err & HAL_UART_ERROR_PE)  g_DebugMonitor.uart4.err_pe++;
        if (err & HAL_UART_ERROR_DMA) g_DebugMonitor.uart4.err_dma++;

        // 唤醒任务去执行自愈重启代码
        if (T2_Comms_Handle_Ptr != NULL) {
            BaseType_t xWoken = pdFALSE;
            vTaskNotifyGiveFromISR((TaskHandle_t)T2_Comms_Handle_Ptr, &xWoken);
            portYIELD_FROM_ISR(xWoken);
        }
    }
}

}