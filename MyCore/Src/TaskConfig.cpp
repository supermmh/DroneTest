#include "TaskConfig.hpp"

DTCM_DATA SPIBus spi1_bus(&hspi1);
DTCM_DATA SPIBus spi2_bus(&hspi2);
DTCM_DATA SPIBus spi3_bus(&hspi3);
DTCM_DATA I2CBus i2c1_bus(&hi2c1);

AXI_SRAM uint8_t ICM42688RxBuffer[ICM42688_Buffer_Length];
AXI_SRAM uint8_t ICM42688TxBuffer[ICM42688_Buffer_Length];
AXI_SRAM uint8_t MMC5983RxBuffer[MMC5983_Buffer_Length];
AXI_SRAM uint8_t MMC5983TxBuffer[MMC5983_Buffer_Length];
AXI_SRAM uint8_t DPS310RxBuffer[DPS310_Buffer_Length];
AXI_SRAM uint8_t DPS310TxBuffer[DPS310_Buffer_Length];
AXI_SRAM uint8_t PMW3901RxBuffer[PMW3901_Buffer_Length];
AXI_SRAM uint8_t PMW3901TxBuffer[PMW3901_Buffer_Length];

DTCM_DATA DPS310 dps310_sensor(
    SENSOR_ID_DPS310,
    &spi1_bus,
    DPS310TxBuffer,
    DPS310RxBuffer,
    GPIOA,
    GPIO_PIN_4);
DTCM_DATA PMW3901 pmw3901_sensor(
    SENSOR_ID_PMW3901,
    &spi2_bus,
    PMW3901TxBuffer,
    PMW3901RxBuffer,
    GPIOB,
    GPIO_PIN_12);
DTCM_DATA ICM42688 icm42688_sensor(
    SENSOR_ID_ICM42688,
    &spi3_bus,
    ICM42688TxBuffer,
    ICM42688RxBuffer,
    GPIOD,
    GPIO_PIN_2);
DTCM_DATA MMC5983 mmc5983_sensor(
    SENSOR_ID_MMC5983,
    &i2c1_bus,
    MMC5983TxBuffer,
    MMC5983RxBuffer,
    0x30);

DTCM_DATA StackType_t ICM42688TaskStack[1024];
DTCM_DATA StaticTask_t ICM42688TaskTCB;
TaskHandle_t ICM42688TaskHandle = NULL;

DTCM_DATA StackType_t MMC5983TaskStack[1024];
DTCM_DATA StaticTask_t MMC5983TaskTCB;
TaskHandle_t MMC5983TaskHandle = NULL;

DTCM_DATA StackType_t DPS310TaskStack[1024];
DTCM_DATA StaticTask_t DPS310TaskTCB;
TaskHandle_t DPS310TaskHandle = NULL;

DTCM_DATA StackType_t PMW3901TaskStack[1024];
DTCM_DATA StaticTask_t PMW3901TaskTCB;
TaskHandle_t PMW3901TaskHandle = NULL;

// 以上为传感器读取任务

void Bridge_SystemSensors_Init(void)
{
    SystemSensors_Init();
}

void SystemSensors_Init()
{
    xTaskCreate(
        SystemSensors_Init_Entry,
        "SensorInit",
        512,
        NULL,
        configMAX_PRIORITIES - 1,
        NULL);
}

void SystemSensors_Init_Entry(void *argument)
{
    System_DTCM_Init();
    spi1_bus.init();
    spi2_bus.init();
    spi3_bus.init();
    i2c1_bus.init();
}