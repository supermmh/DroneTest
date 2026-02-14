#include "TaskConfig.hpp"
#include "DebugMonitor.hpp"
SPIBus spi1_bus(&hspi1);
SPIBus spi2_bus(&hspi2);
SPIBus spi3_bus(&hspi3);
I2CBus i2c1_bus(&hi2c1);

AXI_SRAM uint8_t ICM42688RxBuffer[ICM42688_Buffer_Length];
AXI_SRAM uint8_t ICM42688TxBuffer[ICM42688_Buffer_Length];
AXI_SRAM uint8_t MMC5983RxBuffer[MMC5983_Buffer_Length];
AXI_SRAM uint8_t MMC5983TxBuffer[MMC5983_Buffer_Length];
AXI_SRAM uint8_t DPS310RxBuffer[DPS310_Buffer_Length];
AXI_SRAM uint8_t DPS310TxBuffer[DPS310_Buffer_Length];
AXI_SRAM uint8_t PMW3901RxBuffer[PMW3901_Buffer_Length];
AXI_SRAM uint8_t PMW3901TxBuffer[PMW3901_Buffer_Length];

DPS310 dps310_sensor(
    SENSOR_ID_DPS310,
    &spi1_bus,
    DPS310TxBuffer,
    DPS310RxBuffer,
    GPIOA,
    GPIO_PIN_4);
PMW3901 pmw3901_sensor(
    SENSOR_ID_PMW3901,
    &spi2_bus,
    PMW3901TxBuffer,
    PMW3901RxBuffer,
    GPIOB,
    GPIO_PIN_12);
ICM42688 icm42688_sensor(
    SENSOR_ID_ICM42688,
    &spi3_bus,
    ICM42688TxBuffer,
    ICM42688RxBuffer,
    GPIOD,
    GPIO_PIN_2);
MMC5983 mmc5983_sensor(
    SENSOR_ID_MMC5983,
    &i2c1_bus,
    MMC5983TxBuffer,
    MMC5983RxBuffer,
    0x30);

DTCM_DATA StackType_t ICM42688ReadTaskStack[1024];
DTCM_DATA StaticTask_t ICM42688ReadTaskTCB;
TaskHandle_t ICM42688ReadTaskHandle = NULL;

DTCM_DATA StackType_t ICM42688ProcessTaskStack[1024];
DTCM_DATA StaticTask_t ICM42688ProcessTaskTCB;
TaskHandle_t ICM42688ProcessTaskHandle = NULL;

DTCM_DATA StackType_t MMC5983ReadTaskStack[1024];
DTCM_DATA StaticTask_t MMC5983ReadTaskTCB;
TaskHandle_t MMC5983ReadTaskHandle = NULL;

DTCM_DATA StackType_t MMC5983ProcessTaskStack[1024];
DTCM_DATA StaticTask_t MMC5983ProcessTaskTCB;
TaskHandle_t MMC5983ProcessTaskHandle = NULL;

DTCM_DATA StackType_t DPS310ReadTaskStack[1024];
DTCM_DATA StaticTask_t DPS310ReadTaskTCB;
TaskHandle_t DPS310ReadTaskHandle = NULL;

DTCM_DATA StackType_t DPS310ProcessTaskStack[1024];
DTCM_DATA StaticTask_t DPS310ProcessTaskTCB;
TaskHandle_t DPS310ProcessTaskHandle = NULL;

DTCM_DATA StackType_t PMW3901ReadTaskStack[1024];
DTCM_DATA StaticTask_t PMW3901ReadTaskTCB;
TaskHandle_t PMW3901ReadTaskHandle = NULL;

DTCM_DATA StackType_t PMW3901ProcessTaskStack[1024];
DTCM_DATA StaticTask_t PMW3901ProcessTaskTCB;
TaskHandle_t PMW3901ProcessTaskHandle = NULL;

// 以上为传感器读取和数据处理任务

void Bridge_SystemSensors_Init(void)
{
    SystemSensors_Init();
}

void SystemSensors_Init()
{
    xTaskCreate(
        SystemSensors_Init_Entry,
        "SensorInit",
        1024,
        NULL,
        configMAX_PRIORITIES - 1,
        NULL);
}

void SystemSensors_Init_Entry(void *argument)
{
    // 1. 底层与总线初始化
    spi1_bus.init();
    spi2_bus.init();
    spi3_bus.init();
    i2c1_bus.init();

    // 2. 传感器寄存器初始化
    dps310_sensor.init_regs();
    pmw3901_sensor.init_regs();
    icm42688_sensor.init_regs();
    mmc5983_sensor.init_regs();

    ICM42688ReadTaskHandle = xTaskCreateStatic(
        ICM42688ReadTaskEntry, "ICM42688Read", 1024, NULL,
        configMAX_PRIORITIES - 2, // 读取最高
        ICM42688ReadTaskStack, &ICM42688ReadTaskTCB);

    ICM42688ProcessTaskHandle = xTaskCreateStatic(
        ICM42688ProcessTaskEntry, "ICM42688Proc", 1024, NULL,
        configMAX_PRIORITIES - 3, // 解析紧随其后
        ICM42688ProcessTaskStack, &ICM42688ProcessTaskTCB);

    // --- 导航：PMW3901 光流 (较高优先级) ---
    PMW3901ReadTaskHandle = xTaskCreateStatic(
        PMW3901ReadTaskEntry, "PMW3901Read", 1024, NULL,
        configMAX_PRIORITIES - 4,
        PMW3901ReadTaskStack, &PMW3901ReadTaskTCB);

    PMW3901ProcessTaskHandle = xTaskCreateStatic(
        PMW3901ProcessTaskEntry, "PMW3901Proc", 1024, NULL,
        configMAX_PRIORITIES - 5,
        PMW3901ProcessTaskStack, &PMW3901ProcessTaskTCB);

    // --- 辅助：MMC5983 磁力计 (中等优先级) ---
    MMC5983ReadTaskHandle = xTaskCreateStatic(
        MMC5983ReadTaskEntry, "MMC5983Read", 1024, NULL,
        configMAX_PRIORITIES - 6,
        MMC5983ReadTaskStack, &MMC5983ReadTaskTCB);

    MMC5983ProcessTaskHandle = xTaskCreateStatic(
        MMC5983ProcessTaskEntry, "MMC5983Proc", 1024, NULL,
        configMAX_PRIORITIES - 7,
        MMC5983ProcessTaskStack, &MMC5983ProcessTaskTCB);

    // --- 辅助：DPS310 气压计 (中等优先级) ---
    DPS310ReadTaskHandle = xTaskCreateStatic(
        DPS310ReadTaskEntry, "DPS310Read", 1024, NULL,
        configMAX_PRIORITIES - 6, // 气压和磁力计同一级别即可
        DPS310ReadTaskStack, &DPS310ReadTaskTCB);

    DPS310ProcessTaskHandle = xTaskCreateStatic(
        DPS310ProcessTaskEntry, "DPS310Proc", 1024, NULL,
        configMAX_PRIORITIES - 7,
        DPS310ProcessTaskStack, &DPS310ProcessTaskTCB);

    // ==========================================
    // 4. 绑定 DMA 中断唤醒的解析任务句柄
    // ==========================================
    dps310_sensor.set_task_handle(DPS310ProcessTaskHandle);
    pmw3901_sensor.set_task_handle(PMW3901ProcessTaskHandle);
    icm42688_sensor.set_task_handle(ICM42688ProcessTaskHandle);
    mmc5983_sensor.set_task_handle(MMC5983ProcessTaskHandle);

    SystemQueueInit();
    DBG_MON_INIT();
    // 5. 初始化完毕，释放内存
    vTaskDelete(NULL);
}

void DPS310ReadTaskEntry(void *argument)
{
    TickType_t xLastWakeTime    = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz
    while (1) {
        DBG_MON_ATTEMPT(SENSOR_ID_DPS310);

        if (!dps310_sensor.read_data()) {
            DBG_MON_ERROR(SENSOR_ID_DPS310);
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
} // DPS310 读取任务入口函数
void DPS310ProcessTaskEntry(void *argument)
{
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        dps310_sensor.process_in_task();
    }

} // DPS310 数据处理任务入口函数
void PMW3901ReadTaskEntry(void *argument)
{
    TickType_t xLastWakeTime    = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz
    while (1) {
        DBG_MON_ATTEMPT(SENSOR_ID_PMW3901);
        if (!pmw3901_sensor.read_data()) {
            DBG_MON_ERROR(SENSOR_ID_PMW3901);
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

} // PMW3901 读取任务入口函数
void PMW3901ProcessTaskEntry(void *argument)
{
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        pmw3901_sensor.process_in_task();
    }

} // PMW3901 数据处理任务入口函数
void ICM42688ReadTaskEntry(void *argument)
{
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        DBG_MON_ATTEMPT(SENSOR_ID_ICM42688);
        if (!icm42688_sensor.read_fifo()) {
            DBG_MON_ERROR(SENSOR_ID_ICM42688);
        }
    }

} // ICM42688 读取任务入口函数
void ICM42688ProcessTaskEntry(void *argument)
{

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        icm42688_sensor.process_in_task();
    }

} // ICM42688 数据处理任务入口函数
void MMC5983ReadTaskEntry(void *argument)
{
    TickType_t xLastWakeTime    = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz
    while (1) {
        DBG_MON_ATTEMPT(SENSOR_ID_MMC5983);
        if (!mmc5983_sensor.read_mag()) {
            DBG_MON_ERROR(SENSOR_ID_MMC5983);
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

} // MMC5983 读取任务入口函数
void MMC5983ProcessTaskEntry(void *argument)
{
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        mmc5983_sensor.process_in_task();
    }

} // MMC5983 数据处理任务入口函数