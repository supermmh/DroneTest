#pragma once
#include "main.h" 
#ifdef __cplusplus
extern "C" {
#endif

void Bridge_SystemSensors_Init(void); // 

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "FreeRTOS.h"
#include "task.h"
#include "spi.h"
#include "i2c.h"
#include "QueueConfig.hpp"
#include "Sensors.hpp"
#include "DataStructConfig.hpp"
#include "DTCMinit.hpp"
#include "Mydelay.hpp"
#define ICM42688_Buffer_Length 32
#define MMC5983_Buffer_Length  32
#define DPS310_Buffer_Length   32
#define PMW3901_Buffer_Length  32

extern DTCM_DATA SPIBus spi1_bus;
extern DTCM_DATA SPIBus spi2_bus;
extern DTCM_DATA SPIBus spi3_bus;
extern DTCM_DATA I2CBus i2c1_bus;

extern AXI_SRAM uint8_t ICM42688RxBuffer[ICM42688_Buffer_Length];
extern AXI_SRAM uint8_t ICM42688TxBuffer[ICM42688_Buffer_Length];
extern AXI_SRAM uint8_t MMC5983RxBuffer[MMC5983_Buffer_Length];
extern AXI_SRAM uint8_t MMC5983TxBuffer[MMC5983_Buffer_Length];
extern AXI_SRAM uint8_t DPS310RxBuffer[DPS310_Buffer_Length];
extern AXI_SRAM uint8_t DPS310TxBuffer[DPS310_Buffer_Length];
extern AXI_SRAM uint8_t PMW3901RxBuffer[PMW3901_Buffer_Length];
extern AXI_SRAM uint8_t PMW3901TxBuffer[PMW3901_Buffer_Length];

extern DTCM_DATA ICM42688 icm42688_sensor;
extern DTCM_DATA MMC5983 mmc5983_sensor;
extern DTCM_DATA DPS310 dps310_sensor;
extern DTCM_DATA PMW3901 pmw3901_sensor;

extern TaskHandle_t ICM42688ReadTaskHandle;
extern TaskHandle_t ICM42688ProcessTaskHandle;
extern TaskHandle_t MMC5983ReadTaskHandle;
extern TaskHandle_t MMC5983ProcessTaskHandle;
extern TaskHandle_t DPS310ReadTaskHandle;
extern TaskHandle_t DPS310ProcessTaskHandle;
extern TaskHandle_t PMW3901ReadTaskHandle;
extern TaskHandle_t PMW3901ProcessTaskHandle;



void SystemSensors_Init();                     // 初始化函数，负责创建初始化任务
void SystemSensors_Init_Entry(void *argument); // 初始化任务，负责对总线和传感器进行初始化并创建读取和数据处理任务,以及姿态解算和控制任务
void DPS310ReadTaskEntry(void *argument);         // DPS310 读取任务入口函数
void DPS310ProcessTaskEntry(void *argument);      // DPS310 数据处理任务入口函数
void PMW3901ReadTaskEntry(void *argument);       // PMW3901 读取任务入口函数
void PMW3901ProcessTaskEntry(void *argument);    // PMW3901 数据处理任务入口函数
void ICM42688ReadTaskEntry(void *argument);      // ICM42688 读取任务入口函数
void ICM42688ProcessTaskEntry(void *argument);   // ICM42688 数据处理任务入口函数
void MMC5983ReadTaskEntry(void *argument);      // MMC5983 读取任务入口函数
void MMC5983ProcessTaskEntry(void *argument);   // MMC5983 数据处理任务入口函数

#endif 