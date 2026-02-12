#pragma once

#include"main.h"
#include"FreeRTOS.h"
#include"task.h"
#include"spi.h"
#include"i2c.h"
#include"QueueConfig.hpp"
#include"Sensors.hpp"
#include"DataStructConfig.hpp"
#include"DTCMinit.hpp"
#include"Mydelay.hpp"


#define ICM42688_Buffer_Length 32
#define MMC5983_Buffer_Length 32
#define DPS310_Buffer_Length 32
#define PMW3901_Buffer_Length 32

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


void SystemSensors_Init();
void SystemSensors_Init_Entry(void *argument);