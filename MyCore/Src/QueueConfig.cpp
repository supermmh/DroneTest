#include "QueueConfig.hpp"
#include "DataStructConfig.hpp"
#include "Sensors.hpp"
QueueHandle_t SensorsDataHub = NULL;
QueueHandle_t VehicleStateQueue=NULL;

void SystemQueueInit(void)
{
    SensorsDataHub    = xQueueCreate(20, sizeof(Sensor_Packet_t));
    VehicleStateQueue = xQueueCreate(1, sizeof(VehicleState_t));
}
