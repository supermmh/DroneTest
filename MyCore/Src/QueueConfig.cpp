#include"QueueConfig.hpp"
#include"DataStructConfig.hpp"
#include"Sensors.hpp"
QueueHandle_t SensorsDataHub=NULL;
void SystemQueueInit(void)
{
    SensorsDataHub=xQueueCreate(20,sizeof(Sensor_Packet_t));
}