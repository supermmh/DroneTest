#include "DebugMonitor.hpp"
#include "Mydelay.hpp"
#include <string.h>

#if ENABLE_DEBUG_MONITOR

SystemDebugMonitor_t g_DebugMonitor = {0};
static TaskHandle_t MonitorTaskHandle = NULL;
DTCM_BSS static StackType_t MonitorTaskStack[256];
DTCM_BSS static StaticTask_t MonitorTaskTCB;

static void DebugMonitor_UpdateSensor(SensorDebugInfo_t* info) {
    uint32_t curSuccesses = info->successes;
    info->readsPerSecondHz = curSuccesses - info->_lastSuccesses;
    info->_lastSuccesses = curSuccesses;
    if (info->attempts > 0) info->failureRatePercent = ((float)info->errors / (float)info->attempts) * 100.0f;
}

static void DebugMonitor_Task(void* argument) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t lastTotalRunTime = 0;
    
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
        g_DebugMonitor.uptimeSeconds++;
        
        DebugMonitor_UpdateSensor(&g_DebugMonitor.icm42688);
        DebugMonitor_UpdateSensor(&g_DebugMonitor.pmw3901);
        DebugMonitor_UpdateSensor(&g_DebugMonitor.dps310);
        DebugMonitor_UpdateSensor(&g_DebugMonitor.mmc5983);

        uint32_t curAttUpdates = g_DebugMonitor.attitude.updateCount;
        g_DebugMonitor.attitude.updatesPerSecondHz = curAttUpdates - g_DebugMonitor.attitude._lastUpdateCount;
        g_DebugMonitor.attitude._lastUpdateCount = curAttUpdates;
        
        static TaskStatus_t taskStatusArray[15]; 
        uint32_t totalRunTime;
        UBaseType_t numTasks = uxTaskGetSystemState(taskStatusArray, 15, &totalRunTime);
        g_DebugMonitor.taskCount = numTasks;
            
        uint32_t deltaTotalTime = totalRunTime - lastTotalRunTime;
        lastTotalRunTime = totalRunTime;
        if (deltaTotalTime == 0) deltaTotalTime = 1; 
            
        float sysTotalCpu = 0.0f;
            
        for (UBaseType_t i = 0; i < numTasks && i < 15; i++) {
            TaskHandle_t handle = taskStatusArray[i].xHandle;
            int idx = -1;
            for (int j = 0; j < 15; j++) {
                if (g_DebugMonitor.tasks[j].handle == handle || g_DebugMonitor.tasks[j].handle == NULL) { idx = j; break; }
            }
            if (idx != -1) {
                g_DebugMonitor.tasks[idx].handle = handle;
                strncpy(g_DebugMonitor.tasks[idx].taskName, taskStatusArray[i].pcTaskName, configMAX_TASK_NAME_LEN - 1);
                g_DebugMonitor.tasks[idx].stackHighWaterMark = taskStatusArray[i].usStackHighWaterMark;
                uint32_t deltaTaskTime = taskStatusArray[i].ulRunTimeCounter - g_DebugMonitor.tasks[idx]._lastRunTime;
                g_DebugMonitor.tasks[idx]._lastRunTime = taskStatusArray[i].ulRunTimeCounter;
                float usage = ((float)deltaTaskTime / (float)deltaTotalTime) * 100.0f;
                g_DebugMonitor.tasks[idx].cpuUsagePercent = usage;
                if (strncmp(taskStatusArray[i].pcTaskName, "IDLE", 4) != 0) sysTotalCpu += usage;
            }
        }
        g_DebugMonitor.totalCpuUsage = sysTotalCpu;
    }
}

extern "C" void DebugMonitor_Init() {
    MonitorTaskHandle = xTaskCreateStatic(DebugMonitor_Task, "DebugMon", 256, NULL, tskIDLE_PRIORITY + 1, MonitorTaskStack, &MonitorTaskTCB);
}

extern "C" void DebugMonitor_RecordAttempt(SensorID_e id) {
    switch (id) {
        case SENSOR_ID_ICM42688: g_DebugMonitor.icm42688.attempts++; break;
        case SENSOR_ID_PMW3901:  g_DebugMonitor.pmw3901.attempts++;  break;
        case SENSOR_ID_DPS310:   g_DebugMonitor.dps310.attempts++;   break;
        case SENSOR_ID_MMC5983:  g_DebugMonitor.mmc5983.attempts++;  break;
        default: break;
    }
}

extern "C" void DebugMonitor_RecordSuccess(SensorID_e id) {
    switch (id) {
        case SENSOR_ID_ICM42688: g_DebugMonitor.icm42688.successes++; break;
        case SENSOR_ID_PMW3901:  g_DebugMonitor.pmw3901.successes++;  break;
        case SENSOR_ID_DPS310:   g_DebugMonitor.dps310.successes++;   break;
        case SENSOR_ID_MMC5983:  g_DebugMonitor.mmc5983.successes++;  break;
        default: break;
    }
}

extern "C" void DebugMonitor_RecordError(SensorID_e id) {
    switch (id) {
        case SENSOR_ID_ICM42688: g_DebugMonitor.icm42688.errors++; break;
        case SENSOR_ID_PMW3901:  g_DebugMonitor.pmw3901.errors++;  break;
        case SENSOR_ID_DPS310:   g_DebugMonitor.dps310.errors++;   break;
        case SENSOR_ID_MMC5983:  g_DebugMonitor.mmc5983.errors++;  break;
        default: break;
    }
}

void DebugMonitor_RecordAttitudeUpdate(const VehicleState_t* state) {
    g_DebugMonitor.attitude.updateCount++;
    if (state != nullptr) g_DebugMonitor.attitude.latestState = *state;
}
#endif