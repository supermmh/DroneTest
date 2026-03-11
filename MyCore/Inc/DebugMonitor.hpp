#pragma once
#include "main.h"
#include "DataStructConfig.hpp" 

#define ENABLE_DEBUG_MONITOR 1

#ifdef __cplusplus
extern "C" {
#endif

void DebugMonitor_Init(void);
void DebugMonitor_RecordAttempt(SensorID_e id);
void DebugMonitor_RecordSuccess(SensorID_e id);
void DebugMonitor_RecordError(SensorID_e id);

#ifdef __cplusplus
} 
#endif

#ifdef __cplusplus
void DebugMonitor_RecordAttitudeUpdate(const VehicleState_t *state);
#endif

#if ENABLE_DEBUG_MONITOR
#define DBG_MON_INIT()                 DebugMonitor_Init()
#define DBG_MON_ATTEMPT(id)            DebugMonitor_RecordAttempt(id)
#define DBG_MON_SUCCESS(id)            DebugMonitor_RecordSuccess(id)
#define DBG_MON_ERROR(id)              DebugMonitor_RecordError(id)
#define DBG_MON_ATTITUDE_UPDATE(state) DebugMonitor_RecordAttitudeUpdate(state)
#else 
#define DBG_MON_INIT()                 ((void)0)
#define DBG_MON_ATTEMPT(id)            ((void)0)
#define DBG_MON_SUCCESS(id)            ((void)0)
#define DBG_MON_ERROR(id)              ((void)0)
#define DBG_MON_ATTITUDE_UPDATE(state) ((void)0)
#endif

#ifdef __cplusplus
#include "FreeRTOS.h"
#include "task.h"

struct TaskDebugInfo_t {
    TaskHandle_t handle;
    char taskName[configMAX_TASK_NAME_LEN];
    uint32_t stackHighWaterMark; 
    float cpuUsagePercent;       
    uint32_t _lastRunTime;       
};

struct AttitudeDebugInfo_t {
    uint32_t updateCount;        
    uint32_t updatesPerSecondHz; 
    VehicleState_t latestState;  
    uint32_t _lastUpdateCount; 
};

struct UartDebugInfo_t {
    uint32_t rx_valid_frames; 
    uint32_t tx_frames;       
    uint32_t err_ore;         
    uint32_t err_fe;          
    uint32_t err_ne;          
    uint32_t err_pe;          
    uint32_t err_dma;         
    uint32_t rx_restarts;     
};

struct SensorDebugInfo_t {
    uint32_t attempts;          
    uint32_t successes;         
    uint32_t errors;            
    uint32_t readsPerSecondHz;  
    float failureRatePercent;   
    uint32_t _lastSuccesses; 
};

// 🌟 专供 ST-Link/J-Link 在 Live Expressions 里观察的原生数据槽位
struct SensorRawDebug_t {
    float dps_pressure;
    float pmw_dx;
    float pmw_dy;
    float pmw_squal;
    float mmc_mag_x;
    float mmc_mag_y;
    float mmc_mag_z;
};

struct SystemDebugMonitor_t {
    float totalCpuUsage;    
    uint32_t uptimeSeconds; 
    uint32_t taskCount;
    TaskDebugInfo_t tasks[15];
    SensorDebugInfo_t icm42688;
    SensorDebugInfo_t pmw3901;
    SensorDebugInfo_t dps310;
    SensorDebugInfo_t mmc5983;
    AttitudeDebugInfo_t attitude;
    UartDebugInfo_t uart4;
    
    SensorRawDebug_t raw_sensors; // 🌟 挂载至全局可见
};

extern SystemDebugMonitor_t g_DebugMonitor; 
#endif