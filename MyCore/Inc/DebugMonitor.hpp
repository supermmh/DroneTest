#pragma once
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "DataStructConfig.hpp"

// ==========================================
// 调试模式总开关 (设为 0 时，探针被彻底抹除，编译器实现零性能损耗)
// ==========================================
#define ENABLE_DEBUG_MONITOR 1

#if ENABLE_DEBUG_MONITOR

// 任务健康信息结构体
struct TaskDebugInfo_t {
    TaskHandle_t handle;
    char taskName[configMAX_TASK_NAME_LEN];
    uint32_t stackHighWaterMark; // 栈历史最小剩余量 (单位: Word/4字节，接近 0 时说明即将栈溢出)
    float cpuUsagePercent;       // 过去 1 秒内该任务实际消耗 CPU 算力占比 (%)
    uint32_t _lastRunTime;       // 内部变量
};

// 传感器健康与状态信息结构体
struct SensorDebugInfo_t {
    uint32_t attempts;           // 尝试发起总线读取的总次数
    uint32_t successes;          // 成功读取且数据解析有效的总次数
    uint32_t errors;             // 总线传输失败或数据包无效的次数
    uint32_t readsPerSecondHz;   // 当前真实的每秒成功读取次数 (即传感器有效运行频率 Hz)
    float failureRatePercent;    // 累计读取异常失败率 (%)
    Sensor_Packet_t latestData;  // 传感器最新数值 (在调试器直接展开看解析好的数值)
    
    uint32_t _lastSuccesses;     // 内部变量
};

// 全局 LiveWatch 聚合观测结构体
struct SystemDebugMonitor_t {
    float totalCpuUsage;         // 系统当前整体业务 CPU 占用率
    uint32_t uptimeSeconds;      // 飞控运行时长 (秒)
    uint32_t taskCount;
    
    TaskDebugInfo_t tasks[15];   
    
    SensorDebugInfo_t icm42688;
    SensorDebugInfo_t pmw3901;
    SensorDebugInfo_t dps310;
    SensorDebugInfo_t mmc5983;
};

extern SystemDebugMonitor_t g_DebugMonitor; // Live Watch 观测这个变量

void DebugMonitor_Init();
void DebugMonitor_RecordAttempt(SensorID_e id);
void DebugMonitor_RecordSuccess(SensorID_e id, const Sensor_Packet_t* packet);
void DebugMonitor_RecordError(SensorID_e id);

// 给业务逻辑插装的探针宏
#define DBG_MON_INIT()                  DebugMonitor_Init()
#define DBG_MON_ATTEMPT(id)             DebugMonitor_RecordAttempt(id)
#define DBG_MON_SUCCESS(id, packet)     DebugMonitor_RecordSuccess(id, packet)
#define DBG_MON_ERROR(id)               DebugMonitor_RecordError(id)

#else // 关闭调试监控，实现绝对的零消耗

#define DBG_MON_INIT()                  ((void)0)
#define DBG_MON_ATTEMPT(id)             ((void)0)
#define DBG_MON_SUCCESS(id, packet)     ((void)0)
#define DBG_MON_ERROR(id)               ((void)0)

#endif