#pragma once
#include "main.h"

#ifdef __cplusplus
#include "FreeRTOS.h"
#include "task.h"
#include "Mydelay.hpp"
#include <string.h>

template <typename T>
class uORB_Topic {
private:
    T _data;
    uint64_t _timestamp_ns;
    uint32_t _version;

public:
    uORB_Topic() : _timestamp_ns(0), _version(0) {
        memset(&_data, 0, sizeof(T));
    }

    void publish(const T& new_data) {
        uint64_t now = Get_System_Time_ns();
        taskENTER_CRITICAL();
        _data = new_data;
        _timestamp_ns = now;
        _version++;
        taskEXIT_CRITICAL();
    }

    bool copy_to(T& out_data, uint64_t& out_timestamp, uint32_t& current_version, uint32_t last_seen_version = 0xFFFFFFFF) {
        taskENTER_CRITICAL();
        bool is_updated = (_version != last_seen_version);
        out_data = _data;
        out_timestamp = _timestamp_ns;
        current_version = _version;
        taskEXIT_CRITICAL();
        return is_updated;
    }

    uint64_t get_timestamp() {
        uint64_t ts;
        taskENTER_CRITICAL();
        ts = _timestamp_ns;
        taskEXIT_CRITICAL();
        return ts;
    }
};
#endif