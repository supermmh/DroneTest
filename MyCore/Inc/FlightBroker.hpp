#pragma once

#ifdef __cplusplus
#include "uORB_Topic.hpp"
#include "DataStructConfig.hpp"

class FlightBroker {
public:
    uORB_Topic<VehicleState_t>  vehicle_state;    
    uORB_Topic<InnerSetpoint_t> inner_setpoint;   
    uORB_Topic<OuterSetpoint_t> outer_setpoint;   
};

extern FlightBroker g_FlightBroker;
#endif