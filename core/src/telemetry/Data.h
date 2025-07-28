//
// Created by Gabriele Stulzer on 31/10/24.
//

#ifndef DATA_H
#define DATA_H

#include "data/as_commands.h"
#include "data/vehicle_state.h"
#include "telemetry/gps_maps.h"
// #include "mqtt_connection.h"
#include "paho_mqtt_connection.hpp"
#include "topics.h"

#define gpsMapOriginsTopic "/onboard/extra_tlm_data/gpsMapOrigins"
#define gpsMapOriginsTopic "/onboard/extra_tlm_data/gpsMapOrigins"
#define vehicleStateTopic "/onboard/extra_tlm_data/vehicleState"
#define steerTopic "/onboard/command/steer"
#define steerStatusTopic "/onboard/command/steerStatus"

#define simOutputsTopic "/simulator/outputs"
#define simInputsTopic "/simulator/inputs"

typedef Serializers::Data::VehicleState VehicleState;
typedef Serializers::Telemetry::GPSMapOrigins GPSMapOrigins;
typedef Serializers::Telemetry::Baseline Baseline;
typedef Serializers::Data::ASCommands SteerCommand;
typedef Serializers::Data::ASCommands SteerStatus;

typedef Serializers::Data::ASCommands ThrottleCommand;
typedef Serializers::Data::ASCommands ThrottleStatus;

typedef Serializers::Data::ASCommands BrakesCommand;
typedef Serializers::Data::ASCommands BrakesStatus;

struct TelemetryData {
  VehicleState vehicleState;
  GPSMapOrigins gpsMapOrigins;
  SteerStatus status;
  ThrottleStatus throttle;
  BrakesStatus brake;
  PAHOMQTTConnection *connection;
  // MQTTConnection *connection;
};

#endif //DATA_H
