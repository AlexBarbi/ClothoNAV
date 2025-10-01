//
// Created by Gabriele Stulzer on 31/10/24.
//
#include <iostream>
#include <utility>

#include "Communication.h"
#include "Data.h"
#include "connection.h"
#include "paho_mqtt_connection.hpp"
#include "topics.h"
#include "raylib/src/raylib.h"
#include "serializers/serializers.h"
#include "parameters/parameters.hpp"

Communication::Communication(const std::string &_host,
  const std::string &_vehicleId, SimulationType _mode, TelemetryData *_data)
  : vehicleId(_vehicleId), mode(_mode) {
    this->data = _data;
    
    PAHOMQTTConnectionParameters parameters =
    PAHOMQTTConnectionParameters::get_localhost_default();
    parameters.uri = _host;
    connection = PAHOMQTTConnection(parameters);
    this->data->connection = &this->connection;
    
    connection.setUserData(this);
    
    connection.setOnConnectCallback(onConnectCallback);
    connection.setOnDisconnectCallback(onDisconnectCallback);
    connection.setOnErrorCallback(onErrorCallback);
    connection.setOnMessageCallback(onMessageCallback);
    connection.connect();
  }
  
  void Communication::onConnectCallback(PAHOMQTTConnection *connection,
    void *userData) {
  std::cout << "Connected" << std::endl;
  auto *tlmData = (Communication *)userData;
  
  auto topicVehicleState = tlmData->getVehicleStateTopic(tlmData->mode);
  connection->subscribe(topicVehicleState.topic, topicVehicleState.qos);
}

void Communication::onDisconnectCallback(PAHOMQTTConnection* /*connection*/, void* /*userData*/) {
  std::cout << "Disconnected" << std::endl;
}


void Communication::onErrorCallback(PAHOMQTTConnection* /*connection*/, void* /*userData*/, const mqtt::token& tok) {
  std::cout << "Error: " << tok.get_error_message() << std::endl;
}

void Communication::onMessageCallback(PAHOMQTTConnection* /*connection*/, void* userData, const PAHOMQTTMessage& message) {
  auto* tlmData = static_cast<Communication*>(userData);
  if (message.getTopic() == tlmData->getVehicleStateTopic(tlmData->mode).topic) {
    VehicleState vehicleState;
    vehicleState.deserializeFromJsonString(message.getPayload());
    tlmData->data->vehicleState = vehicleState;
    isVehicleStateReceived = true;
  }
}

bool Communication::sendVehicleCommands(double command, double angle) {
  ThrottleCommand throttleCommand;

  throttleCommand.pedalThrottle = std::clamp(command, 0.0, 100.0) / 100.0;
  throttleCommand.pedalBrakes = -std::clamp(command, -100.0, 0.0) / 100.0;

  angle = std::clamp(angle, -100.0 * DEG2RAD, 100.0 * DEG2RAD);
  throttleCommand.steerAngleDegrees = angle * RAD2DEG;

  if (this->connection.getStatus() == PAHOMQTTConnectionStatus::CONNECTED) {
    return this->connection.send(PAHOMQTTMessage(MQTTTopics::GetTopicAsCommandsSetValues(this->vehicleId, "onboard").topic, throttleCommand.serializeAsJsonString()));
  }

  return false;
}

bool Communication::sendThrottle(double command) {
  ThrottleCommand throttleCommand;
  
  throttleCommand.pedalThrottle = std::clamp(command, 0.0, 100.0) / 100.0;
  throttleCommand.pedalBrakes = -std::clamp(command, -100.0, 0.0) / 100.0;
  
  if (this->connection.getStatus() == PAHOMQTTConnectionStatus::CONNECTED) {
    return this->connection.send(PAHOMQTTMessage(MQTTTopics::GetTopicAsCommandsSetValues(this->vehicleId, "onboard").topic, throttleCommand.serializeAsJsonString()));
  }
  return false;
}

bool Communication::sendSteeringAngle(double angle) {
  SteerCommand steerCommand;

  angle = std::clamp(angle, -100.0 * DEG2RAD, 100.0 * DEG2RAD);

  steerCommand.steerAngleDegrees = angle * RAD2DEG;

  if (this->connection.getStatus() == PAHOMQTTConnectionStatus::CONNECTED) {
    return this->connection.send(PAHOMQTTMessage(MQTTTopics::GetTopicAsCommandsSetValues(this->vehicleId, "onboard").topic, steerCommand.serializeAsJsonString()));
  }

  return false;
}

bool Communication::sendStatus(const Serializers::Data::ASStatus& status) {
  // Enable steering, throttle, and brakes
  return this->connection.send(
      PAHOMQTTMessage(
          MQTTTopics::GetTopicAsCommandsSetStatus(this->vehicleId, "onboard").topic,
          status.serializeAsJsonString()
      )
  );
}

bool Communication::sendCommands(double angle, double throttle) {
  Serializers::Data::ASCommands driveCommands;
  if (angle > 90) {
    angle = 90;
  } else if (angle < -90) {
    angle = -90;
  }
  // std::cout << "Sending commands: angle = " << angle << ", throttle = " << throttle << std::endl;
  driveCommands.steerAngleDegrees = angle;
  if (throttle > 0.0) {
    driveCommands.pedalBrakes = 0.0;
    driveCommands.pedalThrottle = abs(throttle);
  } else if (throttle < -0.0) {
    driveCommands.pedalBrakes = abs(throttle);
    driveCommands.pedalThrottle = 0.0;
  }

  if (this->connection.getStatus() == PAHOMQTTConnectionStatus::CONNECTED) {
    return this->connection.send(PAHOMQTTMessage(MQTTTopics::GetTopicAsCommandsSetValues(this->vehicleId, "onboard").topic, driveCommands.serializeAsJsonString()));
  }
  return false;
}

bool Communication::setStatus(bool steerEnabled, bool throttleEnabled, bool brakeEnabled) {
  Serializers::Data::ASStatus status;

  if (steerEnabled) {
    status.steerStatus = Serializers::Data::Status::status_enabled;
  } else {
    status.steerStatus = Serializers::Data::Status::status_disabled;
  }
  if (throttleEnabled) {
    status.throttleStatus = Serializers::Data::Status::status_enabled;
  } else {
    status.throttleStatus = Serializers::Data::Status::status_disabled;
  }
  if (brakeEnabled) {
    status.brakesStatus = Serializers::Data::Status::status_enabled;
  } else {
    status.brakesStatus = Serializers::Data::Status::status_disabled;
  }

  if (connection.getStatus() == PAHOMQTTConnectionStatus::CONNECTED) {
    return this->connection.send(PAHOMQTTMessage(MQTTTopics::GetTopicAsCommandsSetStatus(this->vehicleId, "onboard").topic, status.serializeAsJsonString()));
  }

  return false;
}

bool Communication::setStatus(bool activated) {
  Serializers::Data::ASStatus status;

  if(activated) {
    status.steerStatus = Serializers::Data::Status::status_enabled;
    status.throttleStatus = Serializers::Data::Status::status_enabled;
    status.brakesStatus = Serializers::Data::Status::status_enabled;
  } else {
    status.steerStatus = Serializers::Data::Status::status_disabled;
    status.throttleStatus = Serializers::Data::Status::status_disabled;
    status.brakesStatus = Serializers::Data::Status::status_disabled;
  }

  if (connection.getStatus() == PAHOMQTTConnectionStatus::CONNECTED) {
    return this->connection.send(PAHOMQTTMessage(MQTTTopics::GetTopicAsCommandsSetStatus(this->vehicleId, "onboard").topic, status.serializeAsJsonString()));
  }

  return true;
}

void Communication::setInitialState() {
  // Set initial vehicle state (position, velocity, heading)
  Serializers::Data::VehicleState initialState;
  initialState.x = 0.0;
  initialState.y = 0.0;
  initialState.u = 0.0;
  initialState.v = 0.0;
  initialState.heading = 0.0;

  // Send initial state to the simulator
  this->connection.send(
    PAHOMQTTMessage(
      MQTTTopics::GetTopicSimulatorInitialState(this->vehicleId).topic,
      initialState.serializeAsJsonString()
    )
  );
}

void Communication::sendData(double currentDesiredSpeed, double timestamp) {
  ///                            ///
  /// Prepare the packet to send ///
  ///                            ///

  Serializers::Data::ValuesMap longitudinalDataToSend;
  longitudinalDataToSend.timestamp.values.push_back(timestamp);
  longitudinalDataToSend.valuesMap["target_speed"].values.push_back(currentDesiredSpeed);
  longitudinalDataToSend.valuesMap["pid_error"].values.push_back(currentDesiredSpeed - this->data->vehicleState.u);

  Serializers::Data::TimeValuesPack dataToSendToTelemetry;
  dataToSendToTelemetry.valuesPack["longitudinal_controller"] = longitudinalDataToSend;

  ///               ///
  /// Send the data ///
  ///               ///

  this->connection.send(
      PAHOMQTTMessage(
          MQTTTopics::GetTopicExtraDataToLog(this->getVehicleId(), "onboard").topic, 
          dataToSendToTelemetry.serializeAsProtobufString())
      );
  ///                  ///
  /// Clear the packet ///
  ///                  ///

  longitudinalDataToSend.timestamp.values.clear();
  longitudinalDataToSend.valuesMap["target_speed"].values.clear();

  dataToSendToTelemetry.valuesPack.clear();
}


MQTTTopics::TopicMessage Communication::getVehicleStateTopic(SimulationType mode) {
  switch (mode) {
    case SimulationType::SIMULATOR:
      return MQTTTopics::GetTopicExtraTlmDataVehicleState(this->vehicleId, "onboard");
      // return MQTTTopics::GetTopicSimulatorOutputs(this->vehicleId);
    case SimulationType::ONBOARD:
    default:
      return MQTTTopics::GetTopicExtraTlmDataVehicleState(this->vehicleId, "onboard");
  }
}
