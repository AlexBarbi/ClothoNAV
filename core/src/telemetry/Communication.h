//
// Created by Gabriele Stulzer on 31/10/24.
//

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "Data.h"
#include "paho_mqtt_connection.hpp"
#include "topics.h"

enum class SimulationType {
  ONBOARD,
  SIMULATOR
};

class Communication {
public:
  Communication(const std::string &host, const std::string &mvehicleId, SimulationType mode,
                TelemetryData *data);
  ~Communication() = default;

  bool sendSteeringAngle(double angle);
  bool sendActivateCommand(bool activate);

  bool sendThrottle(double throttle);
  bool sendBrakes(double brake);
  bool sendCommands(double angle, double throtle);
  bool sendStatus(const Serializers::Data::ASStatus& status);
  bool setStatus(bool);
  bool setStatus(bool, bool, bool);

  bool sendVehicleCommands(double throttle, double steer);

  void setInitialState();

  void sendData(double currentDesiredSpeed, double timestamp);

  PAHOMQTTConnection *getConnection() { return &this->connection; };
  void setVehicleId(const std::string &_vehicleId) {
    this->vehicleId = _vehicleId;
  }  
  void setMode(SimulationType _mode) {
    this->mode = (SimulationType) _mode;
  }
  const std::string &getVehicleId() { return this->vehicleId; }
  SimulationType getMode() { return this->mode; }

  MQTTTopics::TopicMessage getVehicleStateTopic(SimulationType mode);
  
  
private:
  TelemetryData *data;
  PAHOMQTTConnection connection;
  std::string vehicleId;
  SimulationType mode;

  std::string parseMQTTTopic(std::string topic) const;

  // Callbacks
  static void onConnectCallback(PAHOMQTTConnection *connection, void *userData);
  static void onDisconnectCallback(PAHOMQTTConnection *connection,
                                   void *userData);
  static void onErrorCallback(PAHOMQTTConnection *connection, void *userData,
                              const mqtt::token &tok);
  static void onMessageCallback(PAHOMQTTConnection *connection, void *userData,
                                const PAHOMQTTMessage &message);
};

#endif // COMMUNICATION_H
