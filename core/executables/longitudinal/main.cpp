#include <iostream>
#include <sys/time.h>
#include <vector>
#include <Eigen/Dense>
#include <thread>
#include <stdexcept>
#include <SDL2/SDL.h>
#include <SDL_events.h>
#include <thread>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include <string>
#include <stdexcept>  
#include <ceres/ceres.h>

#include "parameters/parameters.hpp"
#include "Clothoids.hh"
#include "connection.h"
#include "map/map.hpp"
#include "paho_mqtt_connection.hpp"
#include "raylib.h"
#include "serializers/serializers.h"
#include "telemetry/Communication.h"
#include "telemetry/Data.h"
#include "utils/Button.hpp"
#include "utils/InputField.hpp"
#include "utils/CheckBox.hpp"
#include "utils/utils.hpp"
#include "utils/app_utils.hpp"
#include "utils/commands.hpp"
#include "trajectory/trajectory.h"
#include "pid/pid.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

// Define the MQTT send rate in Hz
#define MQTT_SEND_RATE 10.0 // Hz

int main(void) {
  ///                        ///
  /// General initialization ///
  ///                        ///

  SetTraceLogLevel(LOG_ERROR);
  InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "raylib [src] example - 2d camera");
  SetWindowPosition(0, 0);
  SetTargetFPS(60);
  
  bool saveandclose = false;
  bool isActivated = false;

  while(!saveandclose && !WindowShouldClose()) {
    BeginDrawing();

    menu(vehicleId, mode, saveandclose);

    displayOnlyMenu();
    EndDrawing();
  }

  if (SDL_Init(SDL_INIT_JOYSTICK) < 0) {
    std::cerr << "Failed to initialize SDL: " << SDL_GetError() << std::endl;
    useGamepad = false;
  } 
  if (SDL_NumJoysticks() < 1) {
    std::cout << "No joysticks connected!" << std::endl;
    // SDL_Quit();
    useGamepad = false;
  }
  SDL_Joystick* joystick = SDL_JoystickOpen(0);
  if (!joystick) {
    std::cerr << "Could not open joystick: " << SDL_GetError() << std::endl;
    // SDL_Quit();
    useGamepad = false;
  }

  if (mode == SimulationType::ONBOARD) {
    vehicleIdName = "hydra";
    host_number = "control.local";
  } else if (mode == SimulationType::SIMULATOR) {
    vehicleIdName = vehicleId;
    host_number = "localhost";
  }

  // Initialize telemetry data and communication
  TelemetryData telemetryData;
  Communication communication(host_number, vehicleIdName, mode, &telemetryData);

  // Wait until the connection is established
  while (communication.getConnection()->getStatus() != PAHOMQTTConnectionStatus::CONNECTED) {
    // std::cout << "Connection not established" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // Get PID parameters based on the current speed
  PIDParams params = getPIDParamsForSpeed(0.0);
  PID pid(dt, throttle_power, brake_power, params.Kp, params.Ki, params.Kd);

  // Initialization
  //--------------------------------------------------------------------------------------
  const int screenWidth = 1200;
  const int screenHeight = 900;
  bool paused = false;
  bool dev_info = true;

  Vector2 cameraTarget = {0, 0};

  Camera2D camera;
  camera.target = cameraTarget;
  camera.offset = (Vector2){screenWidth / 2.0f - 50.0, screenHeight / 2.0f - 25.0};
  camera.rotation = 0.0f;
  camera.zoom = 2.0f;

  SetTargetFPS(60); // Set our game to run at 60 frames-per-second
  //--------------------------------------------------------------------------------------

  // Wheel & Steering
  Wheel frontWheel;
  float currentDelta = 0.0;
  float normalizedDelta = 0.0;
  float prevDelta = 0.0;

  // Speed & PID
  double speed = 0.0;
  float speedRatio = 1.0;
  float prevSpeed = 0.0;

  // Actuator Flags
  bool steeringEnabled = false;
  bool throttleEnabled = false;
  bool brakeEnabled = false;
  bool brakeBlocked = false;

  // Status & Retry
  bool statusChanged = false;
  int retries = 0;

  // SDL Event
  SDL_Event event;

  // --- Main Application Loop ---
  while (!WindowShouldClose()) {
      BeginDrawing();

      // --- Page Handling ---
      switch (currentPage) {
          case MENU:
              menu(vehicleId, mode, saveandclose);
              break;
          case LONGITUDINAL:
              longitudinal(communication, telemetryData, vehicleIdName, mode, currentState, pid);
              break;
          case CIRCUIT:
              circuit(cones_csv, centerline, telemetryData, camera, communication);
              break;
      }

      // --- Control Handlers ---
      sinusoidalSpeed();

      if (useLongitudinal) {
          handleLongitudinal(communication, telemetryData, pid, speed);
      }
      if (useGamepad) {
          handleGamepad(communication, telemetryData, pid, frontWheel, currentDelta, normalizedDelta, prevDelta, speed, 
              speedRatio, prevSpeed, paused, dev_info, camera, cameraTarget, event, isActivated, steeringEnabled, throttleEnabled, 
              brakeEnabled, brakeBlocked, statusChanged, retries, message_lat);
      }
      if (useTrajectory) {
          if (x_pos != telemetryData.vehicleState.x || y_pos != telemetryData.vehicleState.y) {
              handleTrajectory(communication, telemetryData, pid, speed);
          }
      }

      // --- Visualization & UI ---
      visualizepid = speed;
      displayPages();
      EndDrawing();
  }

  CloseWindow();

  return 0;
}