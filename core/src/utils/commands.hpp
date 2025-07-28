#pragma once
#include "telemetry/Communication.h"
#include "telemetry/Data.h"
#include "pid/pid.h"
#include "parameters/parameters.hpp"
#include <SDL2/SDL.h>
#include <Eigen/Dense>

struct PPoint {
    double x;
    double y;
};

struct PPOutput {
    double angle;
    double v_target;
};

void handleLongitudinal(Communication& communication, TelemetryData& telemetryData, PID& pid, double& speed);

void handleGamepad(Communication& communication, TelemetryData& telemetryData, PID& pid, Wheel& frontWheel, float& currentDelta, 
    float& normalizedDelta, float& prevDelta, double& speed, float& speedRatio, float& prevSpeed, bool& paused, bool& dev_info, 
    Camera2D& camera, Vector2& cameraTarget, SDL_Event& event, bool& isActivated, bool& steeringEnabled, bool& throttleEnabled, 
    bool& brakeEnabled, bool& brakeBlocked, bool& statusChanged, int& retries, std::string& message_lat);

void handleTrajectory(Communication& communication, const TelemetryData& telemetryData, PID& pid, double& speed);

PPOutput computeSteeringAngle(const std::vector<CenterlinePoint>& centerline, double x, double y, double heading);

// Get a point on the clothoid track at index idx
Point getPoint(const ClothoidList& clothoidList, int idx);

// Compute steering and target velocity using clothoids
PPOutput computeSteeringAngleClothoids(const ClothoidList& clothoidList, double x, double y, double heading, double lookahead);