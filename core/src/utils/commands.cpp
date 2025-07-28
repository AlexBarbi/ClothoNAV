#include "commands.hpp"
#include "serializers/serializers.h"
#include "utils/app_utils.hpp"
#include "parameters/parameters.hpp"
#include "raymath.h"

#include <iostream>
#include <chrono>
#include <cmath>
#include <algorithm>
#include "raygui.h"

#define MAX_STEER_ANGLE 100.0f
#define MAX_JOYSTICK_VALUE 32767

void handleLongitudinal(Communication& communication, TelemetryData& telemetryData, PID& pid, double& speed) {

    communication.sendData(currentDesiredSpeed, getTimestampMicroseconds());

    if (issimulationrunning && telemetryData.vehicleState.u <= currentDesiredSpeed && currentState == ACCELERATING) {
        PIDParams params = getPIDParamsForSpeed(telemetryData.vehicleState.u);
        pid.updateParams(params, throttle_power, brake_power);
        pidOutput = pid.calculate(currentDesiredSpeed, telemetryData.vehicleState.u);
    }
    else if (issimulationrunning && telemetryData.vehicleState.u >= currentDesiredSpeed && currentState == ACCELERATING) {
        currentState = MAINTAINING;
        start_time = std::chrono::high_resolution_clock::now();
        timestamp = std::chrono::high_resolution_clock::now();
    }
    if (issimulationrunning && currentState == MAINTAINING && (std::chrono::duration_cast<std::chrono::seconds>(timestamp - start_time).count() <= timeDuration)) {
        PIDParams params = getPIDParamsForSpeed(telemetryData.vehicleState.u);
        pid.updateParams(params, throttle_power, brake_power);
        pidOutput = pid.calculate(currentDesiredSpeed, telemetryData.vehicleState.u);
        timestamp = std::chrono::high_resolution_clock::now();
    }
    else if (issimulationrunning && currentState == MAINTAINING && (std::chrono::duration_cast<std::chrono::seconds>(timestamp - start_time).count() >= timeDuration)){
        currentState = DECELERATING;
    }
    if (issimulationrunning && telemetryData.vehicleState.u >= 0.1 && currentState == DECELERATING) {
        pidOutput = -100.0;
    }
    else if (issimulationrunning && telemetryData.vehicleState.u <= 0.1 && currentState == DECELERATING) {
        pidOutput = 0.0;
        if (numberofcyclesdone < cycleNumber -1 ) {
            currentState = ACCELERATING;
            numberofcyclesdone++;
            pid.reset();
        }
        else if (numberofcyclesdone + 1 == cycleNumber) {
            pidOutput = 0.0;
            currentState = STOPPED;
            issimulationrunning = false;
            numberofcyclesdone = cycleNumber;
        }        
    }
    if (currentState == STOPPED) {
        pidOutput = 0.0;
    }
    if (emergencystop == true) {
        pidOutput = 0.0;
        emergencystop = false;
        currentState = STOPPED;
    }
    speed = pidOutput;
    double angletosend = sin(sindt * frequency) * amplitude;
    communication.sendCommands(angletosend, speed); 
}

void handleGamepad(Communication& communication, TelemetryData& telemetryData, PID& pid, Wheel& frontWheel, float& currentDelta, 
    float& normalizedDelta, float& prevDelta, double& speed, float& speedRatio, float& prevSpeed, bool& paused, bool& dev_info, 
    Camera2D& camera, Vector2& cameraTarget, SDL_Event& event, bool& isActivated, bool& steeringEnabled, bool& throttleEnabled, 
    bool& brakeEnabled, bool& brakeBlocked, bool& statusChanged, int& retries, std::string& message_lat) {
    double currentTime = GetTime();
    double previousTime = 0.0;
    float frequency = 100.0;

    while(SDL_PollEvent(&event)) {
        if (event.type == SDL_JOYAXISMOTION) {
            if (!paused) {
                int axis = static_cast<int>(event.jaxis.axis);
                if(axis == 0) {
                    currentDelta = -(float)event.jaxis.value / MAX_JOYSTICK_VALUE * MAX_STEER_ANGLE;
                } else if(axis == 4) {
                    speed = (((float)event.jaxis.value) + MAX_JOYSTICK_VALUE) / (MAX_JOYSTICK_VALUE * 2);
                    if (speed > 1.0) speed = 1.0;
                    else if (speed < 0) speed = 0.0;
                    pidisActivated = false;
                } else if (axis == 5) {
                    speed = (((float)event.jaxis.value) + MAX_JOYSTICK_VALUE) / -(MAX_JOYSTICK_VALUE * 2);
                    if (speed < -1.0) speed = -1.0;
                    else if (speed > 0) speed = 0;
                    pidisActivated = false;
                }
            }
        }
        if (event.type == SDL_JOYBUTTONDOWN) {
            int button = static_cast<int>(event.jbutton.button);
            if (button == 1) {
                throttleEnabled = true;
                brakeEnabled = true;
                statusChanged = true;
            } else if (button == 6) {
                speed = -1.0;
            } else if (button == 3) {
                steeringEnabled = !steeringEnabled;
                statusChanged = true;
            } else if (button == 0) {
                pidisActivated = !pidisActivated;
                speed = 0.0;
                communication.sendCommands(0.0, 0.0);
            }
        } else if (event.type == SDL_JOYBUTTONUP) {
            int button = static_cast<int>(event.jbutton.button);
            if (button == 1) {
                throttleEnabled = false;
                brakeEnabled = false;
                speed = 0.0;
                statusChanged = true;
            } else if (button == 7 || button == 8 || button == 9 || button == 6) {
                speed = 0.0;
            } 
        }
        if (event.type == SDL_JOYHATMOTION) {
            if (event.jhat.value == SDL_HAT_UP) {
                speedRatio += 0.1;
                if (speedRatio > 1.0) speedRatio = 1.0;
            } else if (event.jhat.value == SDL_HAT_DOWN) {
                speedRatio -= 0.1;
                if (speedRatio < 0.0) speedRatio = 0.0;
            }
        }
    }

    if (IsKeyPressed(KEY_PERIOD)) {
        dev_info = !dev_info;
    } else if (IsKeyPressed(KEY_SPACE)) {
        brakeBlocked = !brakeBlocked;
    } else if (IsKeyPressed(KEY_E)) {
        auto succ = communication.setStatus(!isActivated);
        if (succ) {
            isActivated = !isActivated;
            std::cout << "Car " << (isActivated ? "activated" : "deactivated") << std::endl;
        }
    }

    if (statusChanged && retries < 3) {
        auto succ = communication.setStatus(steeringEnabled, throttleEnabled, brakeEnabled);
        if (succ) {
            std::cout << "Status changed: "
                    << "Steering: " << (steeringEnabled ? "enabled" : "disabled")
                    << ", Throttle: " << (throttleEnabled ? "enabled" : "disabled")
                    << ", Brake: " << (brakeBlocked ? "enabled" : "disabled") 
                    << " -- Retries: " << retries << std::endl;
            statusChanged = false;
            retries = 0;
        } else {
            std::cout << "Failed to change status, retrying... - " << retries << std::endl;
            retries++;
        }
    }

    camera.target = cameraTarget;
    camera.zoom += ((float)GetMouseWheelMove() * 0.05f);

    if (IsKeyPressed(KEY_R)) {
        cameraTarget = (Vector2){0, 0};
        camera.target = cameraTarget;
        camera.zoom = 2.0f;
        camera.rotation = 0.0f;
        currentDelta = 0.0;
    }

    if (pidisActivated) {
        PIDParams params = getPIDParamsForSpeed(telemetryData.vehicleState.u);        
        pid.updateParams(params, speedRatio, brake_power);
        double pidOutput = pid.calculate(currentDesiredSpeed, telemetryData.vehicleState.u);
        speed = pidOutput;
        std::cout << "PID Output: " << speed << std::endl;
    }

    if (!paused) {
        if (currentTime - previousTime > 1.0 / frequency && 
            (currentDelta != prevDelta || abs(speed - prevSpeed) > 0.0001)) {
            previousTime = currentTime;
            frontWheel.Update(currentDelta);
            normalizedDelta = -currentDelta;

            std::cout << "Sending: " << frontWheel.GetDelta() * DEG2RAD << std::endl;
            std::cout << "Sending throttle: " << speed << std::endl;
            
            if (!message_lat.empty()) {
                Serializers::Data::VehicleState zmqVehicleState;
                zmqVehicleState.deserializeFromJsonString(message_lat);
                std::cout << "Received ZMQ message: " << message_lat << std::endl;
            } else {
                std::cerr << "Failed to receive ZMQ message or message is empty." << std::endl;
            }
            communication.sendCommands(frontWheel.GetDelta(), speed);
            communication.sendData(currentDesiredSpeed, getTimestampMicroseconds());
            prevDelta = currentDelta;
        }
    }
    // Draw
    //----------------------------------------------------------------------------------
    DrawRectangle(SCREEN_WIDTH - 300, 35, 280, 275, LIGHTGRAY);

    // BeginMode2D(camera);

    // frontWheel.Draw();

    // // Draw world static elemetns

    // DrawLine((int)0, -screenHeight * 10, (int)0, screenHeight * 10, GREEN);
    // DrawLine(-screenWidth * 10, (int)0, screenWidth * 10, (int)0, GREEN);

    // EndMode2D();

    // Draw elements attached to camera
    if (dev_info) {
    DrawText(TextFormat("Steering Angle: %.3f deg", currentDelta),
                GetScreenWidth() - 250, 50, 15, BLACK);

    const char *pausedText = paused ? "Resume" : "Pause";
    const float screenWidth = GetScreenWidth();
    const float screenHeight = GetScreenHeight();

    GuiSliderBar(Rectangle{screenWidth - 250, 70, 200, 30}, "-110", "110",
                    &normalizedDelta, -110, 110);


    if (GuiButton(Rectangle{screenWidth - 250, 100, 100, 30}, pausedText)) {
        paused = !paused;
    }

    if (GuiButton(Rectangle{screenWidth - 150, 100, 100, 30}, "Reset")) {
        currentDelta = 0;
    }

    DrawText(TextFormat("Throttle: %.3f", speed),
                GetScreenWidth() - 250, 160, 15, BLACK);

    float speed_f = static_cast<float>(speed);
    GuiSliderBar(Rectangle{screenWidth - 250, 200, 200, 30}, "-1", "1",
                    &speed_f, -1, 1);
    speed = static_cast<double>(speed_f);

    DrawText(TextFormat("Speed ratio: %.3f", speedRatio),
                GetScreenWidth() - 250, 230, 15, BLACK);

    GuiSliderBar(Rectangle{screenWidth - 250, 260, 200, 30}, "0", "1",
                    &speedRatio, 0, 1);


    const char *steeringEnabledText =
        steeringEnabled ? "Enabled" : "Disabled";
    DrawText(TextFormat("Brake: %s", brakeEnabled ? "Enabled" : "Disabled"),
                screenWidth / 2 - 50, screenHeight - 90, 15, BLUE);

    DrawText(TextFormat("Throttle: %s", throttleEnabled ? "Enabled" : "Disabled"),
                screenWidth / 2 - 50, screenHeight - 70, 15, BLUE);

    DrawText(TextFormat(steeringEnabledText), screenWidth / 2 - 50,
                screenHeight - 50, 15, BLUE);


    // DrawText(TextFormat("Pid: %.2f(Kt), %.2f(Ke)", ke, kt),
    // GetScreenWidth() - 250, 90, 15, BLACK);
    if (paused)
        DrawText(TextFormat("Simulation Paused"),
                GetScreenWidth() / 2 -
                    MeasureText("Simulation Paused", 30) / 2,
                GetScreenHeight() / 2 - 15, 30, RED);
    }
    // EndDrawing();
}

int findPreviewIndex(const std::vector<CenterlinePoint>& centerline, double x, double y, double lookahead) {
    int closest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < centerline.size(); ++i) {
        double dx = centerline[i].x - x;
        double dy = centerline[i].y - y;
        double dist = dx*dx + dy*dy;
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }
    nearest_idx = closest_idx;
    // Cerca il punto a distanza lookahead dalla posizione attuale
    double accum_dist = 0.0;
    for (size_t i = closest_idx; i < centerline.size() - 1; ++i) {
        double dx = centerline[i+1].x - centerline[i].x;
        double dy = centerline[i+1].y - centerline[i].y;
        double seg_len = std::sqrt(dx*dx + dy*dy);
        accum_dist += seg_len;
        if (accum_dist >= lookahead) {
            return i+1;
        }
    }
    return (int)centerline.size() - 1;
}

PPOutput computeSteeringAngle(const std::vector<CenterlinePoint>& centerline, double x, double y, double heading, double lookahead) {
    
    preview_idx = findPreviewIndex(centerline, x, y, lookahead);
    std::cout << "Preview index to reach: " << preview_idx << std::endl;
    PPoint preview_point{centerline[preview_idx].x, centerline[preview_idx].y};

    // Calculate the track heading at the preview point
    double track_dx = centerline[preview_idx-1].x - centerline[preview_idx].x;
    double track_dy = centerline[preview_idx-1].y - centerline[preview_idx].y;
    double track_heading = std::atan2(track_dy, track_dx);
    double angle = (track_heading * RAD2DEG - heading * RAD2DEG);
    if (angle < -90)
    {
        angle += 180;
    }
    else if (angle > 90)
    {
        angle -= 180;
    }

    // Calculate angle to the nearest point
    double nearest_dx = centerline[nearest_idx].x - x;
    double nearest_dy = centerline[nearest_idx].y - y;
    double nearest_heading = std::atan2(nearest_dy, nearest_dx);
    double angle_to_nearest = (nearest_heading * RAD2DEG - heading * RAD2DEG);

    // Calculate distance from vehicle to preview point
    double dx = preview_point.x - x;
    double dy = preview_point.y - y;
    double point_dst = std::sqrt(dx*dx + dy*dy);

    // Calculate distance from vehicle to nearest point
    double nearest_dist = std::sqrt(nearest_dx*nearest_dx + nearest_dy*nearest_dy);
    
    // Gains for distance and angle
    static const double Ke = 0.5; // TODO: tuning
    static const double Kdist = 1.5; // TODO: tuning
    static const double Kt = 1.0; // TODO: tuning
    static const double Kangle = 1.0; // TODO: tuning

    double commanded_angle = Ke * point_dst + Kt * angle;
    
    std::cout << "Target angle: " << track_heading * RAD2DEG << " "
    << "Current heading: " << heading * RAD2DEG <<  " "
    << "angle: " << commanded_angle <<  std::endl;

    double v_target = centerline[nearest_idx].v_final;
    if (v_target < 0.1) {
        v_target = centerline[nearest_idx + 1].v_final; // Prevent very low speeds
    }

    PPOutput output{commanded_angle, v_target};
    return output;
}

ClosestPoint getClosestPointInRange(real_type x, real_type y, real_type s, double range) {
    
    double nextS = s + range;
    double prevS = s - range;

    if (nextS > clothoidList.length()) {
        nextS = nextS - clothoidList.length();
    }

    if (prevS > clothoidList.length()) {
        prevS = prevS - clothoidList.length();
    }

    if (prevS < 0) {
        prevS = clothoidList.length() - prevS;
    }
    ClosestPoint cp;
    int icurve;
    clothoidList.closest_point_in_s_range_ISO(x, y, prevS, nextS, cp.x, cp.y, cp.s, cp.t,
                                        cp.dst, icurve);
    return cp;
};

Point getPoint(double s) {
    real_type x, y;
    clothoidList.eval(s, x, y);
    return {x, y};
};

double getNearestIdx(double s) {
    int idx = s / clothoidList.length() * (100 - 1);
    return idx;
}

PPOutput computeSteeringAngleClothoids(const ClothoidList& clothoidList, double x, double y, double heading, double lookahead) {
    Vector2 rearAxle = {x, y};

    double Lf = lookahead;

    double xAxisHeading = heading;

    lookAheadPoint = {static_cast<float>(rearAxle.x + Lf * cos(xAxisHeading)),
                      static_cast<float>(rearAxle.y + Lf * sin(xAxisHeading))};

    ClosestPoint point = getClosestPointInRange(
        lookAheadPoint.x, lookAheadPoint.y, prevS, Lf * 0.2);

    pp = {static_cast<float>(point.x), static_cast<float>(point.y)};
    float ndst = Vector2Distance(pp, pointOnTrack);

    if (ndst > 0.1 * lookahead)
    {
        Point p = getPoint(point.s + 0.2);
        pp = {static_cast<float>(p.x), static_cast<float>(p.y)};
    }

    pointOnTrack = pp;

    double a = Vector2Angle(
        {lookAheadPoint.y - rearAxle.y, lookAheadPoint.x - rearAxle.x},
        {pointOnTrack.y - rearAxle.y, pointOnTrack.x - rearAxle.x});

    std::cout << "Angle to point: " << a * RAD2DEG << std::endl;
    if (a < 0)
    {
        point.dst = -point.dst;
    };

    double angle = (clothoidList.theta(point.s) - (heading));

    if (angle < -PI)
    {
        angle += 2 * PI;
    }
    else if (angle > PI)
    {
        angle -= 2 * PI;
    }

    double delta = Ke * point.dst - Kt * angle;

    prevS = point.s;

    std::cout << "====================" << std::endl;
    std::cout << "delta: " << delta * RAD2DEG << std::endl;
    std::cout << "dst: " << point.dst << std::endl;
    std::cout << "angle: " << angle * RAD2DEG << " -- Track Heading: " << clothoidList.theta(point.s) * RAD2DEG
              << " -- Car heading: " << heading * RAD2DEG << std::endl;
    std::cout << "====================" << std::endl << std::endl;

    nearest_idx = getNearestIdx(point.s);
    double v_target = v_profile[nearest_idx];

    PPOutput output{delta * RAD2DEG, v_target};
    return output;
}

void handleTrajectory(Communication& communication, const TelemetryData& telemetryData, PID& pid, double& speed) {
    communication.sendData(currentDesiredSpeed, getTimestampMicroseconds());
    static double shiftFactorX = 0.0;
    static double shiftFactorY = 0.0;
    if (resetTelemetry && isVehicleStateReceived) {
        shiftFactorX = telemetryData.vehicleState.x;
        shiftFactorY = telemetryData.vehicleState.y;
        resetTelemetry = false;
    }
    double pos_x = telemetryData.vehicleState.x - shiftFactorX;
    double pos_y = telemetryData.vehicleState.y - shiftFactorY;
    
    // Make look-ahead distance velocity-dependent
    double velocity = telemetryData.vehicleState.u;
    look_ahead_distance = k_lookahead * velocity + base_lookahead;

    // PPOutput output = computeSteeringAngle(centerline, pos_x, pos_y, telemetryData.vehicleState.heading, look_ahead_distance);
    PPOutput output = computeSteeringAngleClothoids(clothoidList, pos_x, pos_y, telemetryData.vehicleState.heading, look_ahead_distance);
    currentDesiredSpeed = output.v_target;
    PIDParams params = getPIDParamsForSpeed(telemetryData.vehicleState.u);
    pid.updateParams(params, throttle_power, brake_power);
    pidOutput = pid.calculate(currentDesiredSpeed, telemetryData.vehicleState.u);

    speed = pidOutput;
    delta = output.angle;
    // std::cout << "Steering angle: " << delta << " Speed: " << speed << std::endl;
    // std::cout << "Current heading: " << telemetryData.vehicleState.heading * RAD2DEG << std::endl;

    communication.sendCommands(delta, speed);
}