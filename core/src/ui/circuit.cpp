#include <string>
#include <vector>
#include "telemetry/Communication.h"
#include "telemetry/Data.h"
#include "utils/utils.hpp"
#include "parameters/parameters.hpp"
#include "raylib.h"
#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>

void circuit(const std::string cones_csv, const std::vector<CenterlinePoint> centerline, 
            const TelemetryData &telemetryData, Camera2D &camera, class Communication &communication) {
  ClearBackground(BLACK);
  static bool followCar = false;
  static double scaleFactor = 0.5;
  static double shiftFactorX = 0.0;
  static double shiftFactorY = 0.0;
  
  if (resetTelemetry && isVehicleStateReceived) {
    shiftFactorX = telemetryData.vehicleState.x;
    shiftFactorY = telemetryData.vehicleState.y;
    resetTelemetry = false;
  }

  // Calcola la media delle coordinate x e y dei samplePoints
  double meanSampleX = 0.0;
  double meanSampleY = 0.0;
  if (!samplePoints.empty()) {
    for (const auto& pt : samplePoints) {
      meanSampleX += pt.x;
      meanSampleY += pt.y;
    }
    meanSampleX /= samplePoints.size();
    meanSampleY /= samplePoints.size();
  }

  BeginMode2D(camera);
  DrawText("Press 'N' to reset telemetry", +300, -100, 5, WHITE);
  DrawText("Press 'B' to stop the vehicle", +300, -110, 5, WHITE);
  DrawText("Press 'Enter' to toggle trajectory", +300, -120, 5, WHITE);
  DrawText("Press 'F' to toggle follow car mode", +300, -130, 5, WHITE);

  // Draw car position as a smaller red circle
  DrawCircle(mToPixels((telemetryData.vehicleState.x - shiftFactorX - meanSampleX) * scaleFactor), 
        -mToPixels((telemetryData.vehicleState.y - shiftFactorY - meanSampleY) * scaleFactor), 
        4, RED);

  // // // Draw target position as smaller white circles
  // DrawCircle(mToPixels(centerline[preview_idx].x * scaleFactor), 
  //       -mToPixels(centerline[preview_idx].y * scaleFactor), 
  //       3, WHITE);
  // DrawCircle(mToPixels(centerline[nearest_idx].x * scaleFactor), 
  //       -mToPixels(centerline[nearest_idx].y * scaleFactor), 
  //       3, PURPLE);

  // // Draw centerline points as even smaller blue circles
  // for (const auto& pt : centerline) {
  //     DrawCircle(mToPixels((pt.x) * scaleFactor), -mToPixels((pt.y) * scaleFactor), 2, GREEN);
  //     char v_final_str[32];
  //     snprintf(v_final_str, sizeof(v_final_str), "%.2f", pt.v_final);
  //     DrawText(v_final_str, mToPixels((pt.x + 10) * scaleFactor), -mToPixels((pt.y + 10) * scaleFactor), 5, WHITE);
  // }

  // Draw sampled points from ClothoidList
  for (const auto& pt : samplePoints) {
      DrawCircle(mToPixels((pt.x - meanSampleX) * scaleFactor), -mToPixels((pt.y - meanSampleY) * scaleFactor), 2, GREEN);
  }

  // Draw left cones as yellow circles
  for (const auto& cone : left_cones) {
      DrawCircle(mToPixels((cone.x() - meanSampleX) * scaleFactor), -mToPixels((cone.y() - meanSampleY) * scaleFactor), 2, BLUE);
  }

  // Draw right cones as green circles
  for (const auto& cone : right_cones) {
      DrawCircle(mToPixels((cone.x() - meanSampleX) * scaleFactor), -mToPixels((cone.y() - meanSampleY) * scaleFactor), 2, YELLOW);
  }

  EndMode2D();

  if (IsKeyPressed(KEY_ENTER)) {
    useTrajectory = !useTrajectory;
  }
  if (IsKeyPressed(KEY_N)) {
    communication.setInitialState();
    visualizepid = 0.0;
    communication.sendVehicleCommands(0.0, 0);
    resetTelemetry = true;
    preview_idx;
    nearest_idx;
    prevS;
    pointOnTrack;
    pp;
    
  }
  if (IsKeyPressed(KEY_B)) {
    useTrajectory = false;
    communication.sendVehicleCommands(0.0, 0.0);
  }

  if (IsKeyPressed(KEY_F)) {
    followCar = !followCar;
  }

  if (followCar) {
    camera.target.x = mToPixels((telemetryData.vehicleState.x - shiftFactorX) * scaleFactor);
    camera.target.y = -mToPixels((telemetryData.vehicleState.y - shiftFactorY) * scaleFactor);
  } else {
    camera.target.x = mToPixels(0.0);
    camera.target.y = -mToPixels(0.0);
  }
}