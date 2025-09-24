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

void circuit(const std::string cones_csv, const TelemetryData &telemetryData, 
              Camera2D &camera, class Communication &communication) {
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

  // Draw center_spline (discretizzata in 10000 punti, colore CYAN)
  int N = 10000;
  double len = center_spline.length();
  for (int i = 0; i < N; ++i) {
      double s = i * len / (N - 1);
      real_type x, y;
      center_spline.eval(s, x, y);
      DrawCircle(mToPixels((x - meanSampleX) * scaleFactor), -mToPixels((y - meanSampleY) * scaleFactor), 0.5, GREEN);
  }
  // N = 100;
  // // Draw all direction splines in a single loop
  // struct {
  //     const ClothoidCurve& spline;
  //     Color color;
  // } splines[] = {
  //     {spline_direction_1, ORANGE},
  //     {spline_direction_2, ORANGE},
  //     {spline_direction_3, ORANGE},
  //     {spline_direction_4, ORANGE}
  // };
  // for (const auto& s : splines) {
  //     double len = s.spline.length();
  //     for (int i = 0; i < N; ++i) {
  //         double seg_s = i * len / (N - 1);
  //         real_type x, y;
  //         s.spline.eval(seg_s, x, y);
  //         DrawCircle(mToPixels((x - meanSampleX - shiftFactorX) * scaleFactor), -mToPixels((y - meanSampleY - shiftFactorY) * scaleFactor), 0.5, s.color);
  //     }
  // }

  N = 10000;
  double len_left = spline_l.length();
  for (int i = 0; i < N; ++i) {
      double s = i * len_left / (N - 1);
      real_type x, y;
      spline_l.eval(s, x, y);
      DrawCircle(mToPixels((x - meanSampleX) * scaleFactor), -mToPixels((y - meanSampleY) * scaleFactor), 0.5, BLUE);
  }

  // Draw left cones as yellow circles
  for (const auto& cone : left_cones) {
      DrawCircle(mToPixels((cone.x() - meanSampleX) * scaleFactor), -mToPixels((cone.y() - meanSampleY) * scaleFactor), 2, BLUE);
  }

  double len_right = spline_r.length();
  for (int i = 0; i < N; ++i) {
      double s = i * len_right / (N - 1);
      real_type x, y;
      spline_r.eval(s, x, y);
      DrawCircle(mToPixels((x - meanSampleX) * scaleFactor), -mToPixels((y - meanSampleY) * scaleFactor), 0.5, YELLOW);
  }

  // Draw right cones as green circles
  for (const auto& cone : right_cones) {
      DrawCircle(mToPixels((cone.x() - meanSampleX) * scaleFactor), -mToPixels((cone.y() - meanSampleY) * scaleFactor), 2, YELLOW);
  }

  // Draw steering direction vector from car position
  double car_x = telemetryData.vehicleState.x - shiftFactorX - meanSampleX;
  double car_y = telemetryData.vehicleState.y - shiftFactorY - meanSampleY;
  double vector_length = 100.0; // lunghezza in pixel
  double angle = telemetryData.vehicleState.heading + delta * DEG2RAD; // delta deve essere in radianti
  double vec_x = car_x + std::cos(angle) * vector_length / mToPixels(1.0);
  double vec_y = car_y + std::sin(angle) * vector_length / mToPixels(1.0);
  DrawLine(mToPixels(car_x * scaleFactor), -mToPixels(car_y * scaleFactor),
           mToPixels(vec_x * scaleFactor), -mToPixels(vec_y * scaleFactor), RED);

  // Draw car heading vector (in GREEN)
  double heading_length = 100.0;
  double heading_angle = telemetryData.vehicleState.heading;
  double heading_x = car_x + std::cos(heading_angle) * heading_length / mToPixels(1.0);
  double heading_y = car_y + std::sin(heading_angle) * heading_length / mToPixels(1.0);
  DrawLine(mToPixels(car_x * scaleFactor), -mToPixels(car_y * scaleFactor),
           mToPixels(heading_x * scaleFactor), -mToPixels(heading_y * scaleFactor), GREEN);

  // Draw velocity vector (u, v) from car position
  double u = telemetryData.vehicleState.u; // Longitudinal velocity
  double v = telemetryData.vehicleState.v; // Lateral velocity
  double vel_length = 1.0; // scale for visualization
  double heading = telemetryData.vehicleState.heading;
  // Calcola la risultante nel sistema globale
  double vel_x = car_x + (u * std::cos(heading) - v * std::sin(heading)) * vel_length;
  double vel_y = car_y + (u * std::sin(heading) + v * std::cos(heading)) * vel_length;
  DrawLine(mToPixels(car_x * scaleFactor), -mToPixels(car_y * scaleFactor),
           mToPixels(vel_x * scaleFactor), -mToPixels(vel_y * scaleFactor), BLUE);


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
    center_spline;
    
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