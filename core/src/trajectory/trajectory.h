#pragma once
#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
#include <string>
#include <Eigen/Dense>

// --- Parameters ---
extern const double s_resolution;     // meters
extern const double s_step;           // sampling step for trajectory
extern const double mu;               // friction coefficient (rubber on asphalt)
extern const double g_const;          // m/s²
extern const double v_max_limit;      // max allowed speed
extern const double a_max;            // max accel [m/s²]
extern const double a_brake;          // max decel [m/s²]
extern const double default_track_width; // default track width
extern const int min_cones_for_track;    // minimum cone pairs needed

// Struct to hold cone pair (right, left)
struct ConePair {
    Eigen::Vector2d right;
    Eigen::Vector2d left;
};

// Struct to hold centerline point (x, y, heading, curvature)
struct CenterlinePoint {
    double x;
    double y;
    double heading;
    double curvature;
    double v_final;
}; 

void LoadConesFromCsv(const std::string& filename,
                      std::vector<Eigen::Vector2d>& left_cones,
                      std::vector<Eigen::Vector2d>& right_cones);

std::vector<CenterlinePoint> CreateCenterlineFromCones(const std::vector<Eigen::Vector2d>& left_cones,
                                                        const std::vector<Eigen::Vector2d>& right_cones,
                                                        std::vector<CenterlinePoint>& centerline);

void CreateClothoidListFromCones(const std::vector<Eigen::Vector2d>& left_cones, const std::vector<Eigen::Vector2d>& right_cones);

// Calculate velocity profile along the centerline
void CalculateVelocityProfile(std::vector<CenterlinePoint>& centerline);

void sampledPoints();

// Calculate velocity profile from ClothoidList
void CalculateVelocityProfileFromClothoidList();

#endif // TRAJECTORY_H
