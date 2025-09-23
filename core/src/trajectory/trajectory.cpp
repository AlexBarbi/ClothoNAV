#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include <string>
#include <stdexcept>
#include <algorithm>
#include <chrono>
#include "spline.h" 
#include "trajectory.h"
#include "raylib.h"
#include "parameters/parameters.hpp"

// --- Parameters ---
const double s_resolution = 0.5;   // meters
const double s_step = 7.5;         // sampling step for trajectory
const double mu = 0.5;             // friction coefficient
const double g_const = 9.81;       // m/s²
const double v_max_limit = 10.0;   // max allowed speed
const double a_max = 2.5;          // max accel [m/s²]
const double a_brake = 4.0;        // max decel [m/s²]
const double default_track_width = 3.0; // default track width
const int min_cones_for_track = 4; // minimum cone pairs needed

void LoadConesFromCsv(const std::string& filename, std::vector<Eigen::Vector2d>& left_cones, std::vector<Eigen::Vector2d>& right_cones) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + filename);
    }

    std::string line;
    std::getline(file, line); // Skip header

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string id_str, x_str, y_str;
        std::getline(ss, id_str, ',');
        std::getline(ss, x_str, ',');
        std::getline(ss, y_str, ',');

        try {
            int id = std::stoi(id_str);
            double x = -std::stod(x_str);
            double y = std::stod(y_str);

            if (id == 0) {
                left_cones.emplace_back(x, y);
            } else if (id == 1) {
                right_cones.emplace_back(x, y);
            }
        } catch (const std::exception& e) {
            std::cerr << "Warning: Skipping invalid row - " << line << ". Error: " << e.what() << std::endl;
            continue;
        }
    }
    file.close();

    // for (auto& cone : left_cones) {
    //     cone[0] -= 21.5;
    //     cone[1] -= 1.5;
    // }
    // for (auto& cone : right_cones) {
    //     cone[0] -= 21.5;
    //     cone[1] -= 1.5;
    // }
}

std::vector<CenterlinePoint> CreateCenterlineFromCones(const std::vector<Eigen::Vector2d>& left_cones,
                                                       const std::vector<Eigen::Vector2d>& right_cones,
                                                       std::vector<CenterlinePoint>& centerline) {
    if (left_cones.size() != right_cones.size() || left_cones.empty()) {
        throw std::runtime_error("Left and right cones must be non-empty and of equal size");
    }
    std::vector<Eigen::Vector2d> center_points;
    for (size_t i = 0; i < left_cones.size(); ++i) {
        center_points.push_back((left_cones[i] + right_cones[i]) / 2.0);
    }

    // Remove duplicates
    std::vector<Eigen::Vector2d> unique_points;
    std::vector<bool> used(center_points.size(), false);
    for (size_t i = 0; i < center_points.size(); ++i) {
        if (!used[i]) {
            unique_points.push_back(center_points[i]);
            for (size_t j = i + 1; j < center_points.size(); ++j) {
                if ((center_points[i] - center_points[j]).norm() < 1e-6) {
                    used[j] = true;
                }
            }
        }
    }

    if (unique_points.size() < 4) {
        throw std::runtime_error("Not enough unique points for spline");
    }

    // Spline interpolation
    std::vector<double> u_vals(unique_points.size()), x_vals, y_vals;
    for (size_t i = 0; i < unique_points.size(); ++i) {
        u_vals[i] = static_cast<double>(i) / (unique_points.size() - 1);
        x_vals.push_back(unique_points[i][0]);
        y_vals.push_back(unique_points[i][1]);
    }

    Spline1 spline_x(u_vals, x_vals);
    Spline1 spline_y(u_vals, y_vals);

    // Generate smooth centerline
    for (double u = 0.0; u <= 1.0; u += 0.01) {
        double x = spline_x(u);
        double y = spline_y(u);
        double dx = spline_x.deriv(1, u);
        double dy = spline_y.deriv(1, u);
        double d2x = spline_x.deriv(2, u);
        double d2y = spline_y.deriv(2, u);

        double heading = std::atan2(dy, dx);
        double curvature = (dx * d2y - dy * d2x) / std::pow(dx * dx + dy * dy, 1.5);
        centerline.push_back({x, y, heading, curvature, 0});
    }

    return centerline;
}

// Create clothoid list from cones
void CreateClothoidListFromCones(const std::vector<Eigen::Vector2d>& left_cones, const std::vector<Eigen::Vector2d>& right_cones) {

    std::vector<real_type> x_center, y_center;
    for (size_t i = 0; i < left_cones.size(); ++i) {
        real_type x = 0.5 * (left_cones[i].x() + right_cones[i].x());
        real_type y = 0.5 * (left_cones[i].y() + right_cones[i].y());
        x_center.push_back(x);
        y_center.push_back(y);
        // std::cout << "Centerline point " << i << ": (" << x_center[i] << ", " << y_center[i] << ")" << std::endl;
    }

    // ClothoidList cl = new ClothoidList();
    clothoidList.build_G1(x_center.size(), &x_center[0], &y_center[0]);
}

// Calculate velocity profile
void CalculateVelocityProfile(std::vector<CenterlinePoint>& centerline) {
    double prev_speed = 0.0;
    for (auto& pt : centerline) {
        double Rss = L * pt.curvature;
        double v_max = sqrt(Rss * F_ymax / m) / 5;
        if (std::isnan(v_max)) {
            pt.v_final = prev_speed;
        } else {
            pt.v_final = v_max;
            prev_speed = pt.v_final;
        }
        // std::cout << "v_final: " << pt.v_final << std::endl;
    }
}

void sampledPoints() {
    for (int i = 0; i < clothoidList.length(); i += 2.0) {
        real_type x, y;

        clothoidList.eval(i, x, y);
        samplePoints.push_back({x, y});
    }
}

// Calculate Velocity Profile from ClothoidList
void CalculateVelocityProfileFromClothoidList() {
    double prev_speed = 0.0;
    real_type x, y;
    // Example: sample points along the clothoid and calculate velocity profile
    size_t num_samples = 100;
    for (size_t i = 0; i < num_samples; ++i) {
        real_type s = clothoidList.length() * static_cast<real_type>(i) / (num_samples - 1);
        clothoidList.eval(s, x, y);
        double curvature = clothoidList.theta(s);
        double Rss = L * curvature;
        double v_max = sqrt(Rss * F_ymax / m) ;
        if (std::isnan(v_max)) {
            v_profile.push_back(prev_speed);
        } else {
            v_profile.push_back(v_max);
            prev_speed = v_max;
        }
        // std::cout << "v_final: " << prev_speed << std::endl;
    }

    sampledPoints();
}