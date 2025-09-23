#pragma once
#include "Clothoids.hh"
#include "raylib.h"
#include <Eigen/Dense>
#include <vector>

using namespace G2lib;

void buildSplineFromCones_l(const std::vector<Eigen::Vector2d>& cones);
void buildSplineFromCones_r(const std::vector<Eigen::Vector2d>& cones);