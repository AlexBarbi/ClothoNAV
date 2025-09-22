#pragma once
#include "Clothoids.hh"
#include <vector>

using namespace G2lib;

class SplineCones {
public:
    static ClothoidList* buildSplineFromCones(const std::vector<Vector2>& cones);
};
