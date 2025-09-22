#include "spline_cones.hpp"
#include "Clothoids.hh"
#include "map.hpp"
#include <vector>

struct CurvilinearPosition {
    double s; // curvilinear abscissa on the spline
    double n; // lateral distance from the spline (signed)
    double xi; // relative yaw (vehicle heading - spline heading)

    CurvilinearPosition(double s_, double n_, double xi_)
        : s(s_), n(n_), xi(xi_) {}
};

class VehicleCurvilinear {
  public:
    // Compute curvilinear position from vehicle state and spline
    static CurvilinearPosition compute(const ClothoidList* spline, double veh_x, double veh_y, double veh_heading) {
        ClosestPoint cp;
        spline->closest_point_ISO(veh_x, veh_y, cp.x, cp.y, cp.s, cp.t, cp.dst);
        double dx = veh_x - cp.x;
        double dy = veh_y - cp.y;
        double n = std::sqrt(dx * dx + dy * dy);
        // sign of n: positive if vehicle is to the left of spline direction
        double theta_s = spline->theta(cp.s);
        double cross = std::sin(theta_s) * (veh_x - cp.x) - std::cos(theta_s) * (veh_y - cp.y);
        if (cross < 0)
            n = -n;
        double xi = veh_heading - theta_s;
        return CurvilinearPosition(cp.s, n, xi);
    }
};

ClothoidList* SplineCones::buildSplineFromCones(const std::vector<Vector2>& cones) {
    std::vector<real_type> x, y;
    for (const auto& cone : cones) {
        x.push_back(cone.x);
        y.push_back(cone.y);
    }
    ClothoidList* cl = new ClothoidList();
    cl->build_G1(x.size(), &x[0], &y[0]);
    return cl;
}