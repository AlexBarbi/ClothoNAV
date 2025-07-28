#pragma once

#include "Clothoids.hh"
#include <complex>
#include <fstream>
#include <vector>

using namespace G2lib;

struct ClosestPoint {
  real_type x;
  real_type y;
  real_type s;
  real_type t;
  real_type dst;
};

struct Point {
  real_type x;
  real_type y;
};

class Map {
  ClothoidList *cl = nullptr;
  std::vector<Point> sampledPoints;
  std::vector<Point> leftCones;
  std::vector<Point> rightCones;

  void samplePoints() {
    for (int i = 0; i < cl->length(); i += 2.0) {
      real_type x, y;

      cl->eval(i, x, y);
      sampledPoints.push_back({x, y});

      real_type offset = 2;

      real_type x1, y1;
      cl->eval_ISO(i, offset, x1, y1);
      leftCones.push_back({x1, y1});

      real_type x2, y2;
      cl->eval_ISO(i, -offset, x2, y2);
      rightCones.push_back({x2, y2});
    }
  }

public:
  void updateTrack();
  void draw();
  void makeCSV();

  void build(const std::vector<real_type> &x, const std::vector<real_type> &y);

  void buildFromCones(const std::vector<Vector2> leftCones, const std::vector<Vector2> rightCones);

  void buildFromCSV(std::ifstream &file);

//   void buildFromSpline(Spline &spline);

  ClosestPoint getClosestPoint(real_type x, real_type y) {
    ClosestPoint cp;
    cl->closest_point_ISO(x, y, cp.x, cp.y, cp.s, cp.t, cp.dst);
    return cp;
  };

  ClosestPoint getClosestPointInRange(real_type x, real_type y, real_type s,
                                      double range) {
    double nextS = s + range;
    double prevS = s - range;

    if (nextS > this->getLength()) {
      nextS = nextS - this->getLength();
    }

    if (prevS > this->getLength()) {
      prevS = prevS - this->getLength();
    }

    if (prevS < 0) {
      prevS = this->getLength() - prevS;
    }
    ClosestPoint cp;
    int icurve;
    cl->closest_point_in_s_range_ISO(x, y, prevS, nextS, cp.x, cp.y, cp.s, cp.t,
                                     cp.dst, icurve);
    return cp;
  };

  ClothoidList *GetClothoidList() { return cl; };

  double getLength() { return cl->length(); };

  double getCurvature(double s) { return cl->theta(s); };

  void clear();

  Point getPoint(double s) {
    real_type x, y;
    cl->eval(s, x, y);
    return {x, y};
  };

  ~Map() { delete cl; };
};