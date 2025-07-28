// #include "map.hpp"
// #include "raylib.h"
// #include "utils/utils.hpp"

// void Map::draw() {
//   for (size_t i = 0; i < sampledPoints.size(); i++) {
//     DrawCircle(mToPixels(sampledPoints[i].x), -mToPixels(sampledPoints[i].y), 5, GRAY);
//   }
//   for (size_t i = 0; i < leftCones.size(); i++) {
//     DrawCircle(mToPixels(leftCones[i].x), -mToPixels(leftCones[i].y), 5, BLUE);
//   }
//   for (size_t i = 0; i < rightCones.size(); i++) {
//     DrawCircle(mToPixels(rightCones[i].x), -mToPixels(rightCones[i].y), 5, GOLD);
//   }
// }

// void Map::build(const std::vector<real_type> &x,
//                 const std::vector<real_type> &y) {
//   cl = new ClothoidList();
//   cl->build_G1(x.size(), &x[0], &y[0]);
//   samplePoints();
// }

// void Map::buildFromCones(const std::vector<Vector2> leftCones, const std::vector<Vector2> rightCones) {
//   std::vector<real_type> x_center, y_center;
//   size_t n = std::min(leftCones.size(), rightCones.size());
//   for (size_t i = 0; i < n; ++i) {
//     real_type x = 0.5 * (leftCones[i].x + rightCones[i].x);
//     real_type y = 0.5 * (leftCones[i].y + rightCones[i].y);
//     x_center.push_back(x);
//     y_center.push_back(y);
//   }
//   cl = new ClothoidList();
//   cl->build_G1(x_center.size(), &x_center[0], &y_center[0]);
//   samplePoints();
// }

// void Map::buildFromSpline(Spline &spline) {
//   std::vector<G2lib::real_type> x;
//   std::vector<G2lib::real_type> y;
//   G2lib::real_type x1, y1, x2, y2;
//   std::vector<Vector2> splinePts = spline.SampleSpline();
//   Vector2 translation = {-splinePts[0].x, -splinePts[0].y};
//   Vector2 newPoint;

//   for(auto &point : splinePts) {
//     newPoint = {point.x + translation.x, point.y + translation.y};
//     x.push_back(pixelsToMm(newPoint.x) / 1000);
//     y.push_back(-pixelsToMm(newPoint.y) / 1000);
//   }

//   /* create clothoid from spline points
//   with inital point in (0,0) and rotated to align first segment to north */
//   cl = new ClothoidList();
//   cl->build_G1(x.size(), &x[0], &y[0]);
//   cl->eval(0, x1, y1);
//   cl->eval(1, x2, y2);
//   cl->rotate(atan2(x2 - x1, y2 - y1), 0, 0);
//   samplePoints();
// }

// void Map::makeCSV() {
//   std::ofstream file("track.csv");
//   file << "x (center line),y (center line), x (right cones), y (right cones), "
//           "x (left cones), y (left cones)"
//        << std::endl;
//   for (int i = 0; i < (int)sampledPoints.size(); i++) {
//     file << sampledPoints[i].x << "," << sampledPoints[i].y << ","
//          << rightCones[i].x << "," << rightCones[i].y << "," << leftCones[i].x
//          << "," << leftCones[i].y << std::endl;
//   }
//   file.close();
// }

// void Map::clear() {
//   sampledPoints.clear();
//   leftCones.clear();
//   rightCones.clear();
// }

// void Map::updateTrack() {
//   clear();
//   samplePoints();
// }