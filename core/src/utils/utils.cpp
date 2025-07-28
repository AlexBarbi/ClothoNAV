#include "utils.hpp"

#include <sys/time.h>

///                    ///
/// Utils for graphics ///
///                    ///

float mmToPixels(double mm) { return static_cast<float>(mm / 100.0); };

float mToPixels(double m) { return static_cast<float>(m * 10.0); };

double pixelsToMm(float pixels) { return pixels * 100.0; };

double calcBottom(Rectangle r) { return r.y+r.height; }

double calcRight(Rectangle r) { return r.x+r.width; }

bool areColorsEqual(Color a, Color b) {
  return a.r == b.r && a.g == b.g && a.b == b.b && a.a == b.a;
}

///                         ///
/// Utils for communication ///
///                         ///

uint64_t getTimestampMicroseconds() {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return tv.tv_sec * 1000000 + tv.tv_usec;
}

///       ///
/// Other ///
///       ///

bool readFile(const std::string file, std::string &content) {
  std::ifstream in(file, std::ifstream::in | std::ifstream::binary);
  if (!in.is_open()) {
    return false;
  }

  std::stringstream sstream;
  sstream << in.rdbuf();
  content = sstream.str();

  return true;
}