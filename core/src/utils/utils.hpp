#ifndef UTILS_H
#define UTILS_H

#include <cstdint>
#include <functional>
#include <fstream>
#include <sstream>

#include "raylib.h"

///                        ///
/// Utilities for graphics ///
///                        ///

float mmToPixels(double mm);
float mToPixels(double m);
double pixelsToMm(float pixels);
double calcBottom(Rectangle r);
double calcRight(Rectangle r);
bool areColorsEqual(Color a, Color b);

///                             ///
/// Utilities for communication ///
///                             ///

uint64_t getTimestampMicroseconds();

///       ///
/// Other ///
///       ///

bool readFile(const std::string file, std::string &content);

#endif // !UTILS_H