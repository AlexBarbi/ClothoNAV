#include "CsvParser.hh"
#include "geographic_coordinates.hh"
#include "raylib.h"

#include <iostream>
#include <string>
#include <vector>

using namespace geographic_coordinates;

std::string_view CSVRow::operator[](std::size_t index) const {
  return std::string_view(&m_line[m_data[index] + 1],
                          m_data[index + 1] - (m_data[index] + 1));
}

std::size_t CSVRow::size() const { return m_data.size() - 1; }

void CSVRow::readNextRow(std::istream &str) {
  std::getline(str, m_line);

  m_data.clear();
  m_data.emplace_back(-1);
  std::string::size_type pos = 0;
  while ((pos = m_line.find(',', pos)) != std::string::npos) {
    m_data.emplace_back(pos);
    ++pos;
  }
  // This checks for a trailing comma with no data after it.
  pos = m_line.size();
  m_data.emplace_back(pos);
}

std::istream &operator>>(std::istream &str, CSVRow &data) {
  data.readNextRow(str);
  return str;
}

CSVIterator::CSVIterator(std::istream &str)
    : m_str(str.good() ? &str : nullptr) {
  ++(*this);
}

CSVIterator::CSVIterator() : m_str(nullptr) {}

CSVIterator &CSVIterator::operator++() {
  if (m_str) {
    if (!((*m_str) >> m_row)) {
      m_str = nullptr;
    }
  }
  return *this;
}

CSVIterator CSVIterator::operator++(int) {
  CSVIterator tmp(*this);
  ++(*this);
  return tmp;
}

CSVRow const &CSVIterator::operator*() const { return m_row; }

CSVRow const *CSVIterator::operator->() const { return &m_row; }

bool CSVIterator::operator==(CSVIterator const &rhs) {
  return ((this == &rhs) ||
          ((this->m_str == nullptr) && (rhs.m_str == nullptr)));
}

bool CSVIterator::operator!=(CSVIterator const &rhs) {
  return !((*this) == rhs);
}

GPSData parseGPSData(CSVIterator &row) {
  GPSData data;
  data.timestamp = std::stoll(std::string((*row)[0]));
  data.lon = std::stod(std::string((*row)[3]));
  data.lat = std::stod(std::string((*row)[4]));
  data.height = std::stod(std::string((*row)[5]));

  // Remove parsed row from iterator
  row++;
  return data;
}

void printNextRowConverted(CSVIterator &begin, geodetic_transformations &g) {
  GPSData data = parseGPSData(begin);
  double east, north, up;
  g.convert_geodetic_coords_to_ENU(data.lat, data.lon, data.height, east, north,
                                   up);
  std::cout << "East: " << east << std::endl;
  std::cout << "North: " << north << std::endl;
  std::cout << "Up: " << up << std::endl;
}

/**
 * Convert GPS data to ENU coordinates
 * @param begin
 * @param skipStartPoints
 * @param skipEndPoints
 * @return
 */
std::vector<GPSENUData> convertGpsToEnu(CSVIterator &begin, int skipStartPoints,
                                        int skipEndPoints) {
  cout << "Converting GPS to ENU" << endl;
  std::vector<GPSENUData> gpsData;
  double scale = 1;

  // Discard header
  begin++;

  // Get First element
  GPSData data = parseGPSData(begin);
  geodetic_transformations g =
      geodetic_transformations(data.lat, data.lon, data.height);

  for (int i = 0; i < skipStartPoints; i++)
    begin++;

  while (begin != CSVIterator()) {
    GPSData tmp = parseGPSData(begin);
    double east, north, up;
    g.convert_geodetic_coords_to_ENU(tmp.lat, tmp.lon, tmp.height, east, north,
                                     up);
    GPSENUData enu;
    enu.timestamp = tmp.timestamp;
    enu.east = east * scale;
    enu.north = north * scale;
    enu.up = up * scale;

    gpsData.push_back(enu);
  }

  gpsData.resize(gpsData.size() - skipEndPoints);
  return gpsData;
}

std::vector<Vector2> convertGpsToVector2(CSVIterator &begin) {
  cout << "Converting GPS to Vector2" << endl;
  std::vector<Vector2> gpsData;
  double scaleFactor = 1000000;

  // Discard header
  begin++;

  // Get First element
  GPSData data = parseGPSData(begin);

  while (begin != CSVIterator()) {
    GPSData tmp = parseGPSData(begin);
    Vector2 v;
    v.x = (tmp.lon - data.lon) * scaleFactor;
    v.y = (tmp.lat - data.lat) * scaleFactor;

    gpsData.push_back(v);
  }
  return gpsData;
}

void exportGpsAndEnu(CSVIterator &loop, std::vector<GPSENUData> &enuData,
                     std::vector<Vector2> &gpsData, int skipStartPoints,
                     int skipEndPoints) {
  cout << "Converting GPS to Vector2" << endl;
  double scaleFactor = 1000000;
  double enuScale = 1;

  // Discard header
  loop++;

  // Get First element
  GPSData data = parseGPSData(loop);
  geodetic_transformations g =
      geodetic_transformations(data.lat, data.lon, data.height);

  for (int i = 0; i < skipStartPoints; i++) {
    loop++;
  }

  while (loop != CSVIterator()) {
    GPSData tmp = parseGPSData(loop);
    Vector2 v;
    v.x = (tmp.lon - data.lon) * scaleFactor;
    v.y = (tmp.lat - data.lat) * scaleFactor;

    gpsData.push_back(v);

    double east, north, up;
    g.convert_geodetic_coords_to_ENU(tmp.lat, tmp.lon, tmp.height, east, north,
                                     up);
    GPSENUData enu;
    enu.timestamp = tmp.timestamp;
    enu.east = east * enuScale;
    enu.north = north * enuScale;
    enu.up = up * enuScale;

    enuData.push_back(enu);
  }

  if (skipEndPoints != 0) {
    gpsData.resize(gpsData.size() - skipEndPoints);
    enuData.resize(gpsData.size() - skipEndPoints);
  }
}