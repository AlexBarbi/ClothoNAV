#pragma once

#include <iostream>
#include <iterator>
#include <string>
#include <vector>

struct GPSData {
  long long timestamp;
  double lon;
  double lat;
  double height;
};

struct GPSENUData {
  long long timestamp;
  double east;
  double north;
  double up;
};

class CSVRow {
public:
  std::string_view operator[](std::size_t index) const;

  std::size_t size() const;

  void readNextRow(std::istream &str);

private:
  std::string m_line;
  std::vector<int> m_data;
};

std::istream &operator>>(std::istream &str, CSVRow &data);

class CSVIterator {
public:
  typedef std::input_iterator_tag iterator_category;
  typedef CSVRow value_type;
  typedef std::size_t difference_type;
  typedef CSVRow *pointer;
  typedef CSVRow &reference;

  CSVIterator(std::istream &str);
  CSVIterator();

  // Pre Increment
  CSVIterator &operator++();
  // Post increment
  CSVIterator operator++(int);
  CSVRow const &operator*() const;
  CSVRow const *operator->() const;

  bool operator==(CSVIterator const &rhs);
  bool operator!=(CSVIterator const &rhs);

private:
  std::istream *m_str;
  CSVRow m_row;
};

GPSData parseGPSData(CSVIterator &row);
std::vector<GPSENUData> convertGpsToEnu(CSVIterator &begin, int = 0, int = 0);

std::vector<class Vector2> convertGpsToVector2(CSVIterator &begin);

void exportGpsAndEnu(CSVIterator &loop, std::vector<GPSENUData> &enuData,
                     std::vector<Vector2> &gpsData, int skipStartPoints = 0,
                     int skipEndPoints = 0);