#pragma once


#include <string>
#include <vector>
#include "Utils_AABB_tree.hh"
namespace G2lib {
  using std::string;
  using std::vector;
  using std::max;
  using std::get;

  using istream_type = std::basic_istream<char>;
  using ostream_type = std::basic_ostream<char>;
  using real_type    = double;
  using integer      = int;
  using AABB_TREE    = Utils::AABBtree<real_type>;
  using AABB_SET     = Utils::AABBtree<real_type>::AABB_SET;
  using AABB_MAP     = Utils::AABBtree<real_type>::AABB_MAP;

  using CurveType = enum class CurveType : integer {
    LINE,
    POLYLINE,
    CIRCLE,
    BIARC,
    BIARC_LIST,
    CLOTHOID,
    CLOTHOID_LIST
  };

  inline
  string
  to_string( CurveType n ) {
    string res = "";
    switch ( n ) {
    case CurveType::LINE:          res = "LINE";          break;
    case CurveType::POLYLINE:      res = "POLYLINE";      break;
    case CurveType::CIRCLE:        res = "CIRCLE";        break;
    case CurveType::BIARC:         res = "BIARC";         break;
    case CurveType::BIARC_LIST:    res = "BIARC_LIST";    break;
    case CurveType::CLOTHOID:      res = "CLOTHOID";      break;
    case CurveType::CLOTHOID_LIST: res = "CLOTHOID_LIST"; break;
    }
    return res;
  };
}
