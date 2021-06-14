#ifndef _H_BASE_PLANNER__
#define _H_BASE_PLANNER__

#include "planner/path.h"
#include <unordered_set>
#include <math.h>
#include <unordered_set>
#include <vector>

class BasePlanner {

public:
  BasePlanner(size_t height, size_t width) :
    height_(height),
    width_(width) {
 };

  virtual bool plan() = 0;

  inline void createOccupancyMap(const std::unordered_set<size_t> &walls) {
    walls_ = walls;
  }

  inline Path getPath() {
    return path_;
  }

  inline void clear() {
    path_.clear();
  }

  inline void addWaypoint(Waypoint &waypoint) {
    path_.addWaypoint(waypoint);
  }

  inline size_t toIndex(size_t x, size_t y) {
    return x + y * width_;
  }

  inline void toCoordinate(size_t index, size_t &x, size_t &y) {
    x = index % width_;
    y = std::floor(index / width_);
  }

protected:
  Waypoint start_point_;
  Waypoint goal_point_;
  Path path_;
  size_t height_;
  size_t width_;
  std::unordered_set<size_t> walls_;
};

#endif 