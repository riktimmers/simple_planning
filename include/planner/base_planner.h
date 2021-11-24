#ifndef _H_BASE_PLANNER__
#define _H_BASE_PLANNER__

#include "planner/path.h"
#include <unordered_set>
#include <math.h>

class BasePlanner {

public:
  BasePlanner(const size_t height, const size_t width);

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

  inline size_t toIndex(const size_t x, const size_t y) {
    return x + y * width_;
  }

  inline void toCoordinate(const size_t index, size_t &x, size_t &y) {
    x = index % width_;
    y = std::floor(index / width_);
  }

  inline void setStartPoint(const size_t x, const size_t y) {
    
    if (x > width_) {
      return;
    }

    start_point_.x = x;
    start_point_.y = y;
  }

  inline void setGoalPoint(const size_t x, const size_t y) {

    if (x > width_) {
      return;
    }

    goal_point_.x = x;
    goal_point_.y = y;
  }

  void createPath(const std::vector<size_t> &path, const size_t start_index, const size_t goal_index);

protected:
  Waypoint start_point_;
  Waypoint goal_point_;
  Path path_;
  const size_t height_;
  const size_t width_;
  std::unordered_set<size_t> walls_;
};

#endif 