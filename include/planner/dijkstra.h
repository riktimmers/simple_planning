#ifndef _H_DIJKSTRA__
#define _H_DIJKSTRA__

#include "planner/base_planner.h"
#include <queue>
#include <algorithm>
#include <array>

class Dijkstra : public BasePlanner {

public:
  Dijkstra(const size_t height, const size_t width);
  void setStartPoint(size_t x, size_t y);
  void setGoalPoint(size_t x, size_t y);
  bool plan();
};

#endif 