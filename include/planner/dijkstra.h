#ifndef _H_DIJKSTRA__
#define _H_DIJKSTRA__

#include "planner/base_planner.h"
#include <queue>
#include <algorithm>
#include <array>

class Dijkstra : public BasePlanner {

public:
  Dijkstra(const size_t height, const size_t width);
  bool plan();
};

#endif 