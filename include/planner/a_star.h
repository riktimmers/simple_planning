#ifndef _H_A_STAR__
#define _H_A_STAR__

#include "planner/base_planner.h"
#include <queue>
#include <algorithm>

class AStar : public BasePlanner {

public:
  AStar(const size_t height, const size_t width);
  bool plan();
};

#endif 