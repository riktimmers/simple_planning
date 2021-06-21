#include "planner/a_star.h"
#include <iostream>

AStar::AStar(const size_t height, const size_t width) : BasePlanner(height, width) {
}

bool AStar::plan() {
  path_.clear();
  const size_t map_size = height_ * width_;
  std::vector<bool> unvisited_set(map_size, true);
  std::vector<float> distances(map_size, std::numeric_limits<float>::infinity());
  std::vector<size_t> previous(map_size, std::numeric_limits<size_t>::max()); 
  std::priority_queue<std::pair<float, size_t>,
                      std::vector<std::pair<float, size_t>>,
                      std::greater<std::pair<float, size_t>>> priority_queue;

  const size_t start_index = toIndex(start_point_.x, start_point_.y);
  const size_t goal_index = toIndex(goal_point_.x, goal_point_.y);

  unvisited_set.at(start_index) = false;
  distances.at(start_index) = 0;

  float distance = std::sqrt(std::pow(start_point_.x - goal_point_.x, 2) + std::pow(start_point_.y - goal_point_.y, 2));
  priority_queue.emplace(std::make_pair(distance, start_index));
  bool found_plan = false;

  const std::array<std::pair<const int, const int>, 8> offsets = { 
    std::make_pair(-1,1),
    std::make_pair(1, 1),
    std::make_pair(-1, -1),
    std::make_pair(1, -1),
    std::make_pair(0, 1),
    std::make_pair(-1, 0),
    std::make_pair(1, 0),
    std::make_pair(0, -1) 
  };

  while (!priority_queue.empty()) {
    const size_t index = priority_queue.top().second;
    priority_queue.pop();

    size_t current_x, current_y;
    toCoordinate(index, current_x, current_y);

    for (size_t offset_index = 0; offset_index < offsets.size(); ++offset_index) {
      const int x = current_x + offsets.at(offset_index).first;
      const int y = current_y + offsets.at(offset_index).second;

      if (x < 0 || x >= static_cast<int>(width_) || y < 0 || y >= static_cast<int>(height_)) {
        continue;
      }

      const size_t adjacent_index = toIndex(x, y);
      //distance = std::sqrt(std::pow(x - current_x, 2) + std::pow(y - current_y, 2));
      distance = std::fabs(x - current_x) + std::fabs(y - current_y);

      //if (unvisited_set.at(adjacent_index) == true && 
      if (walls_.count(adjacent_index) == 0 &&
          distances.at(index) + distance < distances.at(adjacent_index)) {
          
        switch(offset_index) { // Check diagonal walls
          case 0: 
            if (walls_.count(toIndex(current_x - 1, current_y)) || walls_.count(toIndex(current_x, current_y + 1))) {
              continue;
            }
          case 1:
            if (walls_.count(toIndex(current_x, current_y + 1)) || walls_.count(toIndex(current_x + 1, current_y))) {
              continue;
            }
          case 2:
            if (walls_.count(toIndex(current_x - 1, current_y)) || walls_.count(toIndex(current_x, current_y - 1))) {
              continue;
            }
          case 3:
            if (walls_.count(toIndex(current_x, current_y - 1)) || walls_.count(toIndex(current_x + 1, current_y - 1))) {
              continue;
            }
        } // check if diagonal is wall as well 

        previous.at(adjacent_index) = index;
        distances.at(adjacent_index) = distances.at(index) + distance;
        distance = distances.at(adjacent_index) + std::sqrt(std::pow(x - goal_point_.x, 2) + std::pow(y - goal_point_.y, 2)); 
        distance = distances.at(adjacent_index) + (std::fabs(goal_point_.x - x) + std::fabs(goal_point_.y - y));
        priority_queue.emplace(std::make_pair(distance, adjacent_index));
      }
    }

    //unvisited_set.at(index) = false;

    if (index == goal_index) {
      found_plan = true;
      break;
    }
  }
 
  if (found_plan) {
    std::vector<size_t> path;
    path.reserve(previous.size());
    size_t index = goal_index;

    if (goal_index == start_index) { // When mouse hasn't moved, goal becomes 0 as well
      return false;
    }
    
    while(previous.at(index)  != start_index) {
      path.push_back(index);
      index = previous.at(index);
    }

    path.push_back(start_index);
    std::reverse(path.begin(), path.end());

    for(auto &index_path: path) {
      size_t x, y;
      toCoordinate(index_path, x, y);
      Waypoint waypoint;
      waypoint.x = x;
      waypoint.y = y;
      path_.addWaypoint(waypoint);
    }

    return true;
  }

  return false;
}