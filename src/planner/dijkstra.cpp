#include "planner/dijkstra.h"
#include <iostream>

Dijkstra::Dijkstra(const size_t height, const size_t width) : BasePlanner(height, width) {
}

bool Dijkstra::plan() {
  path_.clear(); // Clear the previous path.
  const size_t map_size = height_ * width_;
  std::vector<bool> unvisited_set(map_size, true); // Set all positions to unvisited.
  std::vector<float> distances(map_size, std::numeric_limits<float>::infinity()); // Set all distances to infinity.
  std::vector<size_t> previous(map_size, std::numeric_limits<size_t>::max());  // Set all previous index to max value.
  std::priority_queue<std::pair<float, size_t>,
                     std::vector<std::pair<float, size_t>>,
                     std::greater<std::pair<float, size_t>>> priority_queue; // Priority queue for getting the closest index.
  
  const size_t start_index = toIndex(start_point_.x, start_point_.y); // Convert x, y to starting index.
  const size_t goal_index = toIndex(goal_point_.x, goal_point_.y); // Convert x, y to goal index
  // Set initial params 
  unvisited_set.at(start_index) = false;
  distances.at(start_index) = 0;
  priority_queue.emplace(std::make_pair(0, start_index));
  bool found_plan = false;

  const std::array<std::pair<const int, const int>, 8> offsets = {  // Offsets for checking around certain x, y coordinate.
    std::make_pair(-1, 1),
    std::make_pair(1, 1),
    std::make_pair(-1, -1),
    std::make_pair(1, -1),
    std::make_pair(0, 1),
    std::make_pair(-1, 0),
    std::make_pair(1, 0),
    std::make_pair(0, -1) 
  };

  while (!priority_queue.empty()) { // Keep going until priority queue is empty, or until goal is found (by breaking out)
    const size_t index = priority_queue.top().second; // Get the closest index value
    priority_queue.pop(); // Remove the item from priority_queue
    
    size_t current_x, current_y;
    toCoordinate(index, current_x, current_y); // Convert index to x, y coordinate. 

    for (size_t offset_index = 0; offset_index < offsets.size(); ++offset_index) { // For each offset
      const int x = current_x + offsets.at(offset_index).first;
      const int y = current_y + offsets.at(offset_index).second;

      if (x < 0 || x >= static_cast<int>(width_) || y < 0 || y >= static_cast<int>(height_)) { // Check if x or y is out of bounds.
        continue;
      }

      const size_t adjacent_index = toIndex(x, y); // Convert adjacent x, y to index value.    
      const float distance = std::sqrt(std::pow(x - current_x, 2) + std::pow(y - current_y, 2));  

      if (unvisited_set.at(adjacent_index) == true &&  //Check if adjacent is unvisited, not a wall, and if distance is closer. 
          walls_.count(adjacent_index) == 0 &&
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
        } 
        
        distances.at(adjacent_index) = distances.at(index) + distance;
        priority_queue.emplace(std::make_pair(distances.at(adjacent_index), adjacent_index)); // Add new point to check from.
        previous.at(adjacent_index) = index; // Set previous index value (Point) where path came from.
      } 
    }

    unvisited_set.at(index) = false; // Index has been visited.
    
    if (index == goal_index) { // Check if index is goal.
      found_plan = true;
      break;  
    }
  }

  if (found_plan) { // If plan is found, create path.
    
    if (goal_index == start_index) { // When mouse hasn't moved, goal becomes 0 as well
      return false;
    }

    createPath(previous, start_index, goal_index); // Create the path.
    return true;
  }

  return false;
}
