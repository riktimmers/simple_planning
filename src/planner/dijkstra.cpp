#include "planner/dijkstra.h"
#include <iostream>
Dijkstra::Dijkstra(size_t height, size_t width) : BasePlanner(height, width) {
}

void Dijkstra::setStartPoint(size_t x, size_t y) {
  start_point_.x = x;
  start_point_.y = y;
}

void Dijkstra::setGoalPoint(size_t x, size_t y) {
  goal_point_.x = x;
  goal_point_.y = y;
}

bool Dijkstra::plan() {
  path_.clear();
  const size_t map_size = height_ * width_;
  std::vector<bool> unvisited_set(map_size, true);
  std::vector<float> distances(map_size, std::numeric_limits<float>::infinity());
  std::vector<size_t> previous(map_size, std::numeric_limits<size_t>::max());
  std::priority_queue<std::pair<float, size_t>,
                     std::vector<std::pair<float, size_t>>,
                     std::greater<std::pair<float, size_t>>> priority_queue;
  
  size_t start_index = toIndex(start_point_.x, start_point_.y);
  size_t goal_index = toIndex(goal_point_.x, goal_point_.y);
  // Set initial params 
  unvisited_set.at(start_index) = false;
  distances.at(start_index) = 0;
  priority_queue.emplace(std::make_pair(0, start_index));
  bool found_plan = false;

  while (!priority_queue.empty()) { // Keep going until priority queue is empty, or until goal is found (by breaking out)
    int index = priority_queue.top().second; // Get the closest index value
    priority_queue.pop(); // Remove the item from priority_queue
    int width = static_cast<int>(width_);
    std::array<int, 8> adjecents = {index - 1, index + 1, index - width, index + width, index - width - 1, index - width + 1, index + width - 1, index + width + 1};
    
    size_t current_x, current_y;
    toCoordinate(index, current_x, current_y); 

    for (auto &adjecent_index: adjecents) {

      if (adjecent_index < 0 || adjecent_index >= static_cast<int>(map_size)) {
        continue;
      }

      size_t x, y;
      toCoordinate(adjecent_index, x, y);
      float distance = std::sqrt(std::pow(x - current_x, 2) + std::pow(y - current_y, 2));  

      if (unvisited_set.at(adjecent_index) == true && 
          walls_.count(adjecent_index) == 0 &&
          distances.at(index) + distance < distances.at(adjecent_index)) {
          
        distances.at(adjecent_index) = distances.at(index) + distance;
        priority_queue.emplace(std::make_pair(distances.at(adjecent_index), adjecent_index));
        previous.at(adjecent_index) = index;
      } 
    }

    unvisited_set.at(index) = false;
    
    if (index == static_cast<int>(goal_index)) {
      found_plan = true;
      break;  
    }
  }

  if (found_plan) {
    std::vector<size_t> path;
    path.reserve(previous.size());
    size_t index = goal_index;
    
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


/*
   1.x Mark all nodes unvisited. Create a set of all the unvisited nodes called the unvisited set.
   2.x Assign to every node a tentative distance value: set it to zero for our initial node and to infinity for all other nodes. Set the initial node as current.[15]
   3. For the current node, consider all of its unvisited neighbours and calculate their tentative distances through the current node. 
      Compare the newly calculated tentative distance to the current assigned value and assign the smaller one. 
      For example, if the current node A is marked with a distance of 6, and the edge connecting it with a neighbour B has length 2, 
      then the distance to B through A will be 6 + 2 = 8. If B was previously marked with a distance greater than 8 then change it to 8. 
      Otherwise, the current value will be kept.
   4. When we are done considering all of the unvisited neighbours of the current node, mark the current node as visited and remove it from the unvisited set. A visited node will never be checked again.
   5. If the destination node has been marked visited (when planning a route between two specific nodes) or if the smallest tentative distance among the nodes in the unvisited set is infinity (when planning a complete traversal; occurs when there is no connection between the initial node and remaining unvisited nodes), then stop. The algorithm has finished.
   6. Otherwise, select the unvisited node that is marked with the smallest tentative distance, set it as the new "current node", and go back to step 3.
  Need a map with wall positions 
  need map dimensions 
  work with (x,y) since its easier to find 
*/