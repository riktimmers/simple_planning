#ifndef _H_PATH__
#define _H_PATH__

#include <vector>
#include <algorithm>


// A Waypoint consist of the coordinate x, y.
struct Waypoint {
  size_t x = 0;
  size_t y = 0;

  Waypoint(size_t _x, size_t _y) : x(_x), y(_y) {
  };

  Waypoint() {};
};

// Class that represents the path created by the planner.
class Path {
  
  std::vector<Waypoint> path_;

public:
  Path(const size_t size = 1000) { 
    path_.reserve(size); // Reserve some space in the vector.
  }

  inline void addWaypoint(const Waypoint &waypoint) {
    path_.push_back(waypoint);
  }

  inline Waypoint getWaypoint(size_t index) {
    return path_.at(index);
  }

  inline size_t getSize() { 
    return path_.size();
  }
  
  inline void clear() {
    path_.clear();
  }

  inline std::vector<Waypoint> getPath() {
    return path_;
  }

  inline void reverse() {
    std::reverse(path_.begin(), path_.end());
  }
};

#endif 