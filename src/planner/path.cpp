#include "planner/path.h"

Path::Path(const size_t size) {
  path_.reserve(size); // Reserve some memory for the path already
}

void Path::addWaypoint(const Waypoint &waypoint) {
  path_.push_back(waypoint);
}

Waypoint Path::getWaypoint(size_t index) {
  return path_.at(index);
} 

size_t Path::getSize() {
  return path_.size();
}

std::vector<Waypoint> Path::getPath() {
  return path_;
}

void Path::clear() {
  path_.clear();
}