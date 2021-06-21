#include "planner/base_planner.h"

BasePlanner::BasePlanner(const size_t height, const size_t width) :
	height_(height), 
	width_(width) {
}

void BasePlanner::createPath(const std::vector<size_t> &path, 
														 const size_t start_index, 
														 const size_t goal_index) {
	path_.clear();
	std::vector<size_t> temp_path;
	temp_path.reserve(path.size());

	size_t index = goal_index; 
  size_t x, y;

	while (path.at(index) != start_index) {
		toCoordinate(index, x, y);
		path_.addWaypoint(Waypoint(x, y));
		index = path.at(index);
	}

	toCoordinate(start_index, x, y);
	path_.addWaypoint(Waypoint(x, y));
	path_.reverse();
}