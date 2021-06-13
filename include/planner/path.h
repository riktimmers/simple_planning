#ifndef _H_PATH__
#define _H_PATH__

#include <vector>

struct Waypoint {
  size_t x = 0;
  size_t y = 0;
};

class Path {
  
  std::vector<Waypoint> path_;

public:
  Path(const size_t size = 1000); 
  void addWaypoint(const Waypoint &waypoint);
  Waypoint getWaypoint(size_t index);
  size_t getSize();
  void clear();
  std::vector<Waypoint> getPath();

};

#endif 