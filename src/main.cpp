#include "viewer/viewer.h"
#include "planner/dijkstra.h"

int main(int argc, char **argv) {
  const size_t height = 800;
  const size_t width = 800;
  const size_t raster_width = 10;
  
  Viewer viewer(height, width, raster_width);
  Dijkstra dijkstra(height/raster_width, width/raster_width);
  Path path;
  dijkstra.setStartPoint(10, 11);
  dijkstra.setGoalPoint(77, 57);

  while (true) {
    char key = cv::waitKey(1);
    path.clear();
    dijkstra.createOccupancyMap(viewer.getWalls());

    if (dijkstra.plan()) {
      path = dijkstra.getPath();
    }
    viewer.update(path);

    if (key == 'q') {
      break;
    }
  }
}