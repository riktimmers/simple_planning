#include "viewer/viewer.h"
#include "planner/dijkstra.h"

int main(int argc, char **argv) {
  const size_t height = 800;
  const size_t width = 800;
  const size_t raster_width = 10;
  
  Viewer viewer(height, width, raster_width);
  Dijkstra dijkstra(height/raster_width, width/raster_width);
  Path path;

  const size_t start_x = 0;
  const size_t start_y = 0;
  const size_t goal_x = width - 1;
  const size_t goal_y = height - 1;
  dijkstra.setStartPoint(start_x / raster_width, start_y / raster_width);
  viewer.setStartPosition(start_x, start_y);
  dijkstra.setGoalPoint(goal_x / raster_width, goal_y / raster_width);
  viewer.setGoalPosition(goal_x, goal_y);

   while (true) {
    char key = cv::waitKey(1);
    dijkstra.createOccupancyMap(viewer.getWalls());
    path.clear();
    bool found_plan = dijkstra.plan();
    
    if (found_plan) {
      path = dijkstra.getPath();
    }

    viewer.update(path);

    if (key == 'q') {
      break;
    } else if (key == 's') {
      MouseEvent mouse_event = viewer.getMouseEvent();

      if (!viewer.isWall(mouse_event.x, mouse_event.y)) {
        dijkstra.setStartPoint(std::floor(mouse_event.x / raster_width), std::floor(mouse_event.y / raster_width));
        viewer.setStartPosition(mouse_event.x, mouse_event.y);
      }
    } else if (key == 'g') {
      MouseEvent mouse_event = viewer.getMouseEvent();

      if (!viewer.isWall(mouse_event.x, mouse_event.y)) {
        dijkstra.setGoalPoint(std::floor(mouse_event.x / raster_width), std::floor(mouse_event.y / raster_width));
        viewer.setGoalPosition(mouse_event.x, mouse_event.y);
      }
    }
  }
}