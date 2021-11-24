#include "viewer/viewer.h"
#include "planner/dijkstra.h"
#include "planner/a_star.h"

int main(int argc, char **argv) {
  const size_t height = 800; // Set height of the GUI
  const size_t width = 800; // Set width of the GUI
  const size_t raster_width = 10; // Set size of the squares
  
  Viewer viewer(height, width, raster_width); // Create the viewer
  Dijkstra dijkstra(height/raster_width, width/raster_width); // Create Dijkstra planner
  AStar a_star(height/raster_width, width/raster_width); // Create A* planner

  const size_t start_x = 0; // Start x coordinate
  const size_t start_y = 0; // Start y coordinate
  const size_t goal_x = width - 1; // Goal x coordinate
  const size_t goal_y = height - 1; // Goal y coordinate

  // Set the start and goal positions for the planners and viewer.
  dijkstra.setStartPoint(start_x / raster_width, start_y / raster_width);
  a_star.setStartPoint(start_x / raster_width, start_y / raster_width);
  viewer.setStartPosition(start_x, start_y);
  dijkstra.setGoalPoint(goal_x / raster_width, goal_y / raster_width);
  a_star.setGoalPoint(goal_x / raster_width, goal_y / raster_width);
  viewer.setGoalPosition(goal_x, goal_y);

   while (true) { // Endless loop.
    char key = cv::waitKey(1); // Get key pressed.

    // Update each planners with the new occupancy map (the walls).
    dijkstra.createOccupancyMap(viewer.getWalls());
    a_star.createOccupancyMap(viewer.getWalls());
    bool found_plan_dijkstra = dijkstra.plan(); // Plan and determine if plan was found.
    bool found_plan_a_star = a_star.plan(); // Plan and determine if plan was found.

    // If path was found, give path to viewer.
    if (found_plan_dijkstra) {
      viewer.setDijkstraPath(dijkstra.getPath());
    }

    if (found_plan_a_star) {
      viewer.setAStarPath(a_star.getPath());
    }

    viewer.update(); // Draw everything

    if (key == 'q') { // Handle key presses.
      break;
    } else if (key == 's') {
      MouseEvent mouse_event = viewer.getMouseEvent();

      if (!viewer.isWall(mouse_event.x, mouse_event.y)) {
        dijkstra.setStartPoint(std::floor(mouse_event.x / raster_width), std::floor(mouse_event.y / raster_width));
        a_star.setStartPoint(std::floor(mouse_event.x / raster_width), std::floor(mouse_event.y / raster_width));
        viewer.setStartPosition(mouse_event.x, mouse_event.y);
      }
    } else if (key == 'g') {
      MouseEvent mouse_event = viewer.getMouseEvent();
      
      if (!viewer.isWall(mouse_event.x, mouse_event.y)) {
        dijkstra.setGoalPoint(std::floor(mouse_event.x / raster_width), std::floor(mouse_event.y / raster_width));
        a_star.setGoalPoint(std::floor(mouse_event.x / raster_width), std::floor(mouse_event.y / raster_width));
        viewer.setGoalPosition(mouse_event.x, mouse_event.y);
      }
    }
  }
}