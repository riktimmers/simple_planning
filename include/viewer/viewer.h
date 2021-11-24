#ifndef _H_VIEWER__
#define _H_VIEWER__

#include <opencv2/opencv.hpp>
#include <unordered_set> 
#include <array>
#include "planner/path.h"

struct MouseEvent { // Struct for holding Mouse information, x, y location, and if left or right button was pressed.
  size_t x = 0;
  size_t y = 0;
  bool left_click = false;
  bool right_click = false;
};

class Viewer {

  const size_t height_; 
  const size_t width_;
  const size_t raster_size_;
  const std::string board_name_{"Planning Demo"}; // Title.
  const size_t text_offset_{450}; // Width of right side where text will be.

  std::array<std::string, 7> text_; // Array for holding text printed on the side.
  cv::Mat board_; // The board is a Matrix (OpenCv Image).
  MouseEvent mouse_event_;  // Hold information about the mouse on the board.
  std::unordered_set<size_t> walls_; // Contains information where wall are placed.

  Path path_dijkstra_; // The Dijkstra Path.
  Path path_a_star_; // The A* Path.

  bool start_set_{false}; // The start position hasn't been set by user.
  bool goal_set_{false}; // The goal position hasn't been set by user.

  const cv::Scalar start_color_{224, 220, 0}; // Start position color
  const cv::Scalar goal_color_{0, 0, 255}; // Goal position color
  const cv::Scalar dijkstra_path_color_{0, 255, 0}; // Dijkstra's path color.
  const cv::Scalar a_star_path_color_{255, 0, 0}; // A*'s path color.

  size_t start_index_, goal_index_; // Start and goal indices.

public:
  Viewer(const size_t height = 800, const size_t width = 800, const size_t raster_size = 10); // The GUI with default size values.
  void update();
  MouseEvent getMouseEvent();
  void setStartPosition(const size_t x, const size_t y); // Set the new starting position.
  void setGoalPosition(const size_t x, const size_t y); // Set the new goal position.
  bool isWall(const size_t x, const size_t y); // Check if position is a wall or not.

  void setDijkstraPath(const Path &path);
  void setAStarPath(const Path &path);
  
  inline size_t getWidth() {
    return width_;
  } 

  inline std::unordered_set<size_t> getWalls() {
    return walls_;
  }

private:
  void drawText(cv::Mat &image);
  void drawStartAndGoal(cv::Mat &image);
  void updateWalls(const MouseEvent &mouse_event);
  void removeWall(const MouseEvent &index);
  void addWall(const MouseEvent &index);
  void drawPath(cv::Mat &image, Path &path, const cv::Scalar &color);
  void drawRaster(cv::Mat &image);
  void drawWalls(cv::Mat &image);
  const size_t getIndex(const size_t x, const size_t y);
  static void mouseHandler(int event, int x,int y, int flags, void *mouse_event); // Register the mouse 
  void addText(cv::Mat &image, const std::string text, const size_t line_nr = 1);

};

#endif 