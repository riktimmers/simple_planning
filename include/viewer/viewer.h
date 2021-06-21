#ifndef _H_VIEWER__
#define _H_VIEWER__

#include <opencv2/opencv.hpp>
#include <unordered_set> 
#include "planner/path.h"

struct MouseEvent {
  size_t x = 0;
  size_t y = 0;
  bool left_click = false;
  bool right_click = false;
};

class Viewer {

  const size_t height_; 
  const size_t width_;
  const size_t raster_size_;
  const std::string board_name_{"Planning Demo"};
  const size_t text_offset_{450};
  cv::Mat board_;
  MouseEvent mouse_event_;
  std::unordered_set<size_t> walls_;

  Path path_dijkstra_;
  Path path_a_star_;

  bool start_set_{false};
  bool goal_set_{false};
  size_t start_index_, goal_index_;

public:
  Viewer(const size_t height = 800, const size_t width = 800, const size_t raster_size = 10);
  void update();
  MouseEvent getMouseEvent();
  void setStartPosition(const size_t x, const size_t y);
  void setGoalPosition(const size_t x, const size_t y);
  bool isWall(const size_t x, const size_t y);

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
  static void mouseHandler(int event, int x,int y, int flags, void *mouse_event);

};

#endif 