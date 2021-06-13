#ifndef _H_VIEWER__
#define _H_VIEWER__

#include <opencv2/opencv.hpp>
#include <vector>
#include <unordered_set> 
#include "planner/path.h"

struct MouseEvent {
  size_t x = 0;
  size_t y = 0;
  bool left_click = false;
  bool right_click = false;
};

struct Point {
  size_t x;
  size_t y;
};

class Viewer {

  const size_t height_; 
  const size_t width_;
  const size_t raster_size_;
  const std::string board_name_{"Planning Demo"};
  cv::Mat board_;
  MouseEvent mouse_event_;
  std::unordered_set<size_t> walls_;

public:
  Viewer(const size_t height = 800, const size_t width = 800, const size_t raster_size = 10);
  
  inline size_t getWidth() {
    return width_;
  } 

  void update(Path &path);

  inline std::unordered_set<size_t> getWalls() {
    return walls_;
  }

  MouseEvent getMouseEvent();

private:
  void updateWalls(const MouseEvent &mouse_event);
  void removeWall(const MouseEvent &index);
  void addWall(const MouseEvent &index);
  void drawPath(cv::Mat &image, Path &path);
  void drawRaster(cv::Mat &image);
  void drawWalls(cv::Mat &image);
  const size_t getIndex(const size_t x, const size_t y);
  static void mouseHandler(int event, int x,int y, int flags, void *mouse_event);

};

#endif 