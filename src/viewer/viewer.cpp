#include "viewer/viewer.h"

Viewer::Viewer(const size_t height, const size_t width, const size_t raster_size) : 
  height_(height), 
  width_(width), 
  raster_size_(raster_size),
  board_(height_, width_, CV_8UC3, cv::Scalar(255, 255, 255)) {

  cv::namedWindow(board_name_);
  cv::setMouseCallback(board_name_, mouseHandler, (void*)&mouse_event_);		
}

void Viewer::updateWalls(const MouseEvent &mouse_event) {
  
  if (mouse_event.left_click) {
    addWall(mouse_event);
  }

  if (mouse_event.right_click) {
    removeWall(mouse_event);
  }
}

void Viewer::addWall(const MouseEvent &mouse_event) {
  const size_t index = getIndex(mouse_event.x, mouse_event.y);
  walls_.insert(index);
}

const size_t Viewer::getIndex(const size_t x, const size_t y) {
  const size_t index_x = std::floor(x / raster_size_);
  const size_t index_y = std::floor(y / raster_size_);
  const size_t index = index_x + (index_y * (width_/raster_size_));
  return index;
}

void Viewer::removeWall(const MouseEvent &mouse_event) {
  const size_t index = getIndex(mouse_event.x, mouse_event.y);
  walls_.erase(index);
}

void Viewer::update(Path &path) {
  cv::Mat field;
  board_.copyTo(field);
  MouseEvent mouse_event = getMouseEvent();
  updateWalls(mouse_event);
  
  drawRaster(field);
  drawWalls(field);
  drawPath(field, path);

  cv::imshow(board_name_, field);
}

void Viewer::drawPath(cv::Mat &image, Path &path) {

  if (path.getSize()) {
    for(size_t index = 0; index < path.getSize() - 1; ++index) {
      Waypoint point1 = path.getWaypoint(index);
      Waypoint point2 = path.getWaypoint(index + 1);

      size_t offset = raster_size_ / 2;
      size_t index_x1 = point1.x * raster_size_ + offset;
      size_t index_y1 = point1.y * raster_size_ + offset;
      size_t index_x2 = point2.x * raster_size_ + offset;
      size_t index_y2 = point2.y * raster_size_ + offset;

      cv::line(image, cv::Point(index_x1, index_y1), cv::Point(index_x2, index_y2), cv::Scalar(0, 255, 0), 3);
    }
  }

}

void Viewer::drawRaster(cv::Mat &image) {
  const cv::Scalar black(0, 0, 0); 

  for (size_t raster = raster_size_; raster < width_; raster += raster_size_) {
    cv::line(image, cv::Point(raster, 0), cv::Point(raster, height_), black); 
  }

  for (size_t raster = raster_size_; raster < height_; raster += raster_size_) {
     cv::line(image, cv::Point(0, raster), cv::Point(width_, raster), black);
  }
}

void Viewer::drawWalls(cv::Mat &image) {
  
  for (auto &index: walls_) {
    const	size_t index_x = (index * raster_size_) % width_ ;
    const size_t index_y = std::floor((index / (width_/raster_size_)) * raster_size_);
    cv::rectangle(image, cv::Rect(index_x, index_y, raster_size_, raster_size_), cv::Scalar(0, 0, 0), -1);
  }
}

MouseEvent Viewer::getMouseEvent() {
  return mouse_event_;
}

void Viewer::mouseHandler(int event, int x,int y, int flags, void *mouse_event) {
  MouseEvent* mouse_event_ptr = (MouseEvent*)mouse_event;
  mouse_event_ptr->x = x;
  mouse_event_ptr->y = y;

  if (event == cv::EVENT_LBUTTONDOWN) {
    mouse_event_ptr->left_click = true;
  } else if (event == cv::EVENT_LBUTTONUP) {
    mouse_event_ptr->left_click = false;
  }

  if (event == cv::EVENT_RBUTTONDOWN) {
    mouse_event_ptr->right_click = true;
  } else if (event == cv::EVENT_RBUTTONUP) {
    mouse_event_ptr->right_click = false;
  }
}
