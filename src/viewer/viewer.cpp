#include "viewer/viewer.h"

Viewer::Viewer(const size_t height, const size_t width, const size_t raster_size) : 
  height_(height), 
  width_(width), 
  raster_size_(raster_size),
  board_(height_, width_ + text_offset_, CV_8UC3, cv::Scalar(255, 255, 255)) {

  cv::namedWindow(board_name_);
  drawText(board_);
  cv::setMouseCallback(board_name_, mouseHandler, (void*)&mouse_event_);		
}

void Viewer::setDijkstraPath(const Path &path) {
  path_dijkstra_ = path;
}

void Viewer::setAStarPath(const Path &path) {
  path_a_star_ = path;
}

void Viewer::drawText(cv::Mat &image) {
  cv::rectangle(image, cv::Rect(width_, 0, text_offset_, height_), cv::Scalar(50, 0, 20), -1);
  size_t x_offset = 20;
  size_t y_offset = 20;
  float font_scale = 1.2;
  cv::Scalar font_color(0, 255, 0);
  int font = cv::FONT_HERSHEY_PLAIN;
  cv::putText(image, "- Left mouse button to add wall", cv::Point(width_ + x_offset, y_offset),
              font, font_scale, font_color);
  cv::putText(image, "- Right mouse button to remove wall", cv::Point(width_ + x_offset, y_offset*2), 
              font, font_scale, font_color);
  cv::putText(image, "- 's' to set start location", cv::Point(width_ + x_offset, y_offset*3), 
              font, font_scale, font_color);
  cv::putText(image, "- 'g' to set goal location", cv::Point(width_ + x_offset, y_offset*4), 
              font, font_scale, font_color);

}

bool Viewer::isWall(const size_t x, const size_t y) {
  return walls_.count(getIndex(x, y)) > 0; 
}

void Viewer::setStartPosition(const size_t x, const size_t y) {
  
  if (x > width_) {
    return;
  }

  start_index_ = getIndex(x, y);
  start_set_ = true;
}

void Viewer::setGoalPosition(const size_t x, const size_t y) {
  
  if (x > width_) {
    return;
  }

  goal_index_ = getIndex(x, y);
  goal_set_ = true;
}

void Viewer::drawStartAndGoal(cv::Mat &image) {
  if (start_set_) {
    const	size_t x = (start_index_ * raster_size_) % width_ ;
    const size_t y = std::floor((start_index_ / (width_/raster_size_)) * raster_size_);

    cv::rectangle(image, cv::Rect(std::floor(x), std::floor(y), raster_size_, raster_size_), cv::Scalar(220, 0, 0), -1);
  }

  if (goal_set_) {
    const	size_t x = (goal_index_ * raster_size_) % width_ ;
    const size_t y = std::floor((goal_index_ / (width_/raster_size_)) * raster_size_);

    cv::rectangle(image, cv::Rect(std::floor(x), std::floor(y), raster_size_, raster_size_), cv::Scalar(0, 0, 255), -1);
  }
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
  if (mouse_event.x < width_) {
    const size_t index = getIndex(mouse_event.x, mouse_event.y);

    if (start_set_) {
      if (index == start_index_) {
        return;
      }
    }

    if (goal_set_) {
      if (index == goal_index_) {
        return;
      }
    }
    walls_.insert(index);
  }
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

void Viewer::update() {
  cv::Mat field;
  board_.copyTo(field);
  MouseEvent mouse_event = getMouseEvent();
  updateWalls(mouse_event);

  drawText(field);
  drawStartAndGoal(field);
  drawRaster(field);
  drawWalls(field);
  drawPath(field, path_dijkstra_, cv::Scalar(0, 255, 0));
  drawPath(field, path_a_star_, cv::Scalar(255, 0, 0));

  cv::imshow(board_name_, field);
}

void Viewer::drawPath(cv::Mat &image, Path &path, const cv::Scalar &color) {

  if (path.getSize()) {
    for(size_t index = 0; index < path.getSize() - 1; ++index) {
      Waypoint point1 = path.getWaypoint(index);
      Waypoint point2 = path.getWaypoint(index + 1);

      size_t offset = raster_size_ / 2;
      size_t index_x1 = point1.x * raster_size_ + offset;
      size_t index_y1 = point1.y * raster_size_ + offset;
      size_t index_x2 = point2.x * raster_size_ + offset;
      size_t index_y2 = point2.y * raster_size_ + offset;

      cv::line(image, cv::Point(index_x1, index_y1), cv::Point(index_x2, index_y2), color, 3);
    }
  }

}

void Viewer::drawRaster(cv::Mat &image) {
  const cv::Scalar black(0, 0, 0); 

  for (size_t raster = raster_size_; raster <= width_; raster += raster_size_) {
    cv::line(image, cv::Point(raster, 0), cv::Point(raster, height_), black); 
  }

  for (size_t raster = raster_size_; raster < height_; raster += raster_size_) {
     cv::line(image, cv::Point(0, raster), cv::Point(width_, raster), black);
  }
}

void Viewer::drawWalls(cv::Mat &image) {
  
  for (auto &index: walls_) {
    const	size_t x = (index * raster_size_) % width_ ;
    const size_t y = std::floor((index / (width_/raster_size_)) * raster_size_);
    cv::rectangle(image, cv::Rect(x, y, raster_size_, raster_size_), cv::Scalar(0, 0, 0), -1);
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
