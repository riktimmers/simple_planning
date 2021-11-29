project(simple_planning)
cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS -Wall)

find_package(OpenCV REQUIRED)

include_directories(
  include
)

set(source 
  "src/main.cpp"
  "src/viewer/viewer.cpp"
  "src/planner/dijkstra.cpp"
  "src/planner/a_star.cpp"
  "src/planner/base_planner.cpp"
)

add_executable(main ${source})
target_link_libraries(main ${OpenCV_LIBRARIES})
