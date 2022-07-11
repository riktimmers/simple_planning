# simple_planning
Some simple GUI created with OpenCV that shows the Dijkstra and A* planning algorithms. 

To build:
```
$ mkdir -p build ; cd build ; cmake -DCMAKE_BUILD_TYPE=Release ../ ; make -j5
```

To run the program:
```
$ ./main
```
With the left mouse-button one can place a wall, with right mouse-button the wall is removed.
The starting location can be changed with 's' key and the goal location with 'g'. 
Press 'q' to quit the program. 