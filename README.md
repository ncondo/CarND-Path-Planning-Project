# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

## Reflection
The car is able to successfully complete one lap around the highway (4.32 miles) without incident. The car does not exceed the speed limit (50mph), and does not drive much slower than the speed limit unless it is obstructed by traffic. This is made possible by using a reference velocity variable which continues to increase when the current velocity is less than 49.5mph unless there is traffic ahead.

The car does not exceed a total acceleration of 10m/s^2 nor a jerk of 10m/s^3. When the car is required to speed up or slow down, it is done so gradually by adding or subtracting the reference velocity in .224m/s (~5mph) increments. The car also adheres to the acceleration and jerk limits when changing lanes due to the spline library used for creating smooth trajectories.

The car navigates the highway without coming into contact with any of the other vehicles on the road. The sensor fusion data is processed so that the ego vehicle avoids coming too close to other cars. I process the data such that I set flags if any cars are less than 30m ahead of me, and I also check adjacent lanes for cars that are close (30m) or will be too close at any point in the near future. If the slow down flag is set, I first check if it is safe to change lanes and do so if possible. If not, I begin slowing down so I do not hit the car ahead.

The car is able to smoothly change lanes when it makes sense and it is safe to do so, otherwise the car stays inside one of the 3 lanes on the right hand side of the road. As mentioned earlier, I use a spline library to generate a smooth path for the car to follow which helps keep the car in its lane and also creates smooth lane changes. The spine library and the code for generating paths is described further in the Details section below.

## Details
The car uses a perfect controller and will visit every (x,y) point it receives in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. I use a convenient spline library in order to generate a smooth trajectory of points for the car to follow. Five anchor waypoints are generated: the car's previous location, its current location, and three more that are spaced 30m apart ahead of the car. The spline library is then used to interpolate points in between the anchor waypoints for the car to travel through. Using a reference velocity as described in the Reflection above, I am able to calculate how far apart each point of the spline needs to be set to ensure the car travels at the desired speed.

Since there is some latency between the simulator running and the path planner returning a path, the simulator will continue using points that it was last given. The simulator provides previous_path_x and previous_path_y to help with a smooth transition. These vectors contain the points last given to the simulator controller but with the processed points already removed. To take advantage of this, I use these previous points and only add new points to reach a total of 50 points. For example, if previous_path_x and previous_path_y still contain 30 points, only 20 new points will generated and appended to the previous points. The code for generating the waypoints and using the spline to create a smooth path can be found on lines 343-443 of `main.cpp`.


Here is the data provided from the Simulator to the C++ Program

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time.

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

## Dependencies

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
* [Spline library for creating smooth trajectories](http://kluge.in-chemnitz.de/opensource/spline/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.
