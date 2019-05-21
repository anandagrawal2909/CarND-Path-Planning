# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

![LaneChange](https://github.com/anandagrawal2909/CarND-Path-Planning/blob/master/lane_change.gif)   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

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
## Implementation

The implementation includes
* Modification of main.cpp
*Let's see these file in detail now.

1. 'In Line #107 to Line#120'
Below code identifies lane number for all cars from sensor fusion 

```cpp
            // Check cars from sensor fusion
            bool is_car_ahead = false;
            bool is_car_left = false;
            bool is_car_right = false;
            for ( int i = 0; i < sensor_fusion.size(); i++ ) {
                float d = sensor_fusion[i][6]; 
                int car_lane = -1;
                if ( d > 0 && d < 4 ) {
                  car_lane = 0;
                } else if ( d > 4 && d < 8 ) {
                  car_lane = 1;
                } else if ( d > 8 && d < 12 ) {
                  car_lane = 2;
                }
```
Also, the `speed` is calculated using `vx` and `vy` values.
```cpp
speed = sqrt(vx * vx + vy * vy);
```

2. `line 136 to 144` --> Check if car is in safe distance of 30meters
'''cpp
               //check if car is in safe distance of > 30meters   
                if ( car_lane == lane ) {
                  is_car_ahead |= check_car_s > car_s && check_car_s - car_s < 30;
                } else if ( car_lane - lane == -1 ) {
                  is_car_left |= car_s - 30 < check_car_s && car_s + 30 > check_car_s;
                } else if ( car_lane - lane == 1 ) {
                  is_car_right |= car_s - 30 < check_car_s && car_s + 30 > check_car_s;
                }
            }

'''

3. `Turn car based on state` 
'''cpp
            if ( is_car_ahead ) { 
	      //turn_left
              if ( !is_car_left && lane > 0 ) {
                lane--; 
              } 
	      //turn_Right
	      else if ( !is_car_right && lane != 2 ){
                // if there is no car right and there is a right lane.
                lane++; 
              } 
	      //Reduce speeed
	      else {
                speed_diff -= MAX_ACC;
              }
            } 
	    
	    // if we are not on the center lane then come back to center
	    else {
              if ( lane != 1 ) { 
                if ( ( lane == 0 && !is_car_right ) || ( lane == 2 && !is_car_left ) ) {
                  lane = 1; 
                }
              }

	      //increase speed.
              if ( ref_vel < MAX_SPEED ) {
                speed_diff += MAX_ACC;
              }
            }
'''

4.  `For speed control'
 *we're creating 3 waypoints 30m aparts from each other. 
 Using `spline`, we're traversing through these points to create our total `50` waypoints for car to move forward.
``cpp
vector<double> next_wp0 = getXY(car_s + 30, (lane_width * lane + lane_width / 2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s + 60, (lane_width * lane + lane_width / 2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s + 90, (lane_width * lane + lane_width / 2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
```