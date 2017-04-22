# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

---
## Summary
This project implements a Extended Kalman filter to perform sensor fusion for estimating the state of a moving object. The Extended Kalman Filter allows for incorporating non-linear prediction and measurement models into the gaussian state estimation using the first order taylor series expansion of each function; as a result, you have to calculate the Jacobian for each non-linear predition or measurement function. The sensors used are radar and lidar. Although normally you have to detect the object of interest in the laser or radar measurement, for this project the measurements provided are already of the detected object.

## Results

The state vector being estimated is the x position,y position, x velocity, y velocity (x,y,vx,vy). Using the ground truth provided I got a RMSE of: (0.096,0.085,0.41,0.43) for (x,y,vx,vy). 

Here is a image showing a comparison of the ground truth with the Kalman Filter estimation:
![Alt text](pictures/radar-lidar.png?raw=true "Visualize result")

I also experimented with using just lidar or just radar:
- Just lidar RMSE = (0.12,0.098, 0.55,0.46)
- Just radar RMSE = (0.19,0.27,0.52,0.65)
- Both sensors RMSE = (0.096,0.085,0.41,0.43
- These results show that by itself Lidar is closer to the ground truth, but when fused with radar measurements in the EKF you get 
a more accurate estimation. 


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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./ExtendedKF ../data/obj_pose-laser-radar-synthetic-input.txt`
