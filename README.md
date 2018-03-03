# Run Away Robot with Unscented Kalman Filter Bonus Challenge
Self-Driving Car Engineer Nanodegree Program

---

### Overview

This repository contains all the code needed to complete the Bonus Challenge: Catch the Run Away Car with Unscented Kalman Filter.

### Project Introduction

In this project, not only do you implement an UKF, but also use it to catch an escaped car driving in a circular path. 
The run away car will be being sensed by a stationary sensor, that is able to measure both noisy lidar and radar data. The capture vehicle will need to use these measurements to close in on the run away car. To capture the run away car the capture vehicle needs to come within .1 unit distance of its position. However the capture car and the run away car have the same max velocity, so if the capture vehicle wants to catch the car, it will need to predict where the car will be ahead of time.

### Running the Code

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

`mkdir build && cd build`

`cmake .. && make` 

`./UnscentedKF`

Note that the programs that need to be written to accomplish the project are `src/ukf.cpp`, `ukf.h`, and `main.cpp` which will use some strategy to catch the car, just going to the cars current esimtated position will not be enough since the capture vehicle is not fast enough. There are a number of different strategies you can use to try to catch the car, but all will likely involve predicting where the car will be in the future which the UKF can do. Also remember that the run away car is simplifying moving a circular path without any noise in its movements.


Here is the main protocol that `main.cpp` uses for uWebSocketIO in communicating with the simulator.

**INPUT**: values provided by the simulator to the C++ program



// current noiseless position state of the capture vehicle, called hunter

["hunter_x"]

["hunter_y"]

["hunter_heading"]

// get noisy lidar and radar measurements from the run away car.

["lidar_measurement"]

["radar_measurement"]


**OUTPUT**: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["turn"] <= the desired angle of the capture car "hunter" no limit for the angle

["dist"] <= the desired distance to move the capture car "hunter" can't move faster than run away car



## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4
* uWebSocketIO

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF` 

## Implementation

To intercept the run-away robot, the path with the shortest time is the time where the hunter (vehicle controlled by me) and 
the run-away robot reach the same point at the same time.

To find out the point, I use binary search within the time interval (0s, 8s), the result will give us the intercept point.

The algorithm is implemented in void findInterceptLocation(double hunter_x, double hunter_y, UKF* ukf, double& intercept_x, double& intercept_y) in main.cpp

Here is the details:

```c++
Set t_max (maximum search time) = 8s
Set t_min (minimum search time) = 0s
Set hunter velocity = 5m/s
Set maximum iteration = 10
Set best_intercept_time_diff = 1000s

While t_max > t_min and the number of iteration is less than maximum iteration:
    set t_mid = (t_max + t_min)/2
    use UKF to find out the location of run-away robot at t_mid, denote it as try_intercept_point
    find the distance between the hunter and try_intercept_point
    find the time for the hunter to reach try_intercept_point, denote it as time_intercept
    
    calculate the difference between t_mid and time_intercept, denote it as intercept_time_diff
    
    if best_intercept_time_diff > intercept_time_diff
        Set best_intercept_time_diff = intercept_time_diff
        Set result_intercept_point = try_intercept_point
    
    if intercept_time_diff > t_mid
        t_min = t_mid, it is because we need more time for the robot to make mistake
    else if intercept_time_diff < t_mid    
        t_max = t_mid, try to find an intercept path with shorter time.
        
Return the result_intercept_point
```

### Result
Here is the [result](https://youtu.be/40h_eBTMvLg) . 


    
    


