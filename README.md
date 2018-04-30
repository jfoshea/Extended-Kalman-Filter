# Extended Kalman Filter Project

## Overview 
This project utilizes a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.  Radar measurements are non-linear and the extended Kalman filter (EKF) is the nonlinear version of the Kalman filter which linearizes about an estimate of the current mean and covariance. This project is a fusion of linear Lidar and non-linear Radar measurements.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros

## Build Instructions 
1. Clone the Extended Kalman Filter git repository
    ```  
    $ git clone https://github.com/jfoshea/Extended-Kalman-Filter.git
    ```
2. Make a build directory if it does not exist
    ```  
    $ mkdir build && cd build 
    ```
3. Compile 
    ```  
    $ cmake .. && make 
    ```
4. Run Extended Kalman Filter 
    ```  
    $ ./ExtendedKF 
    ```
