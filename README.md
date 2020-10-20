## Unscented Kalman Filter
In this project, a demonstration of the Unscented Kalman filter is presented to estimate the position of a vehicle with LIDAR & RADAR sensors. This demonstration is based on a simulator developed by Udacity and was completed as part of Udacity's Self-Driving Car Nanodegree. The [Udacity github repo](https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project) for this project has all the details about the software used in the simulator and the installation instructions.

### Summary of Setup Instructions
1. The project uses [uWebSocketIO](https://github.com/uNetworking/uWebSockets) for communication between the user-written algorithm and the simulator. Udacity has provided bash scripts to install this in both [Linux](https://github.com/chandrusuresh/CarND-Extended-Kalman-Filter-Project-master/blob/master/install-ubuntu.sh)/[Mac](https://github.com/chandrusuresh/CarND-Extended-Kalman-Filter-Project-master/blob/master/install-mac.sh) environments. These scripts are included in this repo.
2. The simulator can be downloaded from [here](https://github.com/udacity/self-driving-car-sim/releases).

### Basic Build Instructions (Linux/Mac)
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF`
This should setup a listener for the code to get data from the simulator.
5. Launch the simulator from a terminal: `./`
6. Select the Kalman Filter project and click start to start the simulation.

These steps should get any user in a Linux/Mac environment up and running with the code.

## Documentation
Refer to [UnscentedKalmanFilter.ipynb](https://github.com/chandrusuresh/CarND-Unscented-Kalman-Filter-Project-master/blob/master/UnscentedKalmanFilter.ipynb) for documentation of theory and code.
