# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Implementation

#### The Model

- I used the kinetmatic model taught in the lectures. The model describes how the state vector [x,y,psi,v, cte, epsi] changes over time. Where x, y is the cars position, psi is the orientation and v is the velocity. The cte is the cross track error and is the difference between the center of the road and the vehicles position. The epsi is the orientation error and its the difference between the vehicle orientation and the trajectory orientation. For control inputs I use the steering wheel [delta] and throttle/brake [a] giving a actuator input of [delta,a]. 

The update equations for this model are:
- x' = x + v * cos(psi) * dt 
- y' = y + v * sin(psi) * dt
- psi' = psi + v/L * delta * dt
- v' = v + a * dt
- cte' = cte + v * sin(epsi) * dt
- epsi' = epsi + v/L * delta * dt
- Where dt is the change in time, and L is the distance from the front of the vehicle to its center of gravity.


#### Timestep Length and Elapsed Duration (N & dt)

For MPC you must decide the time over which future predictions are made, or T. 

T = N * dt

To choose N and dt I used trial and error to find numbers that worked. I started with N = 10 and dt = 0.1 , therefore T=1, these numbers worked good if you did not take into account latency, but once a 100ms latency was added, the controller became unstable. How I was able to deal with it was I increased dt so that it was larger than latency: 0.125. N I kept at 10, as I found making changes to N drastically changed the controller, and required that you then had to change dt. When experimenting with dt I chose values between 0.1 and 0.15 until I found one that I thought worked the best. 


#### Polynomial Fitting and MPC Preprocessing

To start I followed the project walkthrough and the cars position is subtracted from the waypoints position, so that the reference origin for the car is a (0,0) in addtion the waypoint points are rotated by (0 - cars orientation) so that the cars reference orientation is 0 also. This simplifies things when calculating the inital state and will make doing the polynomial fit easier since the cars reference orientation is 0 we will mostly be fitting a horizontal line and not a vertical line.

With the intial positions and orientation set to zero the intial state is [0,0,0,v,cte,epsi]

The initial cte in the state above is calculated by first fitting a 3rd degree polynomial to the waypoints and because the cars reference orientation is zero you just evaluate the polynomial at x = 0 and that is the cte. Also the epsi is calculated by finding the difference between the calculated lines orienation and the cars orientation and is again simplified because of the initial transformation. These calculations are at line 123 in main.cpp.

#### Model Predictive Control with Latency

As discussed above I handled latency by increasing the dt so that it was greater than the latency. This way you don't have to worry about the cars state being much different then the one you predicted the actuations for. 


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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

