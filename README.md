# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program
---

## The details of the model
Here I implement the Model Predictive Control(MPC) to control the vehicle. In this model, the control input is 
(delta, a) denote the (steer_value, throttle_value). The state vector is the (x, y, psi, v, cte, epsi). The model is consist
 of the following part:
 * **A reference trajectory(A Polynomial)**: it is get from the path planing part. In this program, it is some points in the global coordinate.
 I transform these points to the vehicle coordinate, and use a Polynomial with 3 order to fit the trajectory.
 * **A vehicle model** (to calculate the state of the vehicle step by step), The detail of the vehicle model is shown in below:
 ```$xslt
 // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
 // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
 // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
 // v_[t+1] = v[t] + a[t] * dt
 // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
 // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
 ```
 * **Some constraints**: lower_bound and upper_bound of delta and a. 
 * **Appropriate N and dt**
 * **A cost function**: it is consist of some terms that we want to minimize, such as the CTE, EPSI, and other term which make the
 vehicle move smoother or make the vehicle move at a fixed speed. 
 * **An optimizer**: To optimize the (delta, a) to minimize the cost. After optimize, we will get a series of delta, a, x, y, psi, cte, spsi
  from 1~N. These are the state of each time step we calculate.
  
## How to choose the N and dt
I choose N=16 and dt=0.05. So the T=N*dt=0.8 second. The dt I choose is small which means actuate frequency is high.
When I choose N=20 and dt=0.1, vehicle may be out of the road when it turns with a high speed. I think it's because the actuate frequency is not enough.
So I set the dt as 0.05.
## How to deal with the latency
Since the latency is set as 100ms, which means latency=2*dt, I choose the delta and a at time step 2 after optimize. The code is:
```$xslt
return {solution.x[delta_start + latency], solution.x[a_start + latency]};
```
## The cost function
The cost function consists of several terms:
* sum(cte^2) and sum(epsi^2), which to control the cte and epsi as small as possible
* sum((v-ref_v)^2), which to make the speed fix at ref_v, the ref_v we set in the program is 60 mph.
* sum(delta^2) and sum(a^2), which to actuate as small as possible 
* sum(400*(delta^t+1 - delta^t)^2) and sum((a^t+1 - a^t)^2), which makes the steering and  acceleration smoother.
## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
