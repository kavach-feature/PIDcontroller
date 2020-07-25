# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

The project inolves two simple routines - UpdateError and TotalError

UpdateError - Computes the proportional error, Differential error and Integral error based on the current CTE.

TotalError - Computes the total error based on the assigned Kp, Ki and Kd values.

# Importance of P, I and D in the simulator
P -It sets the steering angle in proportion to CTE with a proportional factor tau.
I- It is the integral or sum of error to deal with inherent bias existing in the car like steering drift.
D- It is the differential component of the controller which helps to take temporal derivative of error.


The goal of the project is to ensure CTE is always small and ensure proper steering so that the car is not driving outside the lanes or during curves.
# Parameter tuning of P, I , D

It is very difficult to manually identify a correct set of parameters. hence based on the mentor's advice and knowledge hub, following parameters were choses as P,I,D = (0.04,0.000,1.25) in the starters.

The following routine was used initially to tune each of the parameter individually. 
```
     // For proportional.
     pid.Init(0.04, 0.0, 0.0);

    // For integral.
    pid.Init(0.0, 0.00, 0.0);
     
     //For Differential
    pid.Init(0.0, 0.0, 1.25);
    
```
After every testing the an increment of 0.01 for P, 0.0001 for I , 0.1 for D was done to re-test with new values. However, there was no improvement in the stabilization of CTE and the car was moving outside the lane within 30-40 iterations.

At this stage, implementation of Twiddle began to make the parameter choosing more deterministic instead of empirical calculation based on observation.


# Twiddle implementation

Thanks to Dhanoop Karunakaran's pseudocode in Python that lucidly explained the necessary changes in Sebastian's Twiddle exercise -->[Dhanoop Karunakarna's implementation](https://medium.com/intro-to-artificial-intelligence/pid-controller-udacitys-self-driving-car-nanodegree-c4fd15bdc981). The pesudo-code was used as reference to implement Twiddle.

Implementations to dp parameters were made to ensure steering value did not cross 20% within 5 iterations that can cause jerks. 

Twiddle implementation also did not stabilize the car's CTE and was changing spuriously within 40-50 iterations. Tuning of parameters like dp, max number of iterations were made but to no avail.

Later during debugging the code, here's what I found-

The test platform was plain Intel CPU system without any GPU. On this system, the simulator quickly outputs CTE values that are unable to be handle  Twiddle algorithm's steering value within 40-50 iterations. Furthermore, the simulator always outputs different CTE values.

However, when same code ran on Udacity's GPU workspace, the car ran without any issues with/without twiddle implementation. [Final test  on Udacity's GPU workspace in Youtube](https://youtu.be/79NAOetFv48)

It's unclear as to why CTE values are affected by GPU performance as the simulator is a functional simulator.



---
# Udacity Project Instructions
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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)



