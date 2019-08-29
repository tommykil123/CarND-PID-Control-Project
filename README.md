# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

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
## Overview
This project goes over the basics of the PID controller and implementing such control system in a simulation environment.   
The PID controller is composed of three main gains:
1. Proportional Gain
2. Integral Gain
3. Derivative Gain
As the name suggests, these gains are applied to the error to come up with a viable steering angle to keep the vehicle in the lane.
### Cross Track Error
The cross track error was the error associated with the deviation from the trajectory of the lane. In this project, the trajectory would have been the center of the lane.
### The Gains
The individual gains (P, I, D) are applied to a form of the error.
* P Gain: The pgain sets the steering angle proportional factor to CTE with the P gain.
``` c++
- Kp * cte
```
* I Gain: The igain sets the steering angle with the errors accumulated throughout the whole run. This igain is used to counteract the bias that might prevent the vehicle from centering.
``` c++
int_cte += cte
- Ki * int_cte;
```
* D Gain: The dgain sets the steering angle with the difference in error from timestep t and t-1. This dgain is used to counter the overshooting that might occur as a result of the pgain.
``` c++
diff_cte = cte - prev_cte
- Kd * diff_cte
```
### Steer Output
The output of the sum of these values above constitutes the steering angle that drives the vehicle.   
``` c++ 
steer_angle = -Kp * cte - Ki * int_cte - Kd * diff_cte
```

## Tuning
Tuning of the PID controller was very tedious as there with three parameters pgain, igain, and dgain to tune. However, using the Twiddle formula introduced by Sebastian in the Udacity course, I was able to automate finding valid values of the gain to allow the vehicle to follow the lane.
``` c++
function(tol=0.2) {
    p = [0, 0, 0]
    dp = [1, 1, 1]
    best_error = move_robot()
    loop untill sum(dp) > tol
        loop until the length of p using i
            p[i] += dp[i]
            error = move_robot()

            if err < best_err
                best_err = err
                dp[i] *= 1.1
            else
                p[i] -= 2 * dp[i]
                error = move_robot()

                if err < best_err
                    best_err = err
                    dp[i] *= 1.1
                else
                    p[i] += dp[i]
                    dp[i] *= 0.9
    return p
}
```
### Tuning methodology
Because running twiddle algorithm for every run of the track takes a long time, I divided the tuning into two parts.   
1. Tune for the first ~5 seconds of the track resetting the simulation after each iteration.
* This allowed me to see which values P, I, D had the most effect on the system
* Allowed me to see factor of dP, dI, dD (ex: P values can be set +- 0.01 where as I was much more senstive with +- 0.00001)
2. Tune for the next 30 seconds of the track resetting the simulation after each iteration.

## Final Thoughts
The project gave a very good overview to tuning a PID controller. The automation process made it much easier to see the effects of the individual tuning parameters.   
Though the vehicle is able to go around the track, if given the time, I would like to fine tune this so that it is much smoother.
