# Unscented Kalman Filter
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)


[//]: # (Image References)
[image1]: ./images/image1.png
[image2]: ./images/image2.png
[image3]: ./images/image3.png
[image4]: ./images/image4.png

## Basic instructions to run the code
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./UnscentedKF `

## [Rubric](https://review.udacity.com/#!/rubrics/783/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.
---

### Compiling

#### 1. Code must compile without errors with cmake and make. Given that we've made CMakeLists.txt as general as possible, it's recommended that you do not change it unless you can guarantee that your changes will still compile on any platform.

My code is compiling ok, as you can see here:
`Scanning dependencies of target UnscentedKF`
`[ 25%] Building CXX object CMakeFiles/UnscentedKF.dir/src/ukf.cpp.o`
`[ 50%] Building CXX object CMakeFiles/UnscentedKF.dir/src/main.cpp.o`
`[ 75%] Building CXX object CMakeFiles/UnscentedKF.dir/src/tools.cpp.o`
`[100%] Linking CXX executable UnscentedKF`
`[100%] Built target UnscentedKF`

### Accuracy

#### 1. Your algorithm will be run against "obj_pose-laser-radar-synthetic-input.txt". We'll collect the positions that your algorithm outputs and compare them to ground truth data. Your px, py, vx, and vy RMSE should be less than or equal to the values [.09, .10, .40, .30].

My final accuracy for the **obj_pose-laser-radar-synthetic-input.txt** was **[0.0657, 0.0804, 0.3038, 0.1698]**
A good improvement was change the matrix P to:
`std_radr_*std_radr_, 0, 0, 0, 0,`
`0, std_radr_*std_radr_, 0, 0, 0,`
`0, 0, 1, 0, 0,`
`0, 0, 0, std_radphi_, 0,`
`0, 0, 0, 0, std_radphi_;`

### Follows the Correct Algorithm

#### 1. While you may be creative with your implementation, there is a well-defined set of steps that must take place in order to successfully build a Kalman Filter. As such, your project should follow the algorithm as described in the preceding lesson.

I have followed the class for all implementation. At the code is possible to see where in the class I got the reference for the code, just like [this line](https://github.com/marlosdamasceno/unscented-kalman-filter/blob/master/src/ukf.cpp#L179) where I show that was from lesson 8 section 21.

#### 2. Your algorithm should use the first measurements to initialize the state vectors and covariance matrices.

The first measurement is handled at [ukf.cpp](https://github.com/marlosdamasceno/unscented-kalman-filter/blob/master/src/ukf.cpp#L92) from line 92 to line 127.

#### 3. Upon receiving a measurement after the first, the algorithm should predict object position to the current timestep and then update the prediction using the new measurement.

The predict and update steps can be found in [ukf.cpp](https://github.com/marlosdamasceno/unscented-kalman-filter/blob/master/src/ukf.cpp#L129) from line 129 to line 137.


#### 4. Your algorithm sets up the appropriate matrices given the type of measurement and calls the correct measurement function for a given sensor type.

The code can handle both Radar and Lidar sensors, you can find in [ukf.cpp](https://github.com/marlosdamasceno/unscented-kalman-filter/blob/master/src/ukf.cpp#L129) from line 129 to line 137.


### Code Efficiency

#### 1. Your algorithm should avoid unnecessary calculations.

It follows the guidelines of the lessons.

---

### Discussion
The Slack community helped me a lot, again!
Moreover, I had some issues at the beginning because some parts of code the shows an error on the IDE.
![alt text][image1]
![alt text][image2]
However, that does not avoid the code run. Therefore, I left it on that way.

My final result is on the images below.
![alt text][image3]
![alt text][image4]
