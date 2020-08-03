# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.

## Tasks
### MP.0 Mid-Term Report
Explained in this Readme file.

### MP.1 Data Buffer Optimization
Implemented in [src/MidTermProject_Camera_Student.cpp Line 117-119](./src/MidTermProject_Camera_Student.cpp#L117-L119).

### MP.2 Keypoint Detection
Implemented in [src/matching2D_Student.cpp Line 140-265](./src/matching2D_Student.cpp#L140-L265).

### MP.3 Keypoint Removal
Implemented in [src/MidTermProject_Camera_Student.cpp Line 157-170](./src/MidTermProject_Camera_Student.cpp#L157-L170).

### MP.4 Keypoint Descriptors 
Implemented in [src/matching2D_Student.cpp Line 59-97](./src/matching2D_Student.cpp#L59-L97).

### MP.5 Descriptor Matching 
Implemented in [src/matching2D_Student.cpp Line 15-53](./src/matching2D_Student.cpp#L15-L53). 

### MP.6 Descriptor Distance Ratio 
Implemented in [src/matching2D_Student.cpp Line 43-52](./src/matching2D_Student.cpp#L43-L52).

### MP.7 Performance Evaluation 1

Number of detected keypoints on the preceding vehicle

| Detector type | Total number of detected keypoints | Keypoint distribution |
| ------------- | ------------- | ------------- |
| Shi-Tomasi  | 1179 | Good |
| Harris  | 248 | Too few point |
| FAST  | 1491 | Good, but few point around number plate, many outside car | 
| BRISK  | 2762 | Good, similar to FAST | 
| ORB  | 1161 | Not good, most keypoints concentrated in small area | 
| AKAZE  | 1670 | Good, most keypoints are scattered around car edge | 
| SIFT  | 1386 | Good |

### MP.8 Performance Evaluation 2

Number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors can be found in [performance.csv](performance.csv) file.

TOP3 detector / descriptor combinations for detecting most matches.

| Detector-Descriptor combination | Number of matches |
| ------------- | ------------- |
| BRISK-SIFT  | 1739 |
| BRISK-BRIEF  | 1704 |
| BRISK-BRISK  | 1570 |

### MP.9 Performance Evaluation 3

TOP3 detector / descriptor combinations for shortest execution time.

| Detector-Descriptor combination | Total execution time of keypoint detection and descriptor calculation |
| ------------- | ------------- |
| FAST-BRIEF  | 1.54ms |
| FAST-ORB  | 3.8ms |
| ORB-BRIEF  | 6.3ms |