## 	Abstract

The goal of this project is to provide a user friendly way to teleoperate a robot arm with only 2 cameras.
Common teleoperation methods require the user to interface with some sort of controller or kinesthetically move the robot arm which can be impractical and require some sort of learning on the users part. Using OpenPose, Aruco tags, and Cairo simulator my goal is to translate raw RGB images to accurate configurations of a robot arm, allow users to naturally control and teleoperate robots to get more accurate representations of intents and goals. 


## Equipment

- OpenPose
- Cairo Simulator
- Aruco Tags

## Deliverables and Implementation Plan


-  Calibrate camera and define workspace - Deadline: 04/17
   -  Calibrate Iphone7+ and Iphone X camera with chessboard printouts and opencv - Deadline: 04/17
   -  Using Arucotags, define workspace. - Deadline: 04/17 
   -  Using Arucotags, measure workspace dimensions - Deadline: 04/17

-  Map 2D points on 2 images to 3D point in simulator
   -  Get 3D Human Pose Estimate - Deadline: 04/17  
   -  Get $xy$ pose of hand via eagle eye camera - Deadline: 04/19
   -  Get $z$(distance from workspace to hand) via side camera. - Deadline: 04/19 
   -  Use Aruco tag measurements and user inputted arm dimesions to help this calibration - Deadline: 04/19

-  Using Inverse Kinematics and path planning, make smooth transitions from coordinate to coordinate - Deadline: 04/22



**Maybe**
- [ ] Once teleoperation is at a point I like, use Inverse RL to train model on some household tasks (Pouring,chopping)

## Demo

To demonstrate this project, I'll have 3 side by side videos. 
One is the raw frames of me moving, one of the openpose representation, then one of sawyer moving in Cairo simulator.
After a small demonstration I'll go into error,and how it compares to other teleoperation methods. 