## Abstract
This project is a way to teleoperate a Sawyer Robot arm with a single birds eye view camera and Aruco Tags.
I was able to successfully translate the workspace and end effector to points in cairo sim and get the sawyer robot arm to follow a tag taped to a wrist watch, I wasn't able to get orientation of the watch however. 
Dissecting the birds eye view video to single frames, I was able to get coordinates in pixels, and translate them to coordinates in the simulator using linear algebra and trig. 



## Introduction and Background
Modern methods of teleoperation usually include a controller of some kind, using VR headsets and controllers, or very specialized surgery controllers, etc. These controllers are require training/getting used to, and have some sort of learning curve. I wanted to introduce a more intuitive and cheaper method of teleoperation. I propose a method of teleoperation using a single bird's eye camera view to operate in a small contained workspace. Using OpenCV and Aruco tags, I am able to semi accuratley measure the workspace and track an end effector. Granted, I wouldn't want a surgeon to operate on anything more than a grape with this system, but for simpler things it'll do. 


## Methods
`workspace.py` - Measures workspace defined by Aruco Tags, Tracks Watch with Aruco Tag
`detect_aruco.py` - Detects aruco tags, estimates pose of Tags
`main.py` - Main project driver, Instatiates simulator and workspace classes, opens videos and feeds them into the workspace class. 
`cairo_simulator/` - sawyer arm simulator
`Helpers/camera_calib.py` - one time camera calibration, just grabs distortion coefficients and camera matrix
`media/` - test videos


## Results/Discussion
I was able to pretty accuratley get xyz coordinates from video, and translate that to the simulator, and use IK to make the sawyer arm move to those coordinates.
I was unable to get orientation of the end effector however. 

