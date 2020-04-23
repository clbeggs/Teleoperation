import pybullet as p
import cv2
import time
import rospy
import sys
import os 
import argparse
from cairo_simulator import Simulator, SimObject, ASSETS_PATH
from cairo_simulator import Sawyer
from geometry_msgs.msg import Pose 
from pose_detection import Pose_From_Image
import env
import numpy as np
import workspace

def add_marker( pose ):
    """ 
        Add small_sphere marker, no mass and no collision.
        marker_sphere.urdf 
    """
    _ = SimObject( "marker" , "marker_sphere.urdf" , pose )
    
def add_object( obj_file , obj_name="obj" , pos_vec=None , orient_vec=None ):
    """ 
        Add object to sim env. 
    """
    if( orient_vec == () ):
        orient_vec = None
    if( pos_vec == () ):
       pos_vec = None
             
    _ = SimObject( obj_name, obj_file, pos_vec , orient_vec )

def populate_sim( environ , origin ):
    z = environ.define_sim_ws(np.array(origin))
    for i in z:
        add_marker(i)
    for i in z:
        print(i)

def main():
    ### Environment 
    rospy.init_node("CAIRO_Sawyer_Simulator")
    table_origin = [0.6,0,0.5]
    c_sim = Simulator() # Initialize the Simulator
    table = SimObject("Table", ASSETS_PATH + 'table.sdf', (table_origin[0], table_origin[1], 0), (0, 0, 1.5708)) # Table rotated 90deg along z-axis
    sawyer_robot = Sawyer("sawyer0", 0, 0, 0.8)
    add_marker(tuple(table_origin))
    
    
    
    vidcap = cv2.VideoCapture( "./media/ee_new.MOV" )
    namedWin = cv2.namedWindow("Webcam", cv2.WINDOW_AUTOSIZE )
    success, frame = vidcap.read()
    
    
    W = workspace.Workspace( frame , [0.6,0,0.5] )
    frame = W.resize_image( frame )
    c = W.get_sim_corners()
    
    #add_marker( ( c[0][0] , c[0][1] , c[0][2] + 0.1))
    #add_marker( ( c[1][0] , c[1][1] , c[1][2] + 0.2))
    #add_marker( ( c[2][0] , c[2][1] , c[2][2] + 0.3))
    #add_marker( ( c[3][0] , c[3][1] , c[3][2] + 0.4))
    
    for i in c:
        add_marker(tuple(i))
    print("Corners ")

    for i in c:
        print(i)
        
    v = W.track_watch( frame )    
    #add_marker(v)
    
    if( not vidcap.isOpened() ):
        print("ERROR")
        return
    
    font                   = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (10,500)
    fontScale              = 1
    fontColor              = (50,205,50)
    lineType               = 2
    i = 1
    input("Press Enter to continue...")
    while(True):    
        success , frame = vidcap.read()
        frame = W.resize_image( frame )
        if( success == False ):
            break
        v = W.track_watch( frame )
        
        
        if( v is not None ):
            print("COORDS: {}".format(v))
            #joint_config = sawyer_robot.solve_inverse_kinematics(v, [0,0,0,1])
            #sawyer_robot.move_to_joint_pos(joint_config)
            if( i % 10 == 0):
                add_marker( v )
            cv2.putText(frame,'Frame: {}'.format(i), 
            bottomLeftCornerOfText, 
            font, 
            fontScale,
            fontColor,
            lineType)
            cv2.imshow("Webcam" , frame )
            i += 1
            
            
            if( cv2.waitKey(30) >= 0):
                break
 
 
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Teleoperation via camera with Cairosim and openpose')
    parser.add_argument('--media' , nargs=1 , default=[] , help='Video or image location')
    
    args = parser.parse_args()
    
    #if( args.media == [] ):
      #  print("No media passed in, use \"--media\" to point to location of video" )

    main()





""" 
Datum Class Attributes:

['__class__', '__delattr__', '__dir__', '__doc__', '__eq__', 
'__format__', '__ge__', '__getattribute__', '__gt__', '__hash__',
 '__init__', '__init_subclass__', '__le__', '__lt__', '__module__',
  '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', 
  '__setattr__', '__sizeof__', '__str__', '__subclasshook__', 
  'cameraExtrinsics', 'cameraIntrinsics', 'cameraMatrix', 'cvInputData', 
  'cvOutputData', 'cvOutputData3D', 'elementRendered', 'faceHeatMaps',
   'faceKeypoints', 'faceKeypoints3D', 'faceRectangles', 'frameNumber', 
   'handHeatMaps', 'handKeypoints', 'handKeypoints3D', 'handRectangles', 
   'id', 'inputNetData', 'name', 'netInputSizes', 'netOutputSize', 
   'outputData', 'poseCandidates', 'poseHeatMaps', 'poseIds', 'poseKeypoints',
    'poseKeypoints3D', 'poseNetOutput', 'poseScores', 'scaleInputToNetInputs', 
    'scaleInputToOutput', 'scaleNetToOutput', 'subId', 'subIdMax']



"""