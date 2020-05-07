import pybullet as p
import cv2
import time
import rospy
import sys
import os 
from os import path
import argparse
from geometry_msgs.msg import Pose 
from pose_detection import Pose_From_Image
import env
import numpy as np
import workspace

sys.path.insert( 0 , '{}/cairo_simulator/src/'.format( os.path.dirname( os.path.abspath( __file__ ) ) ) )
from cairo_simulator.Simulator import Simulator, SimObject, ASSETS_PATH
from cairo_simulator.Manipulators import Sawyer



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




def main( args ):    
    
    # Init Cairo Simulator 
    rospy.init_node("CAIRO_Sawyer_Simulator")
    table_origin = [0.6,0,0.5]
    c_sim = Simulator() # Initialize the Simulator
    table = SimObject("Table", ASSETS_PATH + 'table.sdf', (table_origin[0], table_origin[1], 0), (0, 0, 1.5708)) # Table rotated 90deg along z-axis
    sawyer_robot = Sawyer("sawyer0", 0, 0, 0.8)
    
    # Add marker at center of the table, will serve as the origin to the workspace
    add_marker(tuple(table_origin))
    
    calibrate_bottom = cv2.VideoCapture( "./media/current/calibrate_bottom1.MOV" )
    _ , calib_bottom_frame = calibrate_bottom.read()
    
    calibrate_top = cv2.VideoCapture( "./media/current/calibrate_top1.MOV" )
    _ , calib_top_frame = calibrate_top.read()
    
    
    #Open video.    
    vidcap = cv2.VideoCapture( args.media[0] )
    namedWin = cv2.namedWindow("Webcam", cv2.WINDOW_AUTOSIZE )
    success, frame = vidcap.read()
    
    # Init Workspace Class
    W = workspace.Workspace( calib_bottom_frame , [0.6,0,0.5] )
    frame = W.resize_image( frame )
    c = W.get_sim_corners()
    W.calibrate_top( calib_top_frame )

    #Add Corners of measured workspace
    for i in c:
        add_marker(tuple(i))
    
    #Check if video is opened
    if( not vidcap.isOpened() ):
        print("ERROR, video not opened")
        return
    
    font                   = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (10,500)
    fontScale              = 1
    fontColor              = (50,205,50)
    lineType               = 2
    i = 1
    
    
    input("Press Enter to continue...")
    time.sleep(3)
    
    
    while(True):    
        success , frame = vidcap.read() #Grab frame
        
        if( (not success) or ( frame is None)): #If video has ended
            break
        
        frame = W.resize_image( frame ) #Reduce size of image. 
        watch_center = W.track_watch( frame ) #Track center of watch
        
        
        
        if( watch_center is not None and frame is not None):
            watch_center = np.array( [ watch_center[0] , watch_center[1] , .64 + .6*W.watch_height(frame) ] )
            frame , quaternion = W.watch_orientation( frame ) 
            joint_config = sawyer_robot.solve_inverse_kinematics( watch_center, tuple( [0,0.707,0.707,0.] ) )
            sawyer_robot.move_to_joint_pos(joint_config)
            #print("Watch Center: {}".format(watch_center))
            if( i % 10 == 0):
                add_marker( watch_center )
            cv2.putText(frame,'Frame: {}'.format(i),  bottomLeftCornerOfText,  font,  fontScale, fontColor, lineType)
            cv2.imshow("Webcam" , frame )
            i += 1
            
            
            if( cv2.waitKey(30) >= 0):
                break
 
 
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Teleoperation via camera with Cairosim and openpose')
    parser.add_argument('--media' , nargs=1 , default=["./media/current/move_bottom_TEST.MOV"] , help='Video location. Types acceptedL ( .mov , .mp4 , more stuff )')
    
    args = parser.parse_args()
    
    if( not path.exists( args.media[0] ) ):
            print( "{} does not exist. ".format(args.media[0] ) )
            raise Exception("{} not found".format(args.media[0]))

    main( args )





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
