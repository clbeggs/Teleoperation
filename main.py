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
        print(i)
        add_marker(i)

def main2(args):
    """ Init. CairoSim Node """
    rospy.init_node("CAIRO_Sawyer_Simulator")
   
    #
    #
    origin = [0.6,0,0.5]
    """ Create simulator and populate with objects """
    c_sim = Simulator() # Initialize the Simulator
    table = SimObject("Table", ASSETS_PATH + 'table.sdf', (origin[0], origin[1], 0), (0, 0, 1.5708)) # Table rotated 90deg along z-axis
    sawyer_robot = Sawyer("sawyer0", 0, 0, 0.8)
    add_marker(tuple(origin))
    environ = env.Env()
    
    #t = lambda x : tuple(x) if(isinstance(x,list)) else x #Convert input to tuple if list. 

    s = Pose_From_Image()
    kp_list = s.get_body_pose_mapping()
    r = s.image("./media/per2.jpg")
    kp = r.poseKeypoints #each keypoint is (x , y  , confidence score )
    img = r.cvOutputData
    print(kp)
    for i , body_part in enumerate(kp[0]):
        print("{} - {}\n".format(kp_list[i] , body_part))
        # print("({},{})".format(body_part[i] , body_part[1]))
    print(type(img))
    cv2.imwrite("./haha.jpg" , img)
    
    populate_sim(environ,origin)
    joint_angs = np.array([0.4493345488433066, -0.15264013369292084, 0.5246021257607938, -0.622124537542545, -0.29306412321553604, -1.93889201589517, -0.15973077175720515, -2.2767884734182654e-08, -0.02083300000000173])
    time.sleep(10)
    sawyer_robot.move_to_joint_pos(joint_angs)
    
    while rospy.is_shutdown() is not True:
        c_sim.step()
        a= sawyer_robot.get_current_joint_states()
        rospy.sleep(0.8)
    p.disconnect()


def main():
    rospy.init_node("CAIRO_Sawyer_Simulator")
    table_origin = [0.6,0,0.5]
    c_sim = Simulator() # Initialize the Simulator
    table = SimObject("Table", ASSETS_PATH + 'table.sdf', (table_origin[0], table_origin[1], 0), (0, 0, 1.5708)) # Table rotated 90deg along z-axis
    sawyer_robot = Sawyer("sawyer0", 0, 0, 0.8)
    add_marker(tuple(table_origin))
    
    ### Eagle Eye Environment ###
    eagle_eye = env.Env()
    dim_ee , img_ee = eagle_eye.measure_workspace("./media/match_env_pers/eagle_eye.jpg")
    
    ### Environment @ an angle ###
    perspective = env.Env()
    dim_p , img_p = perspective.measure_workspace("./media/match_env_pers/perspective.jpg")
    
    ### Save both environment pictures ###
    cv2.imwrite("./media/match_env_pers/perspective_ar.jpg" , img_p )
    cv2.imwrite("./media/match_env_pers/perspective_ee.jpg",img_ee)
    
    ### Get pose of person in perspective photo ###
    openP = Pose_From_Image()
    body_mapping = openP.get_body_pose_mapping()
    datum = openP.image("./media/match_env_pers/perspective_ar.jpg")
    cv2.imwrite( "./media/match_env_pers/pose_pers.jpg",  datum.cvOutputData )
    
    populate_sim(eagle_eye,table_origin) #Place environment photos in simulator
    

    for i in range(len(body_mapping.keys())):
        if( body_mapping[i] == "RWrist"):
            mapped_wrist = perspective.px_to_cm( [datum.poseKeypoints[0][i][0] , datum.poseKeypoints[0][i][1]] )
            mapped_wrist = np.array( [ (mapped_wrist[0] + table_origin[0]) /100 , (mapped_wrist[1] + table_origin[1]) /100 , table_origin[2] + 0.06]  ) 
            break
    print("WRIST {} and TABLE ORIGIN {} =====================================".format(mapped_wrist , table_origin))

    add_marker(mapped_wrist)
    
    time.sleep(10)
    p.disconnect()
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Teleoperation via camera with Cairosim and openpose')
    parser.add_argument('--media' , nargs=1 , default=[] , help='Video or image location')
    
    args = parser.parse_args()
    
    #if( args.media == [] ):
      #  print("No media passed in, use \"--media\" to point to location of video" )
    main()
    #test2()
    #eagle()





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