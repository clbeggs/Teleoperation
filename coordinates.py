import env
import numpy as np
import cv2


class Coord():
    
    
    def __init__( self ):
        self.environ = env.Env()
        self.ws_dimesions = None
        self.current_frame = None
        self.current_frame_path = None
    
    def ws_to_sim( self , coordinates ):
        """ 
            Convert given coordinate in cm to pose in cairo sim
            Sim Table dim: 1.5m x 0.8m
            Ratio: 1.5/0.8 = 1.875
        """
        self.ws_dimensions , _ = self.environ.measure_workspace( self.current_frame_path )
        x_size = ( self.ws_dimensions[0] + self.ws_dimensions[2] ) / 200
        y_size = ( self.ws_dimensions[1] + self.ws_dimensions[3] ) / 200
        
        

    def grab_image( self , img_path ):
        self.current_frame = cv2.imread( "{}".format(img_path) , img_path)
        self.current_frame_path = img_path
        
        
    def end_effector_workspace_coords( self , end_eff_coords ):
        """ 
            From eagle eye camera, get xy coordinates 
            Given end_eff_coords, which is xy coordinate in pixels
        """    
        pass
    
    
    def end_effector_height( self ):
        """ Get distance from workspace to end effector, will be z coordinate. """
        pass