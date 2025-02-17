import numpy as np
import cv2
import detect_aruco
from sympy import symbols, Eq, solve
import math


def vector_between_points( x , y ):
    """ Vectors from x -> y """
    return np.array( [ y[0] - x[0] , y[1] - x[1] ] )

def get_param_mtx( x , y ):
    """ 
        Get parametrization of line through points x , y starting at x
        then convert to 2x2 matrix.
    """
    v = vector_between_points( x , y )
    return [x,v]

def euclid_distance( x , y ):
    """ Euclidiean Distance from x -> y"""
    return math.sqrt( math.pow((y[0] - x[0]), 2) + math.pow((y[1] - x[1]),2) )

def dot_prod( x , y ):
    """ Dot product of two vectors in R^2 """
    return x[0]*y[0] + x[1]*y[1]

def norm( x ):
    return math.sqrt( math.pow( x[0] , 2 ) + math.pow( x[1] , 2 ) )

def projection( a , b ):
    """ Vector projection of a onto b """
    coeff = dot_prod( a , b ) / math.pow( norm(b) , 2 )
    return [ b[0] * coeff , b[1] * coeff ]
    
def cross_ratio( A , B , C , D , tag_size ):
    dist_A_to_B = euclid_distance( A , B ) #Size of tag which is known to be tag_size meters
    dist_C_to_D = euclid_distance( C , D ) #Size of tag which is known to be tag_size meters
    dist_B_to_C = euclid_distance( B , C ) #Distance between tags
    
    cross_ratio = (dist_B_to_C + dist_A_to_B) * ( dist_B_to_C + dist_C_to_D )
    cross_ratio = cross_ratio / ( (dist_B_to_C)*( dist_B_to_C + dist_A_to_B + dist_C_to_D ) )
    
    W = symbols('W')
    eq1 = Eq( ( ( tag_size + W )*( tag_size + W ) ) / ( W * ( W + tag_size + tag_size ) ) , cross_ratio )
    return max( solve(eq1,W) ) #max because polynomial W^2

def print_ws( A , B , C , D ):
    print("\n=============================Workspace Dimensions=============================\n")
    print("                                 {0:10f}cm                             ".format(100*float(A)))
    print("           ______________________________________________________     ")
    print("           |[1]                    [2]                       [3]|     ")
    print("           |                                                    |     ")
    print("           |                                                    |     ")
    print("{0:5f}cm|                                                    |{0:5f}cm".format(100*float(B) ,100*float(C) ))
    print("           |                                                    |     ")
    print("           |                                                    |     ")
    print("           |                                                    |     ")
    print("           |[4]______________________________________________[5]|     ")
    print("                                 {0:5f}cm                             ".format(100*float(D)))
    print("\n=================================================================================\n\n")

class Workspace():
    
    def __init__( self , workspace_image , sim_origin , table_dim=( 1.5 , 0.8 ) ):
        self.aruco = detect_aruco.Detect_Aruco()
        self.ARUCO_TAG_SIZE = .05 #m
        self.sim_origin = sim_origin #Center of table in simulator: ( x , y , z )
        self.ws_origin = None #Center of workspace in cm
        self.px_origin = None #Center of workspace in px
        self.tag_corners = dict()
        self.tag_centers = dict()
        self.corners = None
        self.sim_corners = None
        self.sim_dimensions = table_dim
        self.curr_image = None
        self.bottom_image = None
        self.top_image = None
        self.bottom_end_effector_corners = None
        #Measure workspace from photo
        self.measure_workspace( workspace_image ) 
        
        #Get distance to each corner
        self.get_corners()
        #Use corners to place workspace into simulator
        self.create_workspace()
    
    def measure_workspace( self , image_or_path ):
        """ 
        Given image or cv2 image, measure workspace dimensions
        Origin is center of workspace.
        """
        self.bottom_image = image_or_path
        # Get Aruco Tags
        corners , ids , image , _ = self.aruco.detect_tags( image_or_path )
        
        #Save corners and center of each tag
        for i in range( 1 , 6 ):
            id_indx = list(ids).index( [i] )
            self.tag_corners[i] = corners[id_indx][0] #Store tag corners
            self.tag_centers[i] = [ (self.tag_corners[i][0] + self.tag_corners[i][2]) / 2 , (self.tag_corners[i][1] + self.tag_corners[i][3]) / 2 ]  #Get tag center
        #Get origin in pixels using midpoint formula
        self.px_origin = [ ( (self.tag_corners[1][2][0] + self.tag_corners[5][0][0])/2 + (self.tag_corners[3][3][0] + self.tag_corners[4][1][0])/2 ) / 2 , ( (self.tag_corners[1][2][1] + self.tag_corners[5][0][1])/2 + (self.tag_corners[3][3][1] + self.tag_corners[4][1][1])/2 ) / 2 ]
        self.px_origin = [ int(self.px_origin[0]) , int(self.px_origin[1]) ] 
        
        if( [28] in ids ):
            id_indx = list(ids).index([28])
            self.bottom_end_effector_corners = corners[id_indx][0]
            
        
        
        #Get size of workspace with homography, specifically cross ratio, for each side.
        one_to_three_m  = cross_ratio( self.tag_corners[1][0] , self.tag_corners[1][1] , self.tag_corners[3][0] , self.tag_corners[3][1] , self.ARUCO_TAG_SIZE )
        four_to_five_m  = cross_ratio( self.tag_corners[4][3] , self.tag_corners[4][2] , self.tag_corners[5][3] , self.tag_corners[5][2] , self.ARUCO_TAG_SIZE )
        one_to_four_m   = cross_ratio( self.tag_corners[1][1] , self.tag_corners[1][2] , self.tag_corners[4][1] , self.tag_corners[4][2] , self.ARUCO_TAG_SIZE )
        three_to_five_m = cross_ratio( self.tag_corners[3][0] , self.tag_corners[3][3] , self.tag_corners[5][0] , self.tag_corners[5][3] , self.ARUCO_TAG_SIZE )
        
        # Dimensions go: [ 1->3 , 3->5 , 4->5 , 1->4 ]
        self.dimensions = [ one_to_three_m , three_to_five_m , four_to_five_m , one_to_four_m ]
        
        print_ws(one_to_three_m , one_to_four_m ,three_to_five_m , four_to_five_m)#Print!
        
        #Draw Lines on image
        cv2.line( image , tuple(self.tag_corners[1][1]) , tuple(self.tag_corners[3][0]), (0,255,0) , 2 )
        cv2.line( image , tuple(self.tag_corners[4][2]) , tuple(self.tag_corners[5][3]), (0,255,0) , 2 )
        cv2.line( image , tuple(self.tag_corners[1][2]) , tuple(self.tag_corners[4][1]), (0,255,0) , 2 )
        cv2.line( image , tuple(self.tag_corners[3][3]) , tuple(self.tag_corners[5][0]), (0,255,0) , 2 )
        cv2.circle( image , tuple(self.px_origin) , 2 , (255,255,0) , 5 )
        
        self.curr_image = image
        
    def get_corners( self ):
        """ 
            Get distance from center of workspace to each corner, to assist transformation to the simulator 
        """
        # Using trig -> Dimensions go: [ 1->3 , 3->5 , 4->5 , 1->4 ]
        top_left = math.sqrt( math.pow( self.dimensions[0] / 2 , 2 ) + math.pow( self.dimensions[3] / 2 , 2 ) ) 
        top_right = math.sqrt( math.pow( self.dimensions[0] / 2 , 2 ) + math.pow( self.dimensions[1] / 2 , 2 ) ) 
        bottom_right = math.sqrt( math.pow( self.dimensions[3] / 2 , 2 ) + math.pow( self.dimensions[1] / 2 , 2 ) ) 
        bottom_left = math.sqrt( math.pow( self.dimensions[3] / 2 , 2 ) + math.pow( self.dimensions[2] / 2 , 2 ) ) 
        #self.corners = [ topleft , topright , bottomright , bottomleft  ]
        self.corners = [ top_left , top_right , bottom_right , bottom_left ]

    def create_workspace( self ):
        """ 
            Using self.corners and the origin of the table, return array that has the coordinates for the corners of the 
            workspace in the sim
        """ 
        
        top_left = ( np.array( [ 1 , 1 , 0 ] ) * self.corners[0] ) + self.sim_origin
        top_right = ( np.array( [ 1 , -1 , 0 ] ) * self.corners[1] ) + self.sim_origin
        bottom_right = ( np.array( [ -1 , -1 , 0 ] ) * self.corners[2] ) + self.sim_origin
        bottom_left = ( np.array( [ -1 , 1 , 0 ] ) * self.corners[3] ) + self.sim_origin
        
        self.sim_corners = [ top_left , top_right , bottom_right , bottom_left ]

    def get_sim_corners( self ):
        return np.array(self.sim_corners)
    
    def track_watch( self , image_or_path , end_effector_id=28 ):
        """ 
            From an image, find the watch in pixels and return coordinate in sim
        """
        
        # Get Aruco Tags
        corners , ids , image , _ = self.aruco.detect_tags( image_or_path )
        
        if( [end_effector_id] not in ids ):
            print("Watch tag not found in image")
            return None
        
        #Get watch ID and position in image. 
        watch_index = list(ids).index([end_effector_id])
        watch_corners = corners[watch_index][0]
        watch_center = np.array( [ int( ( watch_corners[0][0] + watch_corners[2][0] ) / 2 ) , int( ( watch_corners[0][1] + watch_corners[2][1] ) / 2 ) ] ) 
        
        #Get vector from origin to our watch
        vector = vector_between_points( self.px_origin , watch_center )
        vector *= (-1)
        #Now we project this vector on two of the four sides of the workspace to measure it.
        # Dimensions go: [ 1->3 , 3->5 , 4->5 , 1->4 ]
        if( vector[1] < 0 ): #If x compenent is negative, project onto line connecting tags 1->4
            x_comp = projection( vector , vector_between_points( self.tag_corners[3][3]  , self.tag_corners[5][0] ) ) 
            x_comp_m = self.dimensions[1] / 2 
            x_comp_px = (-1/2)*norm( vector_between_points( self.tag_corners[3][3]  , self.tag_corners[5][0] ) )
        else: 
            x_comp = projection( vector , vector_between_points( self.tag_corners[1][2]  , self.tag_corners[4][1] ) ) 
            x_comp_m = self.dimensions[3] / 2 
            x_comp_px = (1/2)*norm( vector_between_points( self.tag_corners[1][2]  , self.tag_corners[4][1] ) ) 
            
        if( vector[0] < 0 ): #If y component is negative, projecto onto line connecting tags 4->5
            y_comp = projection( vector , vector_between_points( self.tag_corners[4][2]  , self.tag_corners[5][3] ) ) 
            y_comp_m = self.dimensions[2] / 2
            y_comp_px = (-1/2)*norm( vector_between_points( self.tag_corners[4][2]  , self.tag_corners[5][3] ) )
        else:
            y_comp = projection( vector , vector_between_points( self.tag_corners[1][1]  , self.tag_corners[3][0] ) )
            y_comp_m = self.dimensions[0] / 2
            y_comp_px = (1/2)*norm( vector_between_points( self.tag_corners[1][1]  , self.tag_corners[3][0] ) )
        
        fin = np.array( [(norm(x_comp)) * (x_comp_m/x_comp_px ) , (norm(y_comp)) * (y_comp_m / y_comp_px)] )
        fin = self.workspace_to_sim_transform( fin )
        fin = np.array( [ fin[0] + self.sim_origin[0] , fin[1] + self.sim_origin[1] , .2 + self.sim_origin[2] ])
        return fin
    
    def watch_orientation( self , image_or_path , end_effector_id=28 ):
        """
        Here, you transform 3D points expressed in the tag frame into the camera frame:
        https://answers.opencv.org/question/215377/aruco-orientation-using-the-function-arucoestimateposesinglemarkers/
        
        The camera pose with respect to a marke r is the 3d transformation from the marker coordinate system to the 
        camera coordinate system. It is specified by rotation and translation vectors 
        
        
        [ 0 , 0.707 , 0.707 , 0 ] is the correct quaternion for the entire demo video. right to left
        [ 0 , 0.707 , 0 , 0.707 ] is direction from closest point of table to end of table
            
        """
        if( image_or_path is None ):
            return None , None , None
        
        else:
            im , q , ids = self.aruco.draw_cube( image_or_path )
            for i in range( len( q ) ):
                if( ids[i] == [end_effector_id] ):
                    return im , q[i]
        return None , None , None 
    
    def calibrate_top( self , image_or_path ):
        """
            Grab info from image at top of workspace. 
            
        """ 
        self.top_image = image_or_path 
        corners , top_ids , im_test , _ = self.aruco.detect_tags( self.top_image ) #Top image
        
        self.top_corners = None
        
        
        if( [28] in top_ids ):
            id_indx = list(top_ids).index( [28] )
            self.top_corners = corners[id_indx][0]
        
    def watch_height( self , image_or_path , camera_height=1 , end_effector_id=28 ):
        """ 
            Get height of the aruco tag.
            
            Methods:
                - Just need height of camera at recording.
                ROUGHLY 0.96 meters
        """
        
        corners , ids , image , _ = self.aruco.detect_tags( image_or_path ) #Top image
        
        curr_corners = dict()
        
        for i in range( 1 , 6 ):
            if( [i] in ids ):
                id_indx = list(ids).index( [i] )
                curr_corners[i] = corners[id_indx][0][0] #Store tag corners
                
        curr_end_corners = None
        if( [28] in ids ):
            id_indx = list(ids).index( [28] )
            curr_end_corners = corners[id_indx][0]
            
        #For now we will just measure the top left corner as a rough estimate. Pretty Hardcoded. This can be improved in the future.
        # [ TopLeft -> TopRight , TopRight -> BottomRight , BottomRight -> BottomLeft , BottomLeft -> TopLeft ]
        bottom , top , current = [] , [] , []
        for i in range(3):
            if( len(curr_end_corners) > 0  ):
                bottom.append( euclid_distance( self.bottom_end_effector_corners[i] , self.bottom_end_effector_corners[i+1] ) )
                top.append( euclid_distance( self.top_corners[i] , self.top_corners[i+1] ) )
                current.append( euclid_distance( curr_end_corners[i] , curr_end_corners[i+1] ) )

        
        # Square length size of 300px - bottom - Corresponds to .06 in sim
        # Square length size of 700px - top - Corresponds to ( .06 + 1 ) in sim 
        # Now we get a square length size of 450px, how do we get height?
        # 700 - 300 = 400 , 300 - 300 = 0 , 450 - 300 = 150.
        # 150 is 37.5 percent of 300. So we know that pixel size of 450 corresponds to 37.5 percent distance from bottom to top! Bam. 
        # I'm sure there are better and more accurate ways to do this but I have 4 final tests I have to study for. 
        ratio = []
        
        for i in range(len(current)):
            ratio.append( (current[i] - bottom[i]) / top[i] )
        return np.average(ratio) 
        
        
        
        
        
    def resize_image( self , image ):
        return self.aruco.image_resize( image , 1200 , 900 )
    
    def workspace_to_sim_transform( self , coordinate ):
        """ 
            Coordinate is (x,y) coordinate in measured workspace 
            Return (x,y) coordinate in simulator 
            JUST SCALING, do not add on table_origin
            X: red, Y: green, Z: blue.
            
        """
        # Dimensions go: [ 1->3 , 3->5 , 4->5 , 1->4 ]
        
        x_ratio = ( coordinate[0] / self.dimensions[0] )
        y_ratio = ( coordinate[1] / self.dimensions[1] )
        if( len( coordinate) == 3 ):
            return np.array([coordinate[0] , self.sim_dimensions[1] * y_ratio , coordinate[2]] ) 
        else:
            return np.array([coordinate[0] , self.sim_dimensions[1] * y_ratio] ) 
    
    def pixel_to_worspace_transform( self , coordinate ):
        """ 
            Given pixel coordinate (x,y) translate to meters 
            in workspace.
        """
        pass
    
    

































