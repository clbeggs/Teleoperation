import numpy as np
import cv2
import detect_aruco as da
from sympy import symbols, Eq, solve
import math


def vector_between_points( x , y ):
    """ Vectors from x -> y """
    return [ y[0] - x[0] , y[1] - x[1] ]

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
    




class Env():
    
    def __init__( self ):
        self.aruco = da.Detect_Aruco()
        self.ARUCO_TAG_SIZE = 10 #cm
        self.dimensions = None
        self.aruco_tag_loc = dict()
        self.aruco_tag_centers = dict()
        self.unit_vector_x = None
        self.unit_vector_y = None
        
        for i in range(1,6):
            self.aruco_tag_loc[i] = None
            self.aruco_tag_centers[i] = None
        self.origin = None
        self.sim_origin = None
    
    def measure_workspace( self , image_path ):
        """ 
        Get dimensions of workspace from 5 Aruco Tags 
        There will be one Aruco Tag in each corner, and one at the center top of the
        workspace so we can orient. Each tag will have a specific ID.    
        Each tag will also have a specific size, for now we will default to 10cm    
        ______________________________________________________
        |[1]                    [2] +x->                  [3]|
        |                        -y                          |
        |                        |                           |
        |                       \ /                          |
        |                                                    |
        |                                                    |
        |                                                    |
        |[4]______________________________________________[5]|
        
        For this problem, we will use projective geometry and homography to be able to convert pixel distances to actual measurements from an image.
        https://en.wikipedia.org/wiki/Homography
        """
        #Grab corners and id's of all 5 aruco tags
        corners , ids , image , image_cpy = self.aruco.detect_tags( image_path )

        #Put corresponding corners into aruco_tag_loc
       
    
        for i in range( 1 , 6 ):
            id_indx = np.where(ids == i) # ids.index(i)
            id_indx = int(id_indx[0])
            self.aruco_tag_loc[i] = corners[id_indx]
            
        #Get centers of tags using midpoint formula
        for i in range( 1 , 6 ):
            center = [ (self.aruco_tag_loc[i][0][0][0] + self.aruco_tag_loc[i][0][2][0]) / 2 , (self.aruco_tag_loc[i][0][0][1] + self.aruco_tag_loc[i][0][2][1]) / 2 ]
            self.aruco_tag_centers[i] = [ int(center[0]) , int(center[1])]
            cv2.circle( image , ( self.aruco_tag_centers[i][0] , self.aruco_tag_centers[i][1]) , 3, (0,0,255) , 5 )
        
        self.origin = self.aruco_tag_centers[2]
        self.unit_vector_x = vector_between_points( self.origin , self.aruco_tag_centers[3] )
        
        tmp = vector_between_points( self.origin , self.aruco_tag_centers[4] ) 
        tmp2 = vector_between_points( self.aruco_tag_centers[1] , self.origin )
        self.unit_vector_y = [tmp[0] + tmp2[0] , tmp[1] + tmp2[1]]
        
        del tmp
        del tmp2
            
        """ 
        Cross Ratio: 
        (1-3)_topleft * (1-3)_topright / closedist_between * long_distbetween
        """
        euclid_dist_1 = euclid_distance(self.aruco_tag_loc[1][0][0],self.aruco_tag_loc[1][0][1])#Size of tag 1
        euclid_dist_3 = euclid_distance(self.aruco_tag_loc[3][0][0],self.aruco_tag_loc[3][0][1])#Size of tag 3
        euclid_dist_1_to_3 = euclid_distance(self.aruco_tag_loc[1][0][1] , self.aruco_tag_loc[3][0][0]  ) #distance from top right of 1 to top left of 3
        cross_ratio_pixel = (euclid_dist_1 + euclid_dist_1_to_3) * (euclid_dist_3+euclid_dist_1_to_3) #numerator
        cross_ratio_pixel = cross_ratio_pixel / ((euclid_dist_1_to_3) * (euclid_dist_1_to_3 + euclid_dist_1 + euclid_dist_3 )) #Denominator
        W = symbols('W')
        eq1 = Eq( ((5+W)*(5+W)) / (W*(W+5+5)) , cross_ratio_pixel)
        dist_cm_1_3 = max(solve(eq1,W)) #In cm, max because there it is sqrt, and sometimes are negative values
        
        #Now let's do it for 4->5
        euclid_dist_4 = euclid_distance(self.aruco_tag_loc[4][0][0],self.aruco_tag_loc[4][0][1])#Size of tag 4
        euclid_dist_5 = euclid_distance(self.aruco_tag_loc[5][0][0],self.aruco_tag_loc[5][0][1])#Size of tag 5
        euclid_dist_4_to_5 = euclid_distance(self.aruco_tag_loc[4][0][1] , self.aruco_tag_loc[5][0][0]  ) #distance from top right of 4 to top left of 5
        cross_ratio_pixel = (euclid_dist_4 + euclid_dist_4_to_5) * (euclid_dist_5+euclid_dist_4_to_5) #numerator
        cross_ratio_pixel = cross_ratio_pixel / ((euclid_dist_4_to_5) * (euclid_dist_4_to_5 + euclid_dist_4 + euclid_dist_5 )) #Denominator
        eq1 = Eq( ((5+W)*(5+W)) / (W*(W+5+5)) , cross_ratio_pixel)
        dist_cm_4_5 = max(solve(eq1,W)) #In cm, max because there it is sqrt, and sometimes are negative values
        
        #Now let's do it for 1->4
        euclid_dist_1 = euclid_distance(self.aruco_tag_loc[1][0][1],self.aruco_tag_loc[1][0][2])#Size of tag 1
        euclid_dist_4 = euclid_distance(self.aruco_tag_loc[4][0][1],self.aruco_tag_loc[4][0][2])#Size of tag 4
        euclid_dist_1_to_4 = euclid_distance(self.aruco_tag_loc[1][0][2] , self.aruco_tag_loc[4][0][1]  ) #distance from top right of 1 to top left of 4
        cross_ratio_pixel = (euclid_dist_1 + euclid_dist_1_to_4) * (euclid_dist_4+euclid_dist_1_to_4) #numerator
        cross_ratio_pixel = cross_ratio_pixel / ((euclid_dist_1_to_4) * (euclid_dist_1_to_4 + euclid_dist_1 + euclid_dist_4 )) #Denominator
        eq1 = Eq( ((5+W)*(5+W)) / (W*(W+5+5)) , cross_ratio_pixel)
        dist_cm_1_4 = max(solve(eq1,W)) #In cm, max because there it is sqrt, and sometimes are negative values
        
        
        #Now let's do it for 3->5
        euclid_dist_3 = euclid_distance(self.aruco_tag_loc[3][0][1],self.aruco_tag_loc[3][0][2])#Size of tag 3
        euclid_dist_5 = euclid_distance(self.aruco_tag_loc[5][0][1],self.aruco_tag_loc[5][0][2])#Size of tag 5
        euclid_dist_3_to_5 = euclid_distance(self.aruco_tag_loc[3][0][3] , self.aruco_tag_loc[5][0][0]  ) #distance from top right of 3 to top left of 5
        cross_ratio_pixel = (euclid_dist_3 + euclid_dist_3_to_5) * (euclid_dist_5+euclid_dist_3_to_5) #numerator
        cross_ratio_pixel = cross_ratio_pixel / ((euclid_dist_3_to_5) * (euclid_dist_3_to_5 + euclid_dist_3 + euclid_dist_5 )) #Denominator
        eq1 = Eq( ((5+W)*(5+W)) / (W*(W+5+5)) , cross_ratio_pixel)
        dist_cm_3_5 = max(solve(eq1,W)) #In cm, max because there it is sqrt, and sometimes are negative values
        
        
        print("\n=============================Workspace Dimensions=============================\n")
        print("                                 {0:10f}cm                             ".format(float(dist_cm_1_3)))
        print("           ______________________________________________________     ")
        print("           |[1]                    [2]                       [3]|     ")
        print("           |                                                    |     ")
        print("           |                                                    |     ")
        print("{0:5f}cm|                                                    |{0:5f}cm".format(float(dist_cm_1_4) ,float(dist_cm_3_5) ))
        print("           |                                                    |     ")
        print("           |                                                    |     ")
        print("           |                                                    |     ")
        print("           |[4]______________________________________________[5]|     ")
        print("                                 {0:5f}cm                             ".format(float(dist_cm_4_5)))
        print("\n=================================================================================\n\n")
        
        
        # Outline workspace on image
        cv2.line( image , tuple(self.aruco_tag_loc[1][0][1]) , tuple(self.aruco_tag_loc[3][0][0]), (0,255,0) , 2 )
        cv2.line( image , tuple(self.aruco_tag_loc[4][0][1]) , tuple(self.aruco_tag_loc[5][0][0]), (0,255,0) , 2 )
        cv2.line( image , tuple(self.aruco_tag_loc[1][0][2]) , tuple(self.aruco_tag_loc[4][0][1]), (0,255,0) , 2 )
        cv2.line( image , tuple(self.aruco_tag_loc[3][0][3]) , tuple(self.aruco_tag_loc[5][0][0]), (0,255,0) , 2 )
        cv2.line( image , tuple(self.aruco_tag_loc[1][0][2]) , tuple(self.aruco_tag_loc[1][0][1]), (0,255,0) , 2 )
        cv2.line( image , tuple(self.aruco_tag_loc[3][0][3]) , tuple(self.aruco_tag_loc[3][0][0]), (0,255,0) , 2 )
        

        #Distance list is [ 1->3 , 3->5 , 5->4 , 1->4], going around square clockwise. 
        self.dimensions = [dist_cm_1_3 , dist_cm_3_5 , dist_cm_4_5 , dist_cm_1_4]
        return self.dimensions , image
    
    def px_to_cm( self , coord ):
        """ TODO change so that the the point is measured from two closest vectors on square """
        v = vector_between_points( self.aruco_tag_loc[1][0][1] , coord )
        
        w = vector_between_points(self.aruco_tag_loc[1][0][1] , self.aruco_tag_loc[3][0][0] )
        z = vector_between_points( self.aruco_tag_loc[1][0][2] , self.aruco_tag_loc[4][0][1] )
        
        u = projection( v , w )
        p = projection( v , z )

        return [norm(u)*(self.dimensions[0]/norm(w)) , norm(p)*(self.dimensions[3]/norm(z)) ]
        
    def define_sim_ws2( self , origin_m ):
        """ 
            Given the center point of the table in the simulator,
            define the measured workspace in terms of coordinates in the simulator
            Coord is origin of table in sim 
        """
        arr = []
        midpt_x = (self.aruco_tag_loc[1][0][1][0] + self.aruco_tag_loc[5][0][0][0] ) + (self.aruco_tag_loc[3][0][0][0]  + self.aruco_tag_loc[4][0][1][0] ) 
        midpt_x /= 4 
        
        midpt_y = (self.aruco_tag_loc[1][0][1][1] + self.aruco_tag_loc[5][0][0][1] ) + (self.aruco_tag_loc[3][0][0][1]  + self.aruco_tag_loc[4][0][1][1] )
        midpt_y /= 4 
        
        origin = [ round(midpt_x) , round(midpt_y) ] #Measured origin in pixels
        
        dist_v = self.px_to_cm( origin ) #Distance from top right of tag 1 to origin in cm
        dist_v = np.array(dist_v) / 100 #Convert to meters
        dist_v[0] = dist_v[0] * (-1) #Since the origin is in the center, this vector is in the 2nd quadrant
        arr.append(dist_v) 
        
        #Projection of vector from top right of tag on to origin onto the vector that connects 1 and 3
        u = projection( origin , vector_between_points(self.aruco_tag_loc[1][0][1], self.aruco_tag_loc[3][0][0]) ) 
        #projection of same origin vector to vector from bottom right of 1 to top right of 4
        v = projection( origin , vector_between_points(self.aruco_tag_loc[1][0][2], self.aruco_tag_loc[4][0][1]) )
        
        #
        p = np.array(u) - vector_between_points(self.aruco_tag_loc[1][0][1], self.aruco_tag_loc[3][0][0])
        l = np.array(v) - vector_between_points(self.aruco_tag_loc[1][0][2], self.aruco_tag_loc[4][0][1])
        
        x = np.array(self.px_to_cm(p)) + np.array(self.px_to_cm(v))
        y = np.array(self.px_to_cm(l)) + np.array(self.px_to_cm(u))

        x = x*(-1/100) 
        y = y / 100
        arr.append( y )
        t = np.array(self.px_to_cm( self.aruco_tag_loc[5][0][0] )) / 200 #Distance from top right of 1 to top left of 5 / 2
        t[1] = t[1]*(-1)
        arr.append( t )
        arr.append( x )
        print("BEFORe============================")
        print(arr)
        print(origin_m)
        for i in range(4):
            arr[i] += np.array(origin_m)
            arr[i] = np.resize(arr[i],(3,))
            diff = 0.54 - arr[i][2]
            arr[i] += np.array([0,0,diff])
        print("AFTERT============================")
        print(arr)
            
        return arr
        
    def define_sim_ws( self , origin_m ):
        """ 
            Given the center point of the table in the simulator,
            define the measured workspace in terms of coordinates in the simulator
            Coord is origin of table in sim 
        """
        self.sim_origin = origin_m
        midpt_x = (self.aruco_tag_loc[1][0][1][0] + self.aruco_tag_loc[5][0][0][0] ) + (self.aruco_tag_loc[3][0][0][0]  + self.aruco_tag_loc[4][0][1][0] ) 
        midpt_x /= 4 
        
        midpt_y = (self.aruco_tag_loc[1][0][1][1] + self.aruco_tag_loc[5][0][0][1] ) + (self.aruco_tag_loc[3][0][0][1]  + self.aruco_tag_loc[4][0][1][1] )
        midpt_y /= 4 
        origin = [ round(midpt_x) , round(midpt_y) ] #Measured origin in pixels
        arr = []
        
        dist_v = self.px_to_cm( origin ) #Distance from top right of tag 1 to origin in cm
        dist_v = np.array(dist_v) / 100 #Convert to meters
        dist_v[0] = dist_v[0] * (-1) #Since the origin is in the center, this vector is in the 2nd quadrant
        arr.append(dist_v) 
        
        bottom_left = vector_between_points( self.aruco_tag_loc[4][0][1] , origin )
        bottom_left = np.array(bottom_left) / norm(bottom_left) #turn to unit vector
        bottom_left = bottom_left *( math.sqrt( math.pow(self.dimensions[3],2) + math.pow(self.dimensions[2],2) ) / 2 )  

        bottom_left[0] = bottom_left[0] * (-1)
            
        top_right = vector_between_points( self.aruco_tag_loc[3][0][0] , origin )
        top_right = np.array(top_right) / norm(top_right) #turn to unit vector
        top_right = top_right * ( math.sqrt( math.pow(self.dimensions[0],2) + math.pow(self.dimensions[1],2) ) / 2 )  
        top_right[0] = abs(top_right[0])
        arr.append(top_right /100)
        arr.append(bottom_left/100)
        
        t = np.array(self.px_to_cm( self.aruco_tag_loc[5][0][0] )) / 200 #Distance from top right of 1 to top left of 5 / 2
        t[1] = t[1]*(-1)
        arr.append(t)

        for i in range(4):
            arr[i] = np.resize(arr[i],(3,))
            arr[i] += np.array(origin_m)
            
            diff = 0.54 - arr[i][2]
            arr[i] += np.array([0,0,diff])
        return arr
        
        
        
        #Distance list is [ 1->3 , 3->5 , 5->4 , 1->4], going around square clockwise. 
    
    def point_to_sim( self , point ):
        """ Point in m """
        print(point)
        point += np.array( self.sim_origin )
        return point
    
    def alter_table_urdf( self , dim ):
        """ Change sdf file in cairo sim so that the table dimensions are the same as the measured one above. """
        f = open( "/home/epiphyte/Documents/cairo_simulator/assets/table.sdf" , "r" )    
        lines = f.readlines()
        #Lines to alter 10 , 26
        x_size = (dim[0] + dim[2]) / 200
        y_size = ( dim[1] + dim[3]) / 200
        lines[9] = "            <size>{} {} 0.03</size>\n".format(x_size , y_size)
        lines[25] = "            <size>{} {} 0.03</size>\n".format(x_size , y_size)
        f.close()
        f = open("/home/epiphyte/Documents/cairo_simulator/assets/table.sdf" , "w")
        f.writelines( lines )
        f.close()
        
    def distance_from_cam_to_workspace( self ):
        """ Returns distance from camera to workspace """
        pass


















""" 
    References:
    
    Shortest distance between two skew lines in 3D space. Watch just for the scottish accent ---> https://www.youtube.com/watch?v=HC5YikQxwZA
"""



""" FIND VANISHING POINT 
        
        #Get parametrization of lines 1->3 and 4->5
        #u = get_param_mtx( self.aruco_tag_loc[1][0][0] , self.aruco_tag_loc[3][0][0] ) 
        #v = get_param_mtx( self.aruco_tag_loc[4][0][0] , self.aruco_tag_loc[5][0][0] ) 
        
        #eq_x = Eq( t*u[1][0] + u[0][0] - s*v[1][0] - v[0][0] ) # These equations are: (1->3) - (4->5)
        #eq_y = Eq( t*u[1][1] + u[0][1] - s*v[1][1] - v[0][1] ) # For each parametrization
        #eq_z = Eq( t*u[1][2] + u[0][2] - s*v[1][2] - v[0][2] )

        ### Find the cloeset point between two possibly skew lines. ###
        #t, s = symbols('t s')
        #eq1 = Eq( ( u[1][0] * ( t*u[1][0] + u[0][0] - s*v[1][0] - v[0][0] ) + u[1][1] * ( t*u[1][1] + u[0][1] - s*v[1][1] - v[0][1] ) ) , 0 )
        #eq2 =  Eq( ( v[1][0] * ( t*u[1][0] + u[0][0] - s*v[1][0] - v[0][0] ) + v[1][1] * ( t*u[1][1] + u[0][1] - s*v[1][1] - v[0][1] ) ) , 0 )
        #t_s = solve((eq1,eq2),(t,s))
        #if( len(t_s) == 1 ): #Two lines are parallel.
        #   return None
        #Now that we have the vanishing pt, we can use homography to get the actual distance between tags
        #vanishing_pt = (u[0][0] + u[1][0] * t_s[t]  , u[0][1] + u[1][1] * t_s[t] )



"""
