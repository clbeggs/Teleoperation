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
    
    def measure_workspace( self , image_or_path ):
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
        corners , ids , image , image_cpy = self.aruco.detect_tags( image_or_path )
        
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
        """ 
            With input in pixels, convert it to cm in the simulator
        """
        """ TODO change so that the the point is measured from two closest vectors on square """
        
        
        v = vector_between_points( self.aruco_tag_loc[1][0][1] , coord )# vector between top right of tag 1 to coordinate
        
        w = vector_between_points(self.aruco_tag_loc[1][0][1] , self.aruco_tag_loc[3][0][0] ) #vector between top right of tag 1 to top left of tag 3
        z = vector_between_points( self.aruco_tag_loc[1][0][2] , self.aruco_tag_loc[4][0][1] ) # vector between bottom right of tag 1 to top right of tag 4 
        
        u = projection( v , w ) #x cooordinate in px
        p = projection( v , z ) #y coordinate in px

        return [norm(u)*(self.dimensions[0]/norm(w)) , norm(p)*(self.dimensions[3]/norm(z)) ]
        

        
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
        dist_v[0] = dist_v[0]  #Since the origin is in the center, this vector is in the 2nd quadrant
        arr.append(dist_v) 
        
        bottom_left = vector_between_points( self.aruco_tag_loc[4][0][1] , origin )
        bottom_left = np.array(bottom_left) / norm(bottom_left) #turn to unit vector
        bottom_left = bottom_left *( math.sqrt( math.pow(self.dimensions[3],2) + math.pow(self.dimensions[2],2) ) / 2 )  

        bottom_left[0] = -abs(bottom_left[0])
        bottom_left[1] = abs(bottom_left[1]) 
        top_right = vector_between_points( self.aruco_tag_loc[3][0][0] , origin )
        top_right = np.array(top_right) / norm(top_right) #turn to unit vector
        top_right = top_right * ( math.sqrt( math.pow(self.dimensions[0],2) + math.pow(self.dimensions[1],2) ) / 2 )  
        top_right[0] = abs(top_right[0])
        top_right[1] = (-1)*top_right[1]
        arr.append(top_right /100)
        arr.append(bottom_left/100)
        
        bottom_right = np.array(self.px_to_cm( self.aruco_tag_loc[5][0][0] )) / 200 #Distance from top right of 1 to top left of 5 / 2
        bottom_right[1] = bottom_right[1]*(-1)
        bottom_right[0] = bottom_right[0]*(-1)
        arr.append(bottom_right)
        arr2 = []
        for i in range(4):
            arr2.append( np.array([arr[i][0] + origin_m[0], arr[i][1] + origin_m[1] , 0.54]) )
        arr2.append( np.array([dist_v[0] + origin_m[0], dist_v[1] + origin_m[1] , 0.7]) )
        arr2.append( np.array([bottom_left[0]/100 + origin_m[0], bottom_left[1]/100 + origin_m[1] , 0.9]) )
        print("Top Left: " , dist_v )
        print("Top Right: " , top_right / 100 )
        print("Bottom Right: " , bottom_right )
        print("Bottom Left: " , bottom_left / 100 )
        return arr2
        
        
        
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

    def track_tag( self, image_or_path , id=28 ):
        corners , ids , image , image_cpy = self.aruco.detect_tags( image_or_path )
        
        return image
    def watch_to_sim( self , image_path ):
        """ 
            Assuming Eagle Eye View
            Returns x,y,z coordinates
        """
        corners , ids , image , _ = self.aruco.detect_tags( image_path )
        
        if( ids is None ):
            return None , None
        if( [28] not in ids ):
            print("Watch not found in photo.")
            return None , None
        
        #Get values of watch
        watch_indx = list(ids).index([28])
        watch_corners = corners[watch_indx][0]
        watch_center = [ int( ( watch_corners[0][0] + watch_corners[2][0] ) / 2 ) , int( ( watch_corners[0][1] + watch_corners[2][1] ) / 2 ) ]
        
        """
        print("999999999999999999999999999999999999999999999999999999999999999999999999999")
        ia = 0 
        for a in corners:
            print("Corner " , a[0])
            print("Id: " , ids[ia])
            ia += 1
            print("")
        
        print("===========================")
        print("Watch Indx " , watch_indx)
        print("===========================")
        print("Watch Corners " , watch_corners )
        print("===========================")
        
        print("===========================")
        print(self.aruco_tag_loc[1][0][1] )
        print(self.px_to_cm([self.aruco_tag_loc[1][0][1][0] + 100 , self.aruco_tag_loc[1][0][1][1] + 100] ))
        print("999999999999999999999999999999999999999999999999999999999999999999999999999")
        """
        
        watch_cm = self.px_to_cm( watch_center )
        print("Top Left " , self.px_to_cm(  self.aruco_tag_loc[1][0][1] ))
        print("Bottom Right " , self.px_to_cm( self.aruco_tag_loc[5][0][0] ))
        print("Bottom Left " , self.px_to_cm( self.aruco_tag_loc[4][0][1] ))
        print("Top Right " , self.px_to_cm( self.aruco_tag_loc[3][0][0] ))
        print("Watch cm: " , watch_cm)
        print( self.aruco_tag_centers )
        print("Center " , watch_center)
        print("\n\n")
        watch_cm_three = np.array([self.sim_origin[0] + (watch_cm[0] / 100) , self.sim_origin[1] + (watch_cm[1] / 100) , self.sim_origin[2] + 0.2])
        
        #draw circle on watch
        cv2.circle( image , tuple(watch_center) , 7 , (0, 0, 255) , 5)
        
        return watch_cm_three , image

        















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
