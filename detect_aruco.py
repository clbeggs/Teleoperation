import numpy as np
import cv2, PIL
from cv2 import aruco


class Detect_Aruco():
    
    def __init__(self):
        self.camera_matrix = np.array([[1.24461232e+03,0.00000000e+00,5.96912232e+02],[0.00000000e+00,1.25196774e+03,7.75402584e+02],[0.00000000e+00,0.00000000e+00,1.00000000e+00]])
        self.dist = np.array( [ 0.28950125 , -1.81345971 , -0.00877125 , 0.00460012 , 3.66902124] )
        
    def detect_tags( self , image_path ):
        """ Detect tag, return corners, id, and image with tags outlined """
        image = cv2.imread( image_path )
        image = self.image_resize(image,1200,900) # resize image, maintain aspect ratio
        image_cpy = image.copy() #Unaltered image
        
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_50) #Aruco Tag dictionary
        parameters = aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers( image , aruco_dict , parameters=parameters ) #detect tags
        aruco.drawDetectedMarkers(image, corners, ids) #Draw detected tags
        #aruco.drawDetectedMarkers(image, rejectedImgPoints, borderColor=(100, 0, 240)) #Draw rejected tags
        return corners , ids , image , image_cpy
        
    def estimate_tag_pose( self , image , corners , ids ,  axis_size=0.2 ):
        """ Draws coordinate system on every Aruco Tag, returns image, rotation and translation vector """
        
        rotation_vector , translation_vector , _ = aruco.estimatePoseSingleMarkers( corners , 0.05 , self.camera_matrix , self.dist )
        for i in range( len(ids) ):
            aruco.drawAxis( image , self.camera_matrix , self.dist , rotation_vector , translation_vector , axis_size )
            
        return image , rotation_vector , translation_vector
        
    def image_resize(self,image, width = None, height = None, inter = cv2.INTER_AREA):
        """ 
        Resize Image without distortion 
        ref: https://stackoverflow.com/questions/44650888/resize-an-image-without-distortion-opencv
        """
        # initialize the dimensions of the image to be resized and
        # grab the image size
        dim = None
        (h, w) = image.shape[:2]

        # if both the width and height are None, then return the
        # original image
        if width is None and height is None:
           return image

        # check to see if the width is None
        if width is None:
           # calculate the ratio of the height and construct the
           # dimensions
           r = height / float(h)
           dim = (int(w * r), height)

        # otherwise, the height is None
        else:
            # calculate the ratio of the width and construct the
            # dimensions
            r = width / float(w)
            dim = (width, int(h * r))

        # resize the image
        resized = cv2.resize(image, dim, interpolation = inter)

        # return the resized image
        return resized



"""
s = Detect_Aruco()
c , i , im , im_c = s.detect_tags("./media/ar3.jpg")
im_2 , r , t = s.estimate_tag_pose( im_c , c , i , 0.1 )
cv2.imshow("S" , im) 
cv2.waitKey(0)
cv2.destroyAllWindows()
print(c)
print(i)
"""
""" All Aruco Attributes:

['Board_create', 'CORNER_REFINE_APRILTAG', 'CORNER_REFINE_CONTOUR', 
'CORNER_REFINE_NONE', 'CORNER_REFINE_SUBPIX', 'CharucoBoard_create', 
'DICT_4X4_100', 'DICT_4X4_1000', 'DICT_4X4_250', 'DICT_4X4_50', 'DICT_5X5_100',
 'DICT_5X5_1000', 'DICT_5X5_250', 'DICT_5X5_50', 'DICT_6X6_100', 'DICT_6X6_1000', 
 'DICT_6X6_250', 'DICT_6X6_50', 'DICT_7X7_100', 'DICT_7X7_1000', 'DICT_7X7_250', 
 'DICT_7X7_50', 'DICT_APRILTAG_16H5', 'DICT_APRILTAG_16h5', 'DICT_APRILTAG_25H9', 
 'DICT_APRILTAG_25h9', 'DICT_APRILTAG_36H10', 'DICT_APRILTAG_36H11', 'DICT_APRILTAG_36h10', 
 'DICT_APRILTAG_36h11', 'DICT_ARUCO_ORIGINAL', 'DetectorParameters_create', 
 'Dictionary_create', 'Dictionary_create_from', 'Dictionary_get', 'Dictionary_getBitsFromByteList',
  'Dictionary_getByteListFromBits', 'GridBoard_create', '__doc__', '__loader__', '__name__', 
  '__package__', '__spec__', 'calibrateCameraAruco', 'calibrateCameraArucoExtended', 
  'calibrateCameraCharuco', 'calibrateCameraCharucoExtended', 'custom_dictionary', 
  'custom_dictionary_from', 'detectCharucoDiamond', 'detectMarkers', 'drawAxis', 
  'drawDetectedCornersCharuco', 'drawDetectedDiamonds', 'drawDetectedMarkers', 
  'drawMarker', 'drawPlanarBoard', 'estimatePoseBoard', 'estimatePoseCharucoBoard', 
  'estimatePoseSingleMarkers', 'getBoardObjectAndImagePoints', 'getPredefinedDictionary', 
  'interpolateCornersCharuco', 'refineDetectedMarkers']

"""