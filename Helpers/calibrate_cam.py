""" Calibrate Camera """

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('../media/camera_calibration/*.jpg')
print("Starting!")
for fname in images:
    print("Img: {}".format(fname))
    img = cv2.imread(fname)
    img = image_resize(img , 1200,900) # Need to resize images, iphone7 too high resolution
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7,6), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        #cv2.drawChessboardCorners(img, (7,6), corners2, ret)
        #cv2.imshow('img', img)
        #cv2.waitKey(0)
#cv2.destroyAllWindows()       
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (1200,1600), None,None)
f = open( "camera_calib.txt" , "w" )
f.write( "ret: {}\nmtx: {}\ndist: {}\nrvecs: {}\ntvecs: {}".format(ret, mtx, dist, rvecs, tvecs) )
f.close()

#mtx - Output 3x3 floating-point camera matrix
#dist - Output vector of distortion coefficients 
#rvecs - Output vector of rotation vectors 
#tvecs - Output vector of translation vectors

