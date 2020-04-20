import sys
import cv2
import os
from sys import platform
import argparse
import time
try: 
    sys.path.append('../../python');
    from openpose import pyopenpose as op
except ImportError as e:
    raise e 
""" 
=====pose_detection.py=====
Use OpenPose to grab pose images and data from images
"""








"""All Attributes of pyopenpose:

['BODY_135', 'BODY_25', 'BODY_25B', 'COCO_18', 'Datum',
 'MPI_15', 'MPI_15_4', 'Point', 'PoseModel', 'Rectangle', 'WrapperPython', 
 '__doc__', '__file__', '__loader__', '__name__', '__package__', '__spec__', 
 '__version__', 'getPoseBodyPartMapping', 'getPoseMapIndex', 'getPoseNumberBodyParts', 
 'getPosePartPairs', 'get_gpu_number', 'get_images_on_directory', 'init_argv', 'init_int']

"""

class Pose_From_Image():
    
    def __init__( self ):
        self.params = dict()
        self.params["model_folder"] = "/home/epiphyte/Documents/openpose/models"
        self.params["write_json"] = "./op_output"
        self.params["model_pose"] = "BODY_25"
        self.opWrapper = None
        
        
    def get_body_pose_mapping(self):
        return op.getPoseBodyPartMapping(op.BODY_25) 
    
    def start(self):
        self.opWrapper = op.WrapperPython()
        self.opWrapper.configure(self.params)
        self.opWrapper.start()
        
    def kill(self):
        self.opWrapper.stop()
        
    def image(self , img):
        self.start()
        datum = op.Datum()
        imageToProcess = cv2.imread(img)
        datum.cvInputData = imageToProcess
        self.opWrapper.emplaceAndPop([datum])
        self.kill()
        return datum

""" REFERENCES:
From openpose github:
    openpose/examples/tutorial_api_python/

"""
