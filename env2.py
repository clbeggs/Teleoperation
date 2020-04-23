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
    
    
    