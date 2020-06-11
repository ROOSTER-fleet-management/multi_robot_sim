import math

class Pose2d(object):
    """ class to create an object that holds coordinates and theta (orientation) of a point in 2D """
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

def transform_2d(origin_pf, origin_cf):
    """ function to transform the coordinates in the child frame to the parent frame"""
    x = origin_cf.x*math.cos(origin_pf.theta) - origin_cf.y*math.sin(origin_pf.theta) + origin_pf.x
    y = origin_cf.x*math.sin(origin_pf.theta) + origin_cf.y*math.cos(origin_pf.theta) + origin_pf.y
    theta = origin_cf.theta + origin_pf.theta
    return Pose2d(x,y,theta)