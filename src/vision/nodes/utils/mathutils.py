from math import atan2, pi, sqrt
from geometry_msgs.msg import Quaternion


class EulerAngle(object):
    
    def __init__(self, y:float = 0.0, p:float = 0.0, r:float = 0.0, q:Quaternion = None):
        self.quant = q
        self.yaw = y
        self.pitch = p
        self.roll = r

    def _get_yaw_from_q(self):
        self.yaw = atan2(2*(self.quant.w*self.quant.z+self.quant.x*self.quant.y), 1-2*(self.quant.y*self.quant.y+self.quant.z*self.quant.z))
        return self.yaw 

    def get_yaw(self):
        if not (self.quant is None):
            return self._get_yaw_from_q()
        return self.yaw

    #TODO pitch and roll calculations


def to_rad(angle:float):
    return angle*pi/180.0

def to_deg(angle:float):
    return angle*180.0/pi

def dist(x1:float, x2:float, y1:float, y2:float): return sqrt((x1-x2)**2 + (y1-y2)**2)