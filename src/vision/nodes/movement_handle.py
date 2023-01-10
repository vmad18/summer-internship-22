#!/usr/bin/env python3

from math import pi, cos, sin
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, Point
from utils.controlling import ControlLoop
from utils.mathutils import EulerAngle, to_rad, to_deg, dist
from std_msgs.msg import Float32MultiArray, Int16
from utils.constants import Mode
from utils.mathutils import *

true, false = True, False



'''
Wrapper Class
'''
class Pos2DHook:

    def __init__(self, yaw:float=None, dist:float=0.0):
        
        #setpoints
        self.psi:float = yaw
        self.dist:float = dist
        self.rot:float = 0.0

        self.ppos: Point = None
 
        self.odom_data:Odometry = Odometry()

        self.twist_msg: Twist = Twist()

        self.reached_yaw: bool = true

    def get_yaw(self)-> float:
        return EulerAngle(q=self.odom_data.pose.pose.orientation).get_yaw()+pi
 

#create abstraction of this as an Error Class that is inherited from
class ErrorFuncs(object):
    
    def __init__(self, ps2D:Pos2DHook):
        self.pos2D = ps2D

    def yaw_error(self, psi:float)-> float:
        angles:EulerAngle = EulerAngle(q=self.pos2D.odom_data.pose.pose.orientation)

        error:float = ((psi)-(angles.get_yaw()+pi))%(2*pi)

        error = error if error <= pi else (error-2*pi)
        
        if abs(to_deg(error)) <= .1:
            #rospy.loginfo(f"Breaking {to_deg(error)}")
            self.pos2D.reached_yaw = true
            return 0.0

        #rospy.loginfo(f"{to_deg((angles.get_yaw()+pi))} {abs(to_deg(error))}")
        return error

    def pos_error(self, pos1: Point, pos2: Point, d: float)-> float: 
        return d - dist(pos1.x, pos2.x, pos1.y, pos2.y)



class MovementHandle(object):

    def __init__(self, ps2D:Pos2DHook):

        self.rate:rospy.Rate = rospy.Rate(60) 

        self.pos2D:Pos2DHook = ps2D
        self.error_funcs = ErrorFuncs(self.pos2D)

        #ControlLoops
        #1.25, 1e-9, 1.5
        self.yaw_controller = ControlLoop(self.error_funcs.yaw_error, 1.25, 0, .1)

        self.pub:rospy.Publisher = rospy.Publisher("cmd_vel", Twist, queue_size=2)
        self.sub1:rospy.Subscriber = rospy.Subscriber("move_h", Float32MultiArray, callback=self.set_pos)
        self.sub2:rospy.Subscriber = rospy.Subscriber("mode", Int16, callback=self.set_mode)
        self.odomsub:rospy.Subscriber = rospy.Subscriber("odom", Odometry, callback=self.update_odom)

        self.mode: Mode = Mode.TEL

        self.start_up: int = 60


    '''
    Subscriber Callbacks
    '''

    def set_pos(self, msg:Float32MultiArray):
        arr:list = msg.data

        if len(arr) < 2 or (self.mode != Mode.TEL and self.mode != Mode.BLOB): return

        #rospy.loginfo(f"{180/pi * (self.pos2D.get_yaw()+pi)} {180/pi * arr[0]} {180/pi * ((self.pos2D.get_yaw()+pi)-arr[0])}")

        rospy.loginfo(f"New setpoints: yaw {self.pos2D.psi} -> {(self.pos2D.get_yaw()- arr[0])%2*pi}  |  travel distance {self.pos2D.dist} -> {arr[1]}")

        self.pos2D.psi = ((self.pos2D.get_yaw()) - arr[0])
        self.pos2D.dist = arr[1]
        self.pos2D.reached_yaw = false


    def set_mode(self, m: Int16)-> None:
        d = m.data
        if d == self.mode.value or d == Mode.OWN.value: return
        rospy.loginfo(f"Changing mode... {m}")
        if d == 1:
            self.mode = Mode.STABLE
        elif d == 2:
            self.mode = Mode.TEL
        elif d == 3:
            self.mode = Mode.SEARCH
        elif d == 4:
            self.mode = Mode.OWN
        elif d == 5:
            self.mode = Mode.BLOB
        self.pos2D.twist_msg = Twist()
    

    def update_odom(self, odom:Odometry):
        self.pos2D.odom_data = odom


    '''
    Movements
    '''

    def yaw(self)-> None:
        
        # if self.pos2D.reached_yaw:
        #     self.mode = Mode.STABLE
        #     self.pos2D.twist_msg.angular.z = 0.0
        #     return

        if self.pos2D.psi == None:
            self.pos2D.twist_msg.angular.z = 0.0 
            return

        turn_speed:float = self.yaw_controller.execute(self.pos2D.psi)
        self.pos2D.twist_msg.angular.z = turn_speed


    def drive_dist(self)-> None:                
        p = self.pos2D.odom_data.pose.pose.position
        
        if self.pos2D.ppos == None:
            self.pos2D.ppos = p
        
        error:float = self.error_funcs.pos_error(self.pos2D.ppos, p, self.pos2D.dist)

        if error <= .001 or self.pos2D.dist <= 0:
            self.pos2D.dist = 0
            self.pos2D.twist_msg.linear.x = 0
            self.pos2D.twist_msg.linear.y = 0
            self.pos2D.ppos = None
            if self.mode == Mode.BLOB:
                self.pos2D.psi = None
            return
        
        # movement vector 

        rospy.loginfo(f"{error} {to_deg(self.pos2D.psi)} {to_deg(self.pos2D.get_yaw()+2*pi)%360} {to_deg(pi+self.pos2D.psi)%360}")
        
        phi: float = self.pos2D.get_yaw() + (pi if self.pos2D.get_yaw() > pi else 0)
        rospy.loginfo(f"{phi} {0 if self.pos2D.get_yaw() > pi else pi}")
        #phi += (pi if self.pos2D.get_yaw() > pi else -pi)
        rospy.loginfo(phi)
        

        self.pos2D.twist_msg.linear.x = min(.5, abs(error)) #min(.1, abs(error)) * cos(phi)
        #self.pos2D.twist_msg.linear.y = min(.1, abs(error)) * sin(phi) # possibly change to current direction/yaw 


    def search(self)-> None:
        self.pos2D.twist_msg.angular.z = .15


    def update(self):

        if not (self.start_up<0): 
            self.start_up-=1
            return 

        if self.mode == Mode.SEARCH:
            self.search()
        elif self.mode == Mode.TEL or self.mode == Mode.BLOB:
            self.yaw()
            self.drive_dist()
        
        self.pub.publish(self.pos2D.twist_msg)
        #TODO other motion stuff


    def run(self): 
        while not rospy.is_shutdown():
            self.rate.sleep()
            self.update()
        


def main():
    rospy.init_node("movement_handle")
    try:
        rospy.loginfo("Starting Movement Handler...")
        pos2D:Pos2DHook = Pos2DHook()
        mh:MovementHandle = MovementHandle(pos2D)
        mh.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()