#!/usr/bin/env python3

import rospy
import cv2 as cv
from pyzbar.pyzbar import decode
import cv_bridge
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
from sensor_msgs.msg import Imu, LaserScan
from utils.mathutils import EulerAngle, to_rad, to_deg
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

'''
subscribe -> vision topic
publish -> cmd_vel
'''

true, false = True, False

class DetectHook(object):

    def __init__(self):
        self.x:float = -1
        self.y:float = 0.0
        self.z:float = 0.0

        self.recieved:bool = false
        self.odom_data:Odometry = Odometry()


class DetectListener(object):

    '''
    di just cuz    
    '''
    def __init__(self, dh:DetectHook):
        self.dh:DetectHook = dh

        self.rate:rospy.Rate = rospy.Rate(60) 
        
        self.sub_v:rospy.Publisher = rospy.Publisher("vision", Image, queue_size=2)
        self.sub_c:rospy.Subscriber = rospy.Subscriber("camera", Image, callback=self.capt_video)

        self.bridge:CvBridge = CvBridge()


    def yaw_test(self):
        psi:float = to_rad(float(input("degree checkpoint? ")))
        self.dh.x = psi
        arr:list = [self.dh.x, -1]
        msg:Float32MultiArray = Float32MultiArray()
        msg.data = arr
        self.pub.publish(msg)

    def dist_test(self):
        d:float = float(input("dist checkpoint? "))
        self.dh.y = d
        arr:list = [self.dh.x, self.dh.y]
        msg:Float32MultiArray = Float32MultiArray()
        msg.data = arr
        self.pub.publish(msg)

    def capt_video(self, img:Image)-> None:
        rospy.loginfo("Recieved New Frame")
        self.sub_v.publish(img)

        # cv.imshow("frame", self.bridge.imgmsg_to_cv2(img))
        # cv.waitKey(0)

    def update(self)-> None:
        pass


    def go_to_blob(self): 
        pass


    #persist daemon thread
    def run(self)-> None: 
        while not rospy.is_shutdown():
            #if self.dh.recieved: rospy.loginfo("new information recieved...")
            #self.update()
            self.rate.sleep()
            # self.yaw_test()
            # self.dist_test()



def main()-> None:
    rospy.init_node("detect_listener")
    try:
        rospy.loginfo("Starting Listener...")
        dh:DetectHook = DetectHook()
        dl:DetectListener = DetectListener(dh)
        dl.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()