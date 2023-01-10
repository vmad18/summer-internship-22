#!/usr/bin/env python3

from cmath import pi

from defer import return_value
import rospy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from pyzbar.pyzbar import decode
from std_msgs.msg import Float32MultiArray, Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
from sensor_msgs.msg import Imu, LaserScan
from utils.mathutils import EulerAngle, to_rad, to_deg
from pyzbar.pyzbar import decode
from utils.constants import *
from math import atan2
from utils.constants import Mode
from transformers import TrOCRProcessor, VisionEncoderDecoderModel
from PIL import Image

'''
create topic: vision 
publish -> vision
publish -> topic: cmd-vel (or seperate topic that is subscribed by another node handles movement)
'''

'''
what i would like to do:
create scheduler:
runs threads that does stuff
'''

true, false = True, False


class DetectHandle(object):

    def __init__(self):
        self.rate:rospy.Rate = rospy.Rate(30)

        self.pub_mov:rospy.Publisher = rospy.Publisher("move_h", Float32MultiArray, queue_size=2)
        self.pub_mode:rospy.Publisher = rospy.Publisher("mode", Int16, queue_size=2)
        self.sub:rospy.Subscriber = rospy.Subscriber("vision", Image, callback=self.handle)

        self.mode: Mode = Mode.SEARCH

        self.bridge:CvBridge = CvBridge()
        self.all_cmnds:dict = {}

        self.processor = TrOCRProcessor.from_pretrained("microsoft/trocr-base-handwritten")
        self.model = VisionEncoderDecoderModel.from_pretrained("microsoft/trocr-base-handwritten")

        # kind of crude
        # helps robot get most of the green area in its view
        # value is dependent on angular speed

        self.detects = 10

    # make listener get these frames
    # then subscribe to handler to handle the stuff
    def handle(self, img:Image)-> None:
        img = self.bridge.imgmsg_to_cv2(img)

        if self.mode == Mode.SEARCH:
            self.mask_out(img)
        #self.scan_qr_code(img)

    '''
    Distance to point on plane calculation
    '''
    def dist_to_plane(self, point: np.ndarray)-> float:

        point3D: np.ndarray = np.asarray([point[0], point[1], 1]) # create a direction vector in R3
        l_transform: np.ndarray = (camera_mati @ point3D) # perform linear transformation to real world coordinates

        dist: float = translation_vect[1]/l_transform[1] - translation_vect[1] # compute distance and adjust by translation
        return dist

    '''
    Returns the biggest 
    '''
    def mask_out(self, frame):

        msg:Float32MultiArray = Float32MultiArray()

        lower_p = np.asarray([36, 20, 20])
        upper_p = np.asarray([70, 255, 255])
        green = cv.inRange(cv.cvtColor(frame, cv.COLOR_BGR2HSV), lower_p, upper_p)

        cv.imwrite("/home/ros/cv_ros/src/vision/nodes/greenblob5.jpg", frame)
        img = frame

        ret, thresh = cv.threshold(green, 40, 255, 0)

        contours, hiearchy = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

        if len(contours) == 0 or self.mode != self.mode.SEARCH:
            self.detects = 10
            return

        c = max(contours, key=cv.contourArea) # Extracts biggest green contour

        area = cv.contourArea(c) #Green's Theorem

        # Area threshold, if it is less than the thresh, return | helps to filter out random green noise

        if area < 7e3:
            rospy.loginfo(f"Not detecting anything green! Max Green Area: {area}")
            msg.data = [0, 0]
            self.pub_mov.publish(msg)
            self.detects = 10
            return


        cv.drawContours(frame, c, -1, 255, 3)

        # This is to help the Turtle Bot make sure that it is relatively close to the center of the green blob, and not only at its edge.
        if self.detects > 0:
            rospy.loginfo(f"Detecting: {self.detects}")
            self.detects-=1
            return

        rospy.loginfo(f"Detected something green with an area of: {area}")

        x, y, w, h = cv.boundingRect(c)
        cv.rectangle(img, (x, y), (x+w, y+h), (0, 0, 0), 2)
        img = cv.circle(img, (x+w//2, y+h//2), 2, thickness = 3, color = (0, 100, 255))

        # center of blob
        px = x+(w//2)
        py = y+(h//2)

        # center of frame
        cx = np.shape(frame)[1]//2
        cy = np.shape(frame)[0]//2

        # direction vector from the center of blob to center of FOV
        dir = np.asarray([px-cx, py-cy])

        # Mess around with x comp of atan2, higher value makes it more stable
        # For it to be 99.9% accurate, the distance from the robot to the object is required
        # 2nd parameter is the x focal length
        yaw: float = atan2(dir[0], 824.41491208)

        if self.dist_to_plane(np.asarray([px, py]))/25.4 < 0:
            self.detects = 10
            return

        self.update_mode(Mode.BLOB)

        rospy.loginfo(f"Sending yaw error... {atan2(dir[0], 1800) * 180/pi} {dir[0]}")
        rospy.loginfo(f"Distance to blob: {self.dist_to_plane(np.asarray([px, py]))/25.4} in.")


        arr:list = [yaw, self.dist_to_plane(np.asarray([px, py]))/1000.0 + camera_bot/1000.0]
        msg.data = arr
        self.pub_mov.publish(msg)
        self.update_mode(self.mode.OWN)

        cv.imwrite("/home/ros/cv_ros/src/vision/nodes/greenblob6.jpg", frame)

        cv.imshow("frame", frame)
        cv.waitKey(0)

    # Transformer used for mapping Spatial Domain to Language/Text
    def ocr(self, frame)-> list:
        frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)

        pixel_values = self.processor(frame, return_tensors="pt").pixel_values
        generated_ids = self.model.generate(pixel_values)
        return self.processor.batch_decode(generated_ids, skip_special_tokens=True)


    # QR Code scanner

    def scan_qr_code(self, frame)-> None:

        gray = cv.cvtColor(frame, 0) # convert frame to gray
        qr = decode(gray) # extract qr codes from image

        outs:list = []

        if len(qr) == 0:
            return

        for code in qr:
            data = code.data.decode("utf-8") # decode extracted data to UTF-8
            outs.append(str(data)) # add data to return list

        if len(outs) == 0: return

        for i in outs:
            if i in self.all_cmnds.keys():
                self.all_cmnds[i]+=1
            else:
                self.all_cmnds[i]=1


    def update_mode(self, mode: Mode = None):
        self.mode = self.mode if mode == None else mode
        msg: Int16 = Int16()
        msg.data = self.mode.value
        self.pub_mode.publish(msg)


    def run(self)-> None:
        while not rospy.is_shutdown():
            #do image stuff
            self.rate.sleep()
            self.update_mode()



def main()-> None:
    rospy.init_node("detect_hanlder")
    try:
        rospy.loginfo("Starting Handler...")
        dh:DetectHandle = DetectHandle()
        dh.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
