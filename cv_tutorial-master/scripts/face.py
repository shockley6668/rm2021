#!/usr/bin/env python3

import cv2

import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import numpy as np

def img_callback(msg):
    Image = bridge.imgmsg_to_cv2(msg)
    detectAndDisplay(Image)

def detectAndDisplay(frame):
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # frame_gray = cv.equalizeHist(frame_gray)
    #-- Detect faces
    faces = face_cascade.detectMultiScale(frame_gray)
    for (x,y,w,h) in faces:
        center = (x + w//2, y + h//2)
        frame = cv2.ellipse(frame, center, (w//2, h//2), 0, 0, 360, (255, 0, 255), 4)
        faceROI = frame_gray[y:y+h,x:x+w]
        #-- In each face, detect eyes
        eyes = eyes_cascade.detectMultiScale(faceROI)
        for (x2,y2,w2,h2) in eyes:
            eye_center = (x + x2 + w2//2, y + y2 + h2//2)
            radius = int(round((w2 + h2)*0.25))
            frame = cv2.circle(frame, eye_center, radius, (255, 0, 0 ), 4)
    cv2.imshow('Capture - Face detection', frame)
    cv2.waitKey(30)
if __name__ == '__main__':

    # rp = rospkg.RosPack()
    # path = rp.get_path("rmtt_tracke") 
    face_cascade = cv2.CascadeClassifier('/usr/share/opencv4/haarcascades/haarcascade_frontalface_alt.xml')
    eyes_cascade= cv2.CascadeClassifier('/usr/share/opencv4/haarcascades/haarcascade_eye_tree_eyeglasses.xml')
    bridge = CvBridge()
    rospy.init_node('face_tracker', anonymous=True)
    sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    # pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.spin()
