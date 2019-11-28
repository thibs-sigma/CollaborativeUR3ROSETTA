#!/usr/bin/env python

"""
    ur3_vision.py - Version 0.1 03-10-2019

    Based on the R. Patrick Goebel's moveit_cartesian_demo.py demo code.

    Plan and execute a Cartesian path for the end-effector.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html
"""

import rospy, sys, numpy as np
import moveit_commander
from copy import deepcopy
import geometry_msgs.msg
from cooperative_demo.msg import Tracker # Check package name (if any changes)
import moveit_msgs.msg
import cv2, cv_bridge
from sensor_msgs.msg import Image


from std_msgs.msg import Header, Bool
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

tracker = Tracker()

class ur3_1_vision:
    def __init__(self):
        self.track_flag = False
        self.default_pose_flag = True
        self.UR3_1_is_moving = False
        self._done = False
        
        # Adjust with actual resolution of camera in V-REP (256 x 256) --> (800 x 800)
        self.cx = 400.0
        self.cy = 400.0
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/UR3_1/camera/image_raw', Image, self.image_callback)
        # Subscriber to determine whether or not to use vision
        self.UR3_1_is_moving_sub = rospy.Subscriber('/UR3_1_is_moving', Bool, self.check_moving)
        self.cxy_pub = rospy.Publisher('UR3_1/cxy', Tracker, queue_size=1)
        

    def check_moving(self,data):
        self.UR3_1_is_moving = data.data

    def image_callback(self,msg):
        if not self.UR3_1_is_moving:
            # BEGIN BRIDGE
            image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
            # END BRIDGE
            # BEGIN HSV
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # END HSV
            # BEGIN FILTER
            lower_red = np.array([ 0,  100, 100])
            upper_red = np.array([10, 255, 255])
            mask = cv2.inRange(hsv, lower_red, upper_red)
            (_, cnts, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            #area = cv2.contourArea(cnts)
            h, w, d = image.shape
            # print h, w, d  (800,800,3)
            #BEGIN FINDER
            M = cv2.moments(mask)
            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

            # cx range (55,750) cy range( 55, ~ )
            # END FINDER
            # Isolate largest contour
            #  contour_sizes = [(cv2.contourArea(contour), contour) for contour in cnts]
            #  biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]
                for i, c in enumerate(cnts):
                    area = cv2.contourArea(c)
                    if area > 500: # Reduced from 7500 to 500 to get better detection on our application
                        self.track_flag = True
                        self.cx = cx
                        self.cy = cy
                        self.error_x = self.cx - w/2
                        self.error_y = self.cy - (h/2+195)
                        tracker.x = cx
                        tracker.y = cy
                        tracker.flag1 = self.track_flag
                        tracker.error_x = self.error_x
                        tracker.error_y = self.error_y
                        # (_,_,w_b,h_b)=cv2.boundingRect(c)
                        # print w_b,h_b
                        # BEGIN circle
                        cv2.circle(image, (cx, cy), 10, (0,0,0), -1)
                        cv2.putText(image, "({}, {})".format(int(cx), int(cy)), (int(cx-5), int(cy+15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                        cv2.drawContours(image, cnts, -1, (255, 255, 255),1)
                        #BGIN CONTROL
                        break
                    else:
                        self.track_flag = False
                        tracker.flag1 = self.track_flag


            self.cxy_pub.publish(tracker)

            # Visual output
            cv2.namedWindow("camera_UR3_1", 1)
            cv2.imshow("camera_UR3_1", image )

            cv2.waitKey(1)
            
        if self.UR3_1_is_moving == True:
            # Debug terminal
            print("---- UR3_1 IS MOVING - SHUTTING DOWN VISION ----")
            self._done = True
            # Debug terminal
            # print(self._done)
            # Calling shutdown
            rospy.signal_shutdown("UR3_1_ismoving")
            # sys.exit(0)
            return

    def clean_shutdown(self):
        """Handles ROS shutdown (Ctrl-C) safely."""
        if not self._done:
            rospy.logwarn('Aborting: Shutting down safely...')

def main(args):
    rospy.init_node("ur3_vision", anonymous=False)
    
    follower=ur3_1_vision()

    try:
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    print "OpenCV Version:",cv2.__version__

    main(sys.argv)