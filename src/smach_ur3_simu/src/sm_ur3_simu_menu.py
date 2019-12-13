#!/usr/bin/env python
import os
import subprocess
import time
import sys

from std_msgs.msg import String, Bool

from cv2 import imread

import numpy as np
from numpy import linalg as LA

import rospy
import threading

import smach
from smach import State, StateMachine, Concurrence

import smach_ros
from smach_ros import ServiceState, SimpleActionState, MonitorState, IntrospectionServer

import std_srvs.srv

from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, Pose, Point, PointStamped

from sensor_msgs.msg import JointState
from actionlib_msgs.msg import GoalStatus
from actionlib import SimpleActionClient

import math

from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

from tf import TransformListener

user = os.environ['HOME']

class RESET(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
    
    def execute(self, userdata):
        print("Inside init state machine\n")
        print(rospy.get_name())

        # Wait
        rospy.sleep(2)

        rospy.loginfo("State machine initialized!")

        return 'success'

class RESETSIMU(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

        # Publishers
        self.stop_pub = rospy.Publisher("/stopSimulation", Bool, queue_size=1)
        self.start_pub = rospy.Publisher("/startSimulation", Bool, queue_size=1)
    
    def execute(self, userdata):
        print("Inside RESETSIMU state machine\n")

        # Call 'dual_ur3_resetPose' script
        subprocess.check_call(user + "/CollaborativeUR3ROSETTA/src/cooperative_demo/src/dual_ur3_resetPose.py")

        rospy.loginfo("Robot state reset!")

        # Wait for termination
        rospy.sleep(1)

        # STOP SIMU VREP
        self.stop_pub.publish(True)

        # Wait a bit
        rospy.sleep(1)

        # START AGAIN SIMU VREP
        self.start_pub.publish(True)

        # Wait a bit
        rospy.sleep(1)

        return 'success'


class STOPSIMU(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

        # Publishers
        self.stop_pub = rospy.Publisher("/stopSimulation", Bool, queue_size=1)

    def execute(self, userdata):
        print("Inside STOPSIMU state machine\n")

        # Stop the simulation
        self.stop_pub.publish(True)

        rospy.loginfo("Robot and simulation stopped!")

        # Wait for termination
        rospy.sleep(1)

        rospy.signal_shutdown('STOP requested')

        return 'success'


class CHOOSEACTION(State):
    def __init__(self):
        State.__init__(self, outcomes=['fullDemo', 'getObject', 'manipSync', 'resetPose', 'stop'])

        # Subscribers
        rospy.Subscriber("/desired_action", String, self.actionRequestedCallback)

    def actionRequestedCallback(self, data):
        self.actionRequested_str = data.data
        self.actionReceived = 1
        # print (self.actionRequested_str)
        # return str(actionRequested_str)

    def execute(self, userdata):
        print("Inside CHOOSEACTION state machine\n")

        self.actionReceived = 0
        while self.actionReceived == 0:
            rospy.logwarn_throttle(3,'Waiting for action request...')


        if self.actionRequested_str == 'stop':
            print('Stop requested')
            return 'stop'
        elif self.actionRequested_str == 'get_object':
            print('Pickup requested')
            return 'getObject'
        elif self.actionRequested_str == 'manip':
            print('Manipulation requested')
            return 'manipSync'
        elif self.actionRequested_str == 'full':
            print('Full demo requested')
            return 'fullDemo'
        elif self.actionRequested_str == 'reset':
            print('Reset pose requested')
            return 'resetPose'


# GET OBJECT TASK
class UR31VISION(State):
    def __init__(self):
        State.__init__(self, outcomes=['UR3_1_visionOK', 'stop'])
        
    def execute(self, userdata):
        print("Inside UR31VISION state machine\n")
        # pwd = os.getcwd()
        # os.system("/home/ridgebackbaxter/CollaborativeBaxter_ws/src/assembly_task/src/baxter_object_prediction.py")

        # Call 'ur3_vision' script
        # subprocess.check_call(user + "/CollaborativeUR3ROSETTA/src/cooperative_demo/src/ur3_vision.py") # Errors sometimes generated at closure...
        os.system(user + "/CollaborativeUR3ROSETTA/src/cooperative_demo/src/ur3_vision.py")

        # Wait for termination
        rospy.sleep(1)

        return 'UR3_1_visionOK'

class UR32VISION(State):
    def __init__(self):
        State.__init__(self, outcomes=['UR3_2_visionOK', 'stop'])
        
    def execute(self, userdata):
        print("Inside UR32VISION state machine\n")
        # pwd = os.getcwd()
        # os.system("/home/ridgebackbaxter/CollaborativeBaxter_ws/src/assembly_task/src/baxter_object_prediction.py")

        # Call 'ur3_2_vision' script
        # subprocess.check_call(user + "/CollaborativeUR3ROSETTA/src/cooperative_demo/src/ur3_2_vision.py") # Errors sometimes generated at closure...
        os.system(user + "/CollaborativeUR3ROSETTA/src/cooperative_demo/src/ur3_2_vision.py")

        # Wait for termination
        rospy.sleep(1)

        return 'UR3_2_visionOK'

class GETOBJECT(State):
    def __init__(self):
        State.__init__(self, outcomes=['pickupOK', 'stop'])
        
    def execute(self, userdata):
        print("Inside GETOBJECT state machine\n")
        # pwd = os.getcwd()
        # os.system("/home/ridgebackbaxter/CollaborativeBaxter_ws/src/assembly_task/src/baxter_object_prediction.py")

        # Call 'baxter_object_prediction' script
        subprocess.check_call(user + "/CollaborativeUR3ROSETTA/src/cooperative_demo/src/dual_ur3_getObject_sync_ActionGoal_v2.py")

        # Wait for termination
        rospy.sleep(1)

        return 'pickupOK'

class GETOBJECTONLY(State):
    def __init__(self):
        State.__init__(self, outcomes=['pickupOK', 'stop'])
        
    def execute(self, userdata):
        print("Inside GETOBJECT state machine\n")
        # pwd = os.getcwd()
        # os.system("/home/ridgebackbaxter/CollaborativeBaxter_ws/src/assembly_task/src/baxter_object_prediction.py")

        # Call 'baxter_object_prediction' script
        subprocess.check_call(user + "/CollaborativeUR3ROSETTA/src/cooperative_demo/src/dual_ur3_getObject_sync_ActionGoal_v2.py")

        # Wait for termination
        rospy.sleep(1)

        return 'pickupOnlyOK'

# MANIPULATION TASK
class MANIPULATION(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'fail']) 
        
    def execute(self, userdata):
        print("Inside MANIPULATION state machine\n")

        # Call 'pickup_object' script
        # pwd = os.getcwd()
        # os.system(pwd + "/src/assembly_task/src/request_action.py")
        
        # Call 'dual_ur3_getObject' script
        subprocess.check_call(user + "/CollaborativeUR3ROSETTA/src/cooperative_demo/src/dual_ur3_motionPlanning_sync_ActionGoal_v2.py")

        # Wait for termination
        rospy.sleep(1)

        return 'success'


def clean_shutdown():
    """Handles ROS shutdown (Ctrl-C) safely."""
    rospy.logwarn('Aborting: Shutting down safely...')
    sm.request_preempt()
    rospy.sleep(-1)

if __name__ == "__main__":

    try:
        rospy.init_node('ur3_simu_SMACH')
        
        # Create a SMACH state machine
        sm = StateMachine(outcomes=['success', 'fail', 'stop'])

        # Open the container (HERE IS DEFINED THE SM)
        with sm:
            # Reset the dual UR3 simulation
            StateMachine.add('RESET', RESET(), transitions={'success':'CHOOSE_ACTION'})

            StateMachine.add('RESET_POSE', RESETSIMU(), transitions={'success': 'CHOOSE_ACTION'})
            
            StateMachine.add('STOP_SIMU', STOPSIMU(), transitions={'success':'stop'})

            # Go to CHOOSE_ACTION state
            StateMachine.add('CHOOSE_ACTION', CHOOSEACTION(), transitions={'getObject': 'PICKUPONLY_ACTION', 'manipSync': 'MANIP_ACTION', 'fullDemo': 'PICKUP_ACTION', 'resetPose': 'RESET_POSE', 'stop': 'STOP_SIMU'})

            # GET OBJECT TASK (Concurrence allow to execute two parallel states)
            sm_pickup = Concurrence(outcomes=['success', 'fail'], default_outcome='fail', outcome_map={'success':{'UR3_1_VISION':'UR3_1_visionOK', 'UR3_2_VISION':'UR3_2_visionOK', 'GETOBJECT':'pickupOK'}, 'fail':{'UR3_1_VISION':'stop', 'UR3_2_VISION':'stop', 'GETOBJECT':'stop'}})

            sm_pickupOnly = Concurrence(outcomes=['success', 'fail'], default_outcome='fail', outcome_map={'success':{'UR3_1_VISION':'UR3_1_visionOK', 'UR3_2_VISION':'UR3_2_visionOK', 'GETOBJECT':'pickupOK'}, 'fail':{'UR3_1_VISION':'stop', 'UR3_2_VISION':'stop', 'GETOBJECT':'stop'}})


            with sm_pickup:
                # Concurrence.add('MENU_PICKUP', MENUPICKUP())
                Concurrence.add('UR3_1_VISION', UR31VISION())
                Concurrence.add('UR3_2_VISION', UR32VISION())
                Concurrence.add('GETOBJECT', GETOBJECT())
            
            with sm_pickupOnly:
                # Concurrence.add('MENU_PICKUP', MENUPICKUP())
                Concurrence.add('UR3_1_VISION', UR31VISION())
                Concurrence.add('UR3_2_VISION', UR32VISION())
                Concurrence.add('GETOBJECT', GETOBJECT())
                
            StateMachine.add('PICKUP_ACTION', sm_pickup, transitions={'success':'MANIP_ACTION', 'fail':'stop'})

            StateMachine.add('PICKUPONLY_ACTION', sm_pickupOnly, transitions={'success':'CHOOSE_ACTION', 'fail':'stop'})  
               
            StateMachine.add('MANIP_ACTION', MANIPULATION(), transitions={'success':'CHOOSE_ACTION', 'fail':'stop'})          
        
        # Attach a SMACH introspection server
        sis = IntrospectionServer('ur3_simu_SMACH_introspection', sm, '/UR3_SIMU')
        sis.start()

        # Set preempt handler
        smach_ros.set_preempt_handler(sm)

        # Execute SMACH tree in a separate thread so that we can ctrl-c the script
        smach_thread = threading.Thread(target = sm.execute)

        smach_thread.start()

        

        # Signal handler (wait for CTRL+C)
        rospy.spin()

        rospy.on_shutdown(clean_shutdown)

    
    except rospy.ROSInterruptException:

        rospy.signal_shutdown('SMACH terminated')
        pass
    




