#!/usr/bin/env python

"""
    moveit_ik_demo.py - Version 0.1 2014-01-14
    
    Use inverse kinemtatics to move the end effector to a specified pose
    
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

import rospy, sys
import moveit_commander
from std_msgs.msg import Bool, String
from moveit_msgs.msg import RobotTrajectory, PlanningScene
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import CameraInfo
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from cooperative_demo.msg import Tracker

import image_geometry
import tf

camera_info = None
camera_state_info = None
object_location = None

# class UR3_motionPlanning:

def initCamera(data):
    global camera_info
    camera_info = data
    # print(camera_info)

def getCameraState(data):
    global camera_state_info
    camera_state_info = data

def getObjectLocation(data):
    global object_location
    # global object_orientation
    # global object_name
    # global desired_object
    # if desired_object is not None and desired_object in data.names:
    # i = data.names.index(desired_object)
    # object_name = data.names[i]
    x = data.x
    y = data.y
    object_location = [x, y]
    # object_orientation = tf.transformations.quaternion_from_euler(
    #     0, 0, -data.theta[i])
    # UR3_motionPlanning()
    

def planning_setup():

    # Initialize the move_group API
    moveit_commander.roscpp_initialize(sys.argv)

    # Use the planning scene object to add or remove objects //Interface
    scene = moveit_commander.PlanningSceneInterface()

    rospy.sleep(0.5)

    REFERENCE_FRAME = '/world'
    p = PlanningScene()
    p.is_diff = True    

    # Create a scene publisher to push changes to the scene //PlanningScene
    scene_pub = rospy.Publisher('/move_group/monitored_planning_scene', PlanningScene, queue_size=1)

  

    pose_Ground = PoseStamped()
    pose_Ground.header.frame_id = REFERENCE_FRAME
    pose_Ground.pose.position.x = -1.025
    pose_Ground.pose.position.y = -0.975
    pose_Ground.pose.position.z = 0.27
    pose_Ground.pose.orientation.w = 1.0

    pose_box = PoseStamped()
    pose_box.header.frame_id = REFERENCE_FRAME
    pose_box.pose.position.x = -1.075
    pose_box.pose.position.y = -1.05
    pose_box.pose.position.z = 0.65
    pose_box.pose.orientation.w = 1.0

    # Remove leftover objects from a previous run

    scene.remove_world_object('ground')
    scene.remove_world_object('UR3_support')

    # scene.add_plane(Ground_id, pose_Ground, normal=(0,0,1), offset=0.1)
    scene.add_box("ground", pose_Ground, (0.8, 1.85, 0.54))
    # scene.add_box("UR3_support", pose_Ground, (0.2, 0.2, 0.05))
    scene.add_box("box", pose_box, (0.25, 0.5, 0.125)) #0.450 on y without extremities
    

    scene_pub.publish(PlanningScene)

def UR3_setup():
    
    # Initialize the move group for the two UR3
    arm1 = moveit_commander.MoveGroupCommander('UR3_1_manipulator')
    arm2 = moveit_commander.MoveGroupCommander('UR3_2_manipulator')
            
    # Get the name of the end-effector link
    end_effector_link_UR3_1 = arm1.get_end_effector_link()
    rospy.loginfo("End effector link UR3_1: %s", end_effector_link_UR3_1)

    end_effector_link_UR3_2 = arm2.get_end_effector_link()
    rospy.loginfo("End effector link UR3_2: %s", end_effector_link_UR3_2)
                    
    # Set the reference frame for pose targets
    reference_frame_UR3_1 = '/base_link'
    reference_frame_UR3_2 = '/UR3_2_base_link'
    
    # Set the two UR3 reference frame accordingly
    arm1.set_pose_reference_frame(reference_frame_UR3_1)
    arm2.set_pose_reference_frame(reference_frame_UR3_2)
            
    # Allow replanning to increase the odds of a solution
    arm1.allow_replanning(True)
    arm2.allow_replanning(True)
    
    # Allow some leeway in position (meters) and orientation (radians)
    arm1.set_goal_position_tolerance(0.01) # Changed from 0.01 to 0.05
    arm1.set_goal_orientation_tolerance(0.01)

    arm2.set_goal_position_tolerance(0.01) # Changed from 0.01 to 0.05
    arm2.set_goal_orientation_tolerance(0.01)

    # Set velocity scaling factor
    arm1.set_max_velocity_scaling_factor(0.3)
    arm1.set_max_acceleration_scaling_factor(0.3)

    arm2.set_max_velocity_scaling_factor(0.3)
    arm2.set_max_acceleration_scaling_factor(0.3)

    default_joint_states_UR3_1 = arm1.get_current_joint_values()
    default_joint_states_UR3_2 = arm2.get_current_joint_values()

    # Gripper TOP view for GRASPING OBJECT - UR3 1
    default_joint_states_UR3_1[0] = 1.576927900314331
    default_joint_states_UR3_1[1] = -1.539525032043457
    default_joint_states_UR3_1[2] = 1.1562254428863525
    default_joint_states_UR3_1[3] = -1.1815946102142334
    default_joint_states_UR3_1[4] = -1.570829153060913
    default_joint_states_UR3_1[5] = -3.1213247776031494

    # Test 1 (gripper TOP view) - UR3 2
    default_joint_states_UR3_2[0] = 1.576927900314331
    default_joint_states_UR3_2[1] = -1.539525032043457
    default_joint_states_UR3_2[2] = 1.1562254428863525
    default_joint_states_UR3_2[3] = -1.1815946102142334
    default_joint_states_UR3_2[4] = -1.5707422494888306
    default_joint_states_UR3_2[5] = -3.1272690296173096

    # Initial position
    arm1.set_joint_value_target(default_joint_states_UR3_1)
    arm2.set_joint_value_target(default_joint_states_UR3_2)

    # Set the internal state to the current state
    arm1.set_start_state_to_current_state()
    plan_UR3_1 = arm1.plan()

    arm2.set_start_state_to_current_state()
    plan_UR3_2 = arm2.plan()

    # print('plan UR3_1:', plan_UR3_1.joint_trajectory)

    # arm1.execute(plan_UR3_1, wait=False)
    # arm2.execute(plan_UR3_2, wait=False)

    # Trick to allow synchronous movement of the two UR3
    # Publishes directly to controllers the MoveIt! plan 
    trajectory_UR3_1_pub.publish(plan_UR3_1.joint_trajectory)
    trajectory_UR3_2_pub.publish(plan_UR3_2.joint_trajectory)

def UR3_motionPlanning():

    # Activate vision sensor
    activeCamera_UR3_1_pub.publish(True)

    # Request object
    objectRequested_pub.publish('Cylinder')

    # Pause
    rospy.sleep(1)

    # Camera stuff
    camera_model = image_geometry.PinholeCameraModel()
    camera_model.fromCameraInfo(camera_info)

    # Debug
    rospy.loginfo("Camera parameters retrieved correctly")

    # Initialize the move_group API
    moveit_commander.roscpp_initialize(sys.argv)
    
    # Initialize the move group for the right arm
    arm = moveit_commander.MoveGroupCommander('UR3_1_manipulator')
            
    # Get the name of the end-effector link
    end_effector_link = arm.get_end_effector_link()
    rospy.loginfo("End effector link: %s", end_effector_link)
                    
    # Set the reference frame for pose targets
    reference_frame = '/base_link'
    
    # Set the right arm reference frame accordingly
    arm.set_pose_reference_frame(reference_frame)
            
    # Allow replanning to increase the odds of a solution
    arm.allow_replanning(True)
    
    # Allow some leeway in position (meters) and orientation (radians)
    arm.set_goal_position_tolerance(0.005) # Changed from 0.01 to 0.05
    arm.set_goal_orientation_tolerance(0.005)

    # Set velocity scaling factor
    arm.set_max_velocity_scaling_factor(0.3)
    arm.set_max_acceleration_scaling_factor(0.3)

    # ------------------------

    # CAMERA AND VISION STUFF

    # Assuming that the camera = end effector pose (MAYBE CHANGE THIS)
    camera_x = camera_state_info.position.x
    camera_y = camera_state_info.position.y
    camera_z = camera_state_info.position.z

    print "CAMERA X:", camera_x
    print "CAMERA Y:", camera_y
    print "CAMERA Z:", camera_z

    zoffset = 0.00  # table height in UR3's frame (see setup_notes.txt)
    pixel_size = 0.001246422  # camera calibration (meter/pixels)
    #pixel_size_x = 0.000125  # camera calibration (meter/pixels)
    #pixel_size_y = 0.0000625  # camera calibration (meter/pixels)
    h = camera_z-zoffset  # height from table to camera
    x0 = camera_x  # x camera position
    y0 = camera_y  # y camera position
    x_offset = 0.996  # 0.08 offsets 
    y_offset = 0.190 # 0.07
    height = 800  # image frame dimensions
    width = 800
    cx = object_location[0]
    cy = object_location[1]

    # Convert pixel coordinates to UR3 coordinates
    xb = (cy - (height/2))*pixel_size*h + x0 + x_offset
    yb = (cx - (width/2))*pixel_size*h + y0 + y_offset

    print "Object Location (pixels):", (cx, cy)
    print "Object Location (world):", (xb/10, -yb/10)
    # print "Object Orientation:", list(reversed(object_orientation))

    zsafe = 0.220 # TESTED FOR SIMU UR3 (OK FOR PICKING UP TOO)
    zpick = 0.235  # CHANGE HEIGHT FOR PICKING UP

    # dsafe = [xb, yb, zsafe, 0.99, 0.01, 0.01, 0.01]
    # START POSE -- [-0.114, 0.290, 0.393, -0.502, -0.495, 0.505, -0.498] 
    dsafe = [-0.014, 0.170, zsafe, -0.502, -0.495, 0.505, -0.498] # WORKING, HARD CODED
    # dsafe = [xb, yb, zsafe, -0.707, 0.004, 0.707, 0.005]
    

    print "Sent pose:", dsafe

    # Desactivate vision sensor
    activeCamera_UR3_1_pub.publish(False)

    # ------------------------

    # TEST WITH VISION DETECTION

    # Set the target pose.
    target_pose = PoseStamped()
    target_pose.header.frame_id = reference_frame
    target_pose.header.stamp = rospy.Time.now()   

    # Test for UR3
    target_pose.pose.position.x = dsafe[0]
    target_pose.pose.position.y = dsafe[1]
    target_pose.pose.position.z = dsafe[2]

    target_pose.pose.orientation.x = dsafe[3]
    target_pose.pose.orientation.y = dsafe[4]
    target_pose.pose.orientation.z = dsafe[5]
    target_pose.pose.orientation.w = dsafe[6]
    
    # Test for UR3 (home position)
    # target_pose.pose.position.x = 0.456
    # target_pose.pose.position.y = 0.195
    # target_pose.pose.position.z = 0.066

    # target_pose.pose.orientation.x = 0.002
    # target_pose.pose.orientation.y = 0.002
    # target_pose.pose.orientation.z = 0.707
    # target_pose.pose.orientation.w = 0.707


    # Set the start state to the current state
    arm.set_start_state_to_current_state()
    
    # Set the goal pose of the end effector to the stored pose
    arm.set_pose_target(target_pose, end_effector_link)
    
    # Plan the trajectory to the goal
    plan_UR3_1 = arm.plan()
    
    # Execute the planned trajectory
    rospy.logwarn("Executing target pose (IK)")
    
    trajectory_UR3_1_pub.publish(plan_UR3_1.joint_trajectory)
    
    # arm.execute(traj)

    # # Shut down MoveIt cleanly
    # moveit_commander.roscpp_shutdown()
    
    # # Exit MoveIt
    # moveit_commander.os._exit(0)



    # ------------------------

def lift_after_grasping():
    
    # Initialize the move group for the two UR3
    arm1 = moveit_commander.MoveGroupCommander('UR3_1_manipulator')
    arm2 = moveit_commander.MoveGroupCommander('UR3_2_manipulator')
            
    # Get the name of the end-effector link
    end_effector_link_UR3_1 = arm1.get_end_effector_link()
    rospy.loginfo("End effector link UR3_1: %s", end_effector_link_UR3_1)

    end_effector_link_UR3_2 = arm2.get_end_effector_link()
    rospy.loginfo("End effector link UR3_2: %s", end_effector_link_UR3_2)
                    
    # Set the reference frame for pose targets
    reference_frame_UR3_1 = '/base_link'
    reference_frame_UR3_2 = '/UR3_2_base_link'
    
    # Set the two UR3 reference frame accordingly
    arm1.set_pose_reference_frame(reference_frame_UR3_1)
    arm2.set_pose_reference_frame(reference_frame_UR3_2)
            
    # Allow replanning to increase the odds of a solution
    arm1.allow_replanning(True)
    arm2.allow_replanning(True)
    
    # Allow some leeway in position (meters) and orientation (radians)
    arm1.set_goal_position_tolerance(0.01) # Changed from 0.01 to 0.05
    arm1.set_goal_orientation_tolerance(0.01)

    arm2.set_goal_position_tolerance(0.01) # Changed from 0.01 to 0.05
    arm2.set_goal_orientation_tolerance(0.01)

    # Set velocity scaling factor
    arm1.set_max_velocity_scaling_factor(0.3)
    arm1.set_max_acceleration_scaling_factor(0.3)

    arm2.set_max_velocity_scaling_factor(0.3)
    arm2.set_max_acceleration_scaling_factor(0.3)

    default_joint_states_UR3_1 = arm1.get_current_joint_values()
    default_joint_states_UR3_2 = arm2.get_current_joint_values()

    # Gripper TOP view for GRASPING OBJECT - UR3 1
    default_joint_states_UR3_1[0] = 0.9196466207504272
    default_joint_states_UR3_1[1] = -2.010862350463867
    default_joint_states_UR3_1[2] = 1.2049345970153809
    default_joint_states_UR3_1[3] = -0.7549290657043457
    default_joint_states_UR3_1[4] = -1.5760185718536377
    default_joint_states_UR3_1[5] = 2.5080723762512207

    # Test 1 (gripper TOP view) - UR3 2
    # default_joint_states_UR3_2[0] = 0.0
    # default_joint_states_UR3_2[1] = -0.7842642664909363
    # default_joint_states_UR3_2[2] = 0.0
    # default_joint_states_UR3_2[3] = -0.7811948657035828
    # default_joint_states_UR3_2[4] = -1.5707422494888306
    # default_joint_states_UR3_2[5] = -3.1272690296173096

    # Initial position
    arm1.set_joint_value_target(default_joint_states_UR3_1)
    # arm2.set_joint_value_target(default_joint_states_UR3_2)

    # Set the internal state to the current state
    arm1.set_start_state_to_current_state()
    plan_UR3_1 = arm1.plan()

    # arm2.set_start_state_to_current_state()
    # plan_UR3_2 = arm2.plan()

    # print('plan UR3_1:', plan_UR3_1.joint_trajectory)

    # arm1.execute(plan_UR3_1, wait=False)
    # arm2.execute(plan_UR3_2, wait=False)

    # Trick to allow synchronous movement of the two UR3
    # Publishes directly to controllers the MoveIt! plan 
    trajectory_UR3_1_pub.publish(plan_UR3_1.joint_trajectory)
    # trajectory_UR3_2_pub.publish(plan_UR3_2.joint_trajectory)

def safe_position_box():
    
    # Initialize the move group for the two UR3
    arm1 = moveit_commander.MoveGroupCommander('UR3_1_manipulator')
    arm2 = moveit_commander.MoveGroupCommander('UR3_2_manipulator')
            
    # Get the name of the end-effector link
    end_effector_link_UR3_1 = arm1.get_end_effector_link()
    rospy.loginfo("End effector link UR3_1: %s", end_effector_link_UR3_1)

    end_effector_link_UR3_2 = arm2.get_end_effector_link()
    rospy.loginfo("End effector link UR3_2: %s", end_effector_link_UR3_2)
                    
    # Set the reference frame for pose targets
    reference_frame_UR3_1 = '/base_link'
    reference_frame_UR3_2 = '/UR3_2_base_link'
    
    # Set the two UR3 reference frame accordingly
    arm1.set_pose_reference_frame(reference_frame_UR3_1)
    arm2.set_pose_reference_frame(reference_frame_UR3_2)
            
    # Allow replanning to increase the odds of a solution
    arm1.allow_replanning(True)
    arm2.allow_replanning(True)
    
    # Allow some leeway in position (meters) and orientation (radians)
    arm1.set_goal_position_tolerance(0.01) # Changed from 0.01 to 0.05
    arm1.set_goal_orientation_tolerance(0.01)

    arm2.set_goal_position_tolerance(0.01) # Changed from 0.01 to 0.05
    arm2.set_goal_orientation_tolerance(0.01)

    # Set velocity scaling factor
    arm1.set_max_velocity_scaling_factor(0.3)
    arm1.set_max_acceleration_scaling_factor(0.3)

    arm2.set_max_velocity_scaling_factor(0.3)
    arm2.set_max_acceleration_scaling_factor(0.3)

    default_joint_states_UR3_1 = arm1.get_current_joint_values()
    default_joint_states_UR3_2 = arm2.get_current_joint_values()

    # Gripper TOP view for GRASPING OBJECT - UR3 1
    default_joint_states_UR3_1[0] = 3.1272690296173096
    default_joint_states_UR3_1[1] = -2.010862350463867
    default_joint_states_UR3_1[2] = 1.2049345970153809
    default_joint_states_UR3_1[3] = -0.7549290657043457
    default_joint_states_UR3_1[4] = -1.5760185718536377
    default_joint_states_UR3_1[5] = 2.5080723762512207

    # Test 1 (gripper TOP view) - UR3 2
    # default_joint_states_UR3_2[0] = 0.0
    # default_joint_states_UR3_2[1] = -0.7842642664909363
    # default_joint_states_UR3_2[2] = 0.0
    # default_joint_states_UR3_2[3] = -0.7811948657035828
    # default_joint_states_UR3_2[4] = -1.5707422494888306
    # default_joint_states_UR3_2[5] = -3.1272690296173096

    # Initial position
    arm1.set_joint_value_target(default_joint_states_UR3_1)
    # arm2.set_joint_value_target(default_joint_states_UR3_2)

    # Set the internal state to the current state
    arm1.set_start_state_to_current_state()
    plan_UR3_1 = arm1.plan()

    # arm2.set_start_state_to_current_state()
    # plan_UR3_2 = arm2.plan()

    # print('plan UR3_1:', plan_UR3_1.joint_trajectory)

    # arm1.execute(plan_UR3_1, wait=False)
    # arm2.execute(plan_UR3_2, wait=False)

    # Trick to allow synchronous movement of the two UR3
    # Publishes directly to controllers the MoveIt! plan 
    trajectory_UR3_1_pub.publish(plan_UR3_1.joint_trajectory)
    # trajectory_UR3_2_pub.publish(plan_UR3_2.joint_trajectory)

def position_release_1():
    
    # Initialize the move group for the two UR3
    arm1 = moveit_commander.MoveGroupCommander('UR3_1_manipulator')
    arm2 = moveit_commander.MoveGroupCommander('UR3_2_manipulator')
            
    # Get the name of the end-effector link
    end_effector_link_UR3_1 = arm1.get_end_effector_link()
    rospy.loginfo("End effector link UR3_1: %s", end_effector_link_UR3_1)

    end_effector_link_UR3_2 = arm2.get_end_effector_link()
    rospy.loginfo("End effector link UR3_2: %s", end_effector_link_UR3_2)
                    
    # Set the reference frame for pose targets
    reference_frame_UR3_1 = '/base_link'
    reference_frame_UR3_2 = '/UR3_2_base_link'
    
    # Set the two UR3 reference frame accordingly
    arm1.set_pose_reference_frame(reference_frame_UR3_1)
    arm2.set_pose_reference_frame(reference_frame_UR3_2)
            
    # Allow replanning to increase the odds of a solution
    arm1.allow_replanning(True)
    arm2.allow_replanning(True)
    
    # Allow some leeway in position (meters) and orientation (radians)
    arm1.set_goal_position_tolerance(0.01) # Changed from 0.01 to 0.05
    arm1.set_goal_orientation_tolerance(0.01)

    arm2.set_goal_position_tolerance(0.01) # Changed from 0.01 to 0.05
    arm2.set_goal_orientation_tolerance(0.01)

    # Set velocity scaling factor
    arm1.set_max_velocity_scaling_factor(0.3)
    arm1.set_max_acceleration_scaling_factor(0.3)

    arm2.set_max_velocity_scaling_factor(0.3)
    arm2.set_max_acceleration_scaling_factor(0.3)

    default_joint_states_UR3_1 = arm1.get_current_joint_values()
    default_joint_states_UR3_2 = arm2.get_current_joint_values()

    # Gripper TOP view for GRASPING OBJECT - UR3 1
    default_joint_states_UR3_1[0] = 2.872013568878174
    default_joint_states_UR3_1[1] = -0.6862711906433105
    default_joint_states_UR3_1[2] = 0.012467622756958008
    default_joint_states_UR3_1[3] = -0.8892719745635986
    default_joint_states_UR3_1[4] = -1.5806825160980225
    default_joint_states_UR3_1[5] = 2.8955631256103516


    # Test 1 (gripper TOP view) - UR3 2
    # default_joint_states_UR3_2[0] = 0.0
    # default_joint_states_UR3_2[1] = -0.7842642664909363
    # default_joint_states_UR3_2[2] = 0.0
    # default_joint_states_UR3_2[3] = -0.7811948657035828
    # default_joint_states_UR3_2[4] = -1.5707422494888306
    # default_joint_states_UR3_2[5] = -3.1272690296173096

    # Initial position
    arm1.set_joint_value_target(default_joint_states_UR3_1)
    # arm2.set_joint_value_target(default_joint_states_UR3_2)

    # Set the internal state to the current state
    arm1.set_start_state_to_current_state()
    plan_UR3_1 = arm1.plan()

    # arm2.set_start_state_to_current_state()
    # plan_UR3_2 = arm2.plan()

    # print('plan UR3_1:', plan_UR3_1.joint_trajectory)

    # arm1.execute(plan_UR3_1, wait=False)
    # arm2.execute(plan_UR3_2, wait=False)

    # Trick to allow synchronous movement of the two UR3
    # Publishes directly to controllers the MoveIt! plan 
    trajectory_UR3_1_pub.publish(plan_UR3_1.joint_trajectory)
    # trajectory_UR3_2_pub.publish(plan_UR3_2.joint_trajectory)

def pickup_object():
    rospy.loginfo("Grasping object!")
    gripper_UR3_1_pub.publish(True)
    # gripper_UR3_2_pub.publish(True)

def release_object():
    rospy.loginfo("Releasing object!")
    gripper_UR3_1_pub.publish(False)
    # gripper_UR3_2_pub.publish(False)


if __name__ == "__main__":
    rospy.init_node('dual_ur3_getObject')
    
    try:

        # ROS stuff
        rospy.Subscriber("/UR3_1/camera/camera_info", CameraInfo, initCamera)
        rospy.Subscriber("/UR3_1/endpoint_state", Pose, getCameraState)

        rospy.Subscriber('UR3_1/cxy', Tracker, getObjectLocation, queue_size=1) # Position center of object in image

        rospy.Subscriber("/UR3_2/camera/camera_info", CameraInfo, initCamera)
        rospy.Subscriber("/UR3_2/endpoint_state", Pose, getCameraState)

        rospy.Subscriber('UR3_2/cxy', Tracker, getObjectLocation, queue_size=1) # Position center of object in image

        # scene_pub = rospy.Publisher('/move_group/monitored_planning_scene', PlanningScene, queue_size=1)
        gripper_UR3_1_pub = rospy.Publisher('/UR3_1/gripper_command', Bool, queue_size=1)
        gripper_UR3_2_pub = rospy.Publisher('/UR3_2/gripper_command', Bool, queue_size=1)

        trajectory_UR3_1_pub = rospy.Publisher('/UR3_1_controller/command', JointTrajectory, queue_size=10)
        trajectory_UR3_2_pub = rospy.Publisher('/UR3_2_controller/command', JointTrajectory, queue_size=10)

        activeCamera_UR3_1_pub = rospy.Publisher('/UR3_1/camera/camera_active', Bool, queue_size=1)
        activeCamera_UR3_2_pub = rospy.Publisher('/UR3_2/camera/camera_active', Bool, queue_size=1)

        objectRequested_pub = rospy.Publisher('/object_requested', String, queue_size=1)

        rate = rospy.Rate(50)

        # while (camera_info is None):
        #     rate.sleep()

        rospy.logwarn("Initializing MoveIt! planning scene...")

        planning_setup()

        rospy.logwarn("MoveIt! planning scene ready!")

        rospy.loginfo("Moving to setup pose...")

        UR3_setup()
        # up_position()
        # reset_pose()

        # !! MODIFY THIS LOGIC SOMEHOW... like menu/choices or even SMACH !! #

        raw_input("Press Enter to continue and get the object...")

        UR3_motionPlanning()

        raw_input("Press Enter to grasp the object...")

        pickup_object()

        raw_input("Press Enter to continue and lift the object (JointStates command)...")

        lift_after_grasping()

        raw_input("Press Enter to continue and move the object (JointStates command)...")

        safe_position_box()

        raw_input("Press Enter to continue and get ready to release object STEP1 (JointStates command)...")

        position_release_1()

        raw_input("Press Enter to release the object...")

        release_object()

        raw_input("Press Enter to reset the pose...")

        UR3_setup()

        # raw_input("Press Enter to reset the pose...")

        # reset_pose()



        rospy.spin()
    
    except rospy.ROSInterruptException:
        pass