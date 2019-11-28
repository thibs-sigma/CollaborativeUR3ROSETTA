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

camera_info_UR3_1 = None
camera_state_info_UR3_1 = None
object_location_UR3_1 = None

camera_info_UR3_2 = None
camera_state_info_UR3_2 = None
object_location_UR3_2 = None

# class UR3_motionPlanning:

def initCamera_UR3_1(data):
    global camera_info_UR3_1
    camera_info_UR3_1 = data
    # print(camera_info_UR3_1)

def getCameraState_UR3_1(data):
    global camera_state_info_UR3_1
    camera_state_info_UR3_1 = data

def getObjectLocation_UR3_1(data):
    global object_location_UR3_1
    # global object_orientation
    # global object_name
    # global desired_object
    # if desired_object is not None and desired_object in data.names:
    # i = data.names.index(desired_object)
    # object_name = data.names[i]
    x = data.x
    y = data.y
    object_location_UR3_1 = [x, y]
    # object_orientation = tf.transformations.quaternion_from_euler(
    #     0, 0, -data.theta[i])
    # UR3_motionPlanning()

def initCamera_UR3_2(data):
    global camera_info_UR3_2
    camera_info_UR3_2 = data
    # print(camera_info)

def getCameraState_UR3_2(data):
    global camera_state_info_UR3_2
    camera_state_info_UR3_2 = data

def getObjectLocation_UR3_2(data):
    global object_location_UR3_2
    # global object_orientation
    # global object_name
    # global desired_object
    # if desired_object is not None and desired_object in data.names:
    # i = data.names.index(desired_object)
    # object_name = data.names[i]
    x = data.x
    y = data.y
    object_location_UR3_2 = [x, y]
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
    activeCamera_UR3_2_pub.publish(True)

    # Request object
    objectRequested_pub.publish('Cylinder')

    # Pause
    rospy.sleep(1)

    # Camera stuff
    camera_model = image_geometry.PinholeCameraModel()
    camera_model.fromCameraInfo(camera_info_UR3_1)

    # Debug
    rospy.loginfo("Camera parameters retrieved correctly")

    # Initialize the move_group API
    moveit_commander.roscpp_initialize(sys.argv)
    
    # Initialize the move group for the right arm
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

    # ------------------------

    # CAMERA AND VISION STUFF - UR3_1

    # Assuming that the camera = end effector pose (MAYBE CHANGE THIS)
    camera_x_UR3_1 = camera_state_info_UR3_1.position.x
    camera_y_UR3_1 = camera_state_info_UR3_1.position.y
    camera_z_UR3_1 = camera_state_info_UR3_1.position.z

    print "CAMERA X:", camera_x_UR3_1
    print "CAMERA Y:", camera_y_UR3_1
    print "CAMERA Z:", camera_z_UR3_1

    zoffset_UR3_1 = 0.00  # table height in UR3's frame (see setup_notes.txt)
    pixel_size_UR3_1 = 0.001246422  # camera calibration (meter/pixels)
    #pixel_size_x_UR3_1 = 0.000125  # camera calibration (meter/pixels)
    #pixel_size_y_UR3_1 = 0.0000625  # camera calibration (meter/pixels)
    h_UR3_1 = camera_z_UR3_1-zoffset_UR3_1  # height from table to camera
    x0_UR3_1 = camera_x_UR3_1  # x camera position
    y0_UR3_1 = camera_y_UR3_1  # y camera position
    x_offset_UR3_1 = 0.996  # 0.08 offsets 
    y_offset_UR3_1 = 0.190 # 0.07
    height_UR3_1 = 800  # image frame dimensions
    width_UR3_1 = 800
    cx_UR3_1 = object_location_UR3_1[0]
    cy_UR3_1 = object_location_UR3_1[1]

    # Convert pixel coordinates to UR3 coordinates
    xb_UR3_1 = (cy_UR3_1 - (height_UR3_1/2))*pixel_size_UR3_1*h_UR3_1 + x0_UR3_1 + x_offset_UR3_1
    yb_UR3_1 = (cx_UR3_1 - (width_UR3_1/2))*pixel_size_UR3_1*h_UR3_1 + y0_UR3_1 + y_offset_UR3_1

    print "Object Location (pixels):", (cx_UR3_1, cy_UR3_1)
    print "Object Location (world):", (xb_UR3_1/10, -yb_UR3_1/10)
    # print "Object Orientation:", list(reversed(object_orientation))

    zsafe_UR3_1 = 0.220 # TESTED FOR SIMU UR3 (OK FOR PICKING UP TOO)
    zpick_UR3_1 = 0.235  # CHANGE HEIGHT FOR PICKING UP

    # dsafe = [xb, yb, zsafe, 0.99, 0.01, 0.01, 0.01]
    # START POSE -- [-0.114, 0.290, 0.393, -0.502, -0.495, 0.505, -0.498] 
    dsafe_UR3_1 = [-0.014, 0.170, zsafe_UR3_1, -0.502, -0.495, 0.505, -0.498] # WORKING, HARD CODED
    # dsafe = [xb, yb, zsafe, -0.707, 0.004, 0.707, 0.005]
    

    print "Sent pose:", dsafe_UR3_1

    # Desactivate vision sensor
    activeCamera_UR3_1_pub.publish(False)

    # Exit vision node
    UR3_1_ismoving_pub.publish(True)

    rospy.sleep(1)

    # ------------------------

    # ------------------------

    # CAMERA AND VISION STUFF - UR3_2

    # Assuming that the camera = end effector pose (MAYBE CHANGE THIS)
    camera_x_UR3_2 = camera_state_info_UR3_2.position.x
    camera_y_UR3_2 = camera_state_info_UR3_2.position.y
    camera_z_UR3_2 = camera_state_info_UR3_2.position.z

    print "CAMERA X:", camera_x_UR3_2
    print "CAMERA Y:", camera_y_UR3_2
    print "CAMERA Z:", camera_z_UR3_2

    zoffset_UR3_2 = 0.00  # table height in UR3's frame (see setup_notes.txt)
    pixel_size_UR3_2 = 0.001246422  # camera calibration (meter/pixels)
    #pixel_size_x_UR3_2 = 0.000125  # camera calibration (meter/pixels)
    #pixel_size_y_UR3_2 = 0.0000625  # camera calibration (meter/pixels)
    h_UR3_2 = camera_z_UR3_2-zoffset_UR3_2  # height from table to camera
    x0_UR3_2 = camera_x_UR3_2  # x camera position
    y0_UR3_2 = camera_y_UR3_2  # y camera position
    x_offset_UR3_2 = 0.996  # 0.08 offsets 
    y_offset_UR3_2 = 0.190 # 0.07
    height_UR3_2 = 800  # image frame dimensions
    width_UR3_2 = 800
    cx_UR3_2 = object_location_UR3_2[0]
    cy_UR3_2 = object_location_UR3_2[1]

    # Convert pixel coordinates to UR3 coordinates
    xb_UR3_2 = (cy_UR3_2 - (height_UR3_2/2))*pixel_size_UR3_2*h_UR3_2 + x0_UR3_2 + x_offset_UR3_2
    yb_UR3_2 = (cx_UR3_2 - (width_UR3_2/2))*pixel_size_UR3_2*h_UR3_2 + y0_UR3_2 + y_offset_UR3_2

    print "Object Location (pixels):", (cx_UR3_2, cy_UR3_2)
    print "Object Location (world):", (xb_UR3_2/10, -yb_UR3_2/10)
    # print "Object Orientation:", list(reversed(object_orientation))

    zsafe_UR3_2 = 0.220 # TESTED FOR SIMU UR3 (OK FOR PICKING UP TOO)
    zpick_UR3_2 = 0.235  # CHANGE HEIGHT FOR PICKING UP

    # dsafe = [xb, yb, zsafe, 0.99, 0.01, 0.01, 0.01]
    # START POSE -- [-0.114, 0.290, 0.393, -0.502, -0.495, 0.505, -0.498] 
    dsafe_UR3_2 = [-0.014, 0.170, zsafe_UR3_2, -0.502, -0.495, 0.505, -0.498] # WORKING, HARD CODED
    # dsafe = [xb, yb, zsafe, -0.707, 0.004, 0.707, 0.005]
    

    print "Sent pose:", dsafe_UR3_2

    # Desactivate vision sensor
    activeCamera_UR3_2_pub.publish(False)

    # Exit UR3 2 vision node
    UR3_2_ismoving_pub.publish(True)

    rospy.sleep(1)

    # ------------------------

    # TEST WITH VISION DETECTION

    # Set the target pose UR3_1
    target_pose_UR3_1 = PoseStamped()
    target_pose_UR3_1.header.frame_id = reference_frame_UR3_1
    target_pose_UR3_1.header.stamp = rospy.Time.now() 

    # Set the target pose UR3_2
    target_pose_UR3_2 = PoseStamped()
    target_pose_UR3_2.header.frame_id = reference_frame_UR3_2
    target_pose_UR3_2.header.stamp = rospy.Time.now()   

    # Pose UR3_1
    target_pose_UR3_1.pose.position.x = dsafe_UR3_1[0]
    target_pose_UR3_1.pose.position.y = dsafe_UR3_1[1]
    target_pose_UR3_1.pose.position.z = dsafe_UR3_1[2]

    target_pose_UR3_1.pose.orientation.x = dsafe_UR3_1[3]
    target_pose_UR3_1.pose.orientation.y = dsafe_UR3_1[4]
    target_pose_UR3_1.pose.orientation.z = dsafe_UR3_1[5]
    target_pose_UR3_1.pose.orientation.w = dsafe_UR3_1[6]

    # Pose UR3_2
    target_pose_UR3_2.pose.position.x = dsafe_UR3_2[0]
    target_pose_UR3_2.pose.position.y = dsafe_UR3_2[1]
    target_pose_UR3_2.pose.position.z = dsafe_UR3_2[2]

    target_pose_UR3_2.pose.orientation.x = dsafe_UR3_2[3]
    target_pose_UR3_2.pose.orientation.y = dsafe_UR3_2[4]
    target_pose_UR3_2.pose.orientation.z = dsafe_UR3_2[5]
    target_pose_UR3_2.pose.orientation.w = dsafe_UR3_2[6]
    

    # Set the start state to the current state
    arm1.set_start_state_to_current_state()
    arm2.set_start_state_to_current_state()
    
    # Set the goal pose of the end effector to the stored pose
    arm1.set_pose_target(target_pose_UR3_1, end_effector_link_UR3_1)
    arm2.set_pose_target(target_pose_UR3_2, end_effector_link_UR3_2)
    
    # Plan the trajectory to the goal
    plan_UR3_1 = arm1.plan()
    plan_UR3_2 = arm2.plan()
    
    # Execute the planned trajectory
    rospy.logwarn("Executing target pose UR3_1 (IK)")
    rospy.logwarn("Executing target pose UR3_2 (IK)")
    
    trajectory_UR3_1_pub.publish(plan_UR3_1.joint_trajectory)
    trajectory_UR3_2_pub.publish(plan_UR3_2.joint_trajectory)
    
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

    # Gripper TOP view for LIFTING OBJECT - UR3 1
    default_joint_states_UR3_1[0] = 0.9196466207504272
    default_joint_states_UR3_1[1] = -2.010862350463867
    default_joint_states_UR3_1[2] = 1.2049345970153809
    default_joint_states_UR3_1[3] = -0.7549290657043457
    default_joint_states_UR3_1[4] = -1.5760185718536377
    default_joint_states_UR3_1[5] = 2.5080723762512207

    # Gripper TOP view for LIFTING OBJECT - UR3 2
    default_joint_states_UR3_2[0] = 0.9196466207504272
    default_joint_states_UR3_2[1] = -2.010862350463867
    default_joint_states_UR3_2[2] = 1.2049345970153809
    default_joint_states_UR3_2[3] = -0.7549290657043457
    default_joint_states_UR3_2[4] = -1.5760185718536377
    default_joint_states_UR3_2[5] = 2.5080723762512207

    # Initial position
    arm1.set_joint_value_target(default_joint_states_UR3_1)
    arm2.set_joint_value_target(default_joint_states_UR3_2)

    # Set the internal state to the current state
    arm1.set_start_state_to_current_state()
    plan_UR3_1 = arm1.plan()

    # arm2.set_start_state_to_current_state()
    arm2.set_start_state_to_current_state()
    plan_UR3_2 = arm2.plan()

    # print('plan UR3_1:', plan_UR3_1.joint_trajectory)

    arm1.execute(plan_UR3_1, wait=False)
    arm2.execute(plan_UR3_2, wait=False)

    # Trick to allow synchronous movement of the two UR3
    # Publishes directly to controllers the MoveIt! plan 
    trajectory_UR3_1_pub.publish(plan_UR3_1.joint_trajectory)
    trajectory_UR3_2_pub.publish(plan_UR3_2.joint_trajectory)

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
    default_joint_states_UR3_2[0] = 0.0
    default_joint_states_UR3_2[1] = -2.010862350463867
    default_joint_states_UR3_2[2] = 1.2049345970153809
    default_joint_states_UR3_2[3] = -0.7549290657043457
    default_joint_states_UR3_2[4] = -1.5760185718536377
    default_joint_states_UR3_2[5] = 2.5080723762512207

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
    default_joint_states_UR3_2[0] = -0.255255461
    default_joint_states_UR3_2[1] = -0.6862711906433105
    default_joint_states_UR3_2[2] = 0.012467622756958008
    default_joint_states_UR3_2[3] = -0.8892719745635986
    default_joint_states_UR3_2[4] = -1.5806825160980225
    default_joint_states_UR3_2[5] = 2.8955631256103516

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

def grasp_object():
    rospy.loginfo("Grasping object!")
    gripper_UR3_1_pub.publish(True)
    gripper_UR3_2_pub.publish(True)

def release_object():
    rospy.loginfo("Releasing object!")
    gripper_UR3_1_pub.publish(False)
    gripper_UR3_2_pub.publish(False)


if __name__ == "__main__":
    rospy.init_node('dual_ur3_getObject')
    
    try:

        # ROS stuff
        rospy.Subscriber("/UR3_1/camera/camera_info", CameraInfo, initCamera_UR3_1)
        rospy.Subscriber("/UR3_1/endpoint_state", Pose, getCameraState_UR3_1)

        rospy.Subscriber('UR3_1/cxy', Tracker, getObjectLocation_UR3_1, queue_size=1) # Position center of object in image

        rospy.Subscriber("/UR3_2/camera/camera_info", CameraInfo, initCamera_UR3_2)
        rospy.Subscriber("/UR3_2/endpoint_state", Pose, getCameraState_UR3_2)

        rospy.Subscriber('UR3_2/cxy', Tracker, getObjectLocation_UR3_2, queue_size=1) # Position center of object in image

        # scene_pub = rospy.Publisher('/move_group/monitored_planning_scene', PlanningScene, queue_size=1)
        gripper_UR3_1_pub = rospy.Publisher('/UR3_1/gripper_command', Bool, queue_size=1)
        gripper_UR3_2_pub = rospy.Publisher('/UR3_2/gripper_command', Bool, queue_size=1)

        trajectory_UR3_1_pub = rospy.Publisher('/UR3_1_controller/command', JointTrajectory, queue_size=10)
        trajectory_UR3_2_pub = rospy.Publisher('/UR3_2_controller/command', JointTrajectory, queue_size=10)

        activeCamera_UR3_1_pub = rospy.Publisher('/UR3_1/camera/camera_active', Bool, queue_size=1)
        activeCamera_UR3_2_pub = rospy.Publisher('/UR3_2/camera/camera_active', Bool, queue_size=1)

        objectRequested_pub = rospy.Publisher('/object_requested', String, queue_size=1)

        UR3_1_ismoving_pub = rospy.Publisher('/UR3_1_is_moving', Bool, queue_size=1)
        UR3_2_ismoving_pub = rospy.Publisher('/UR3_2_is_moving', Bool, queue_size=1)

        rate = rospy.Rate(50)

        # while (camera_info is None):
        #     rate.sleep()

        rospy.logwarn("Initializing MoveIt! planning scene...")

        planning_setup()

        rospy.logwarn("MoveIt! planning scene ready!")

        rospy.logwarn("Moving to setup pose...")

        UR3_setup()

        # Wait for motion to complete
        rospy.sleep(10)

        # In addition to logwarn, publish to a topic for interface output then #

        rospy.logwarn("Moving to get object pose...")

        UR3_motionPlanning()

        # Wait for motion to complete
        rospy.sleep(10)

        rospy.logwarn("Grasping the object...")

        grasp_object()

        # Wait for motion to complete
        rospy.sleep(10)

        rospy.logwarn("Lifting the object...")

        lift_after_grasping()

        # Wait for motion to complete
        rospy.sleep(10)

        rospy.logwarn("Moving the object...")

        safe_position_box()

        # Wait for motion to complete
        rospy.sleep(10)

        rospy.logwarn("Moving to pre-release pose...")

        position_release_1()

        # Wait for motion to complete
        rospy.sleep(10)

        rospy.logwarn("Releasing the object...")

        release_object()

        # Wait for motion to complete
        rospy.sleep(10)

        rospy.logwarn("Reseting the pose...")

        UR3_setup()

        # Wait for motion to complete
        rospy.sleep(10)

        # raw_input("Press Enter to reset the pose...")

        rospy.signal_shutdown("Get object OK")

        # reset_pose()

        rospy.spin()
    
    except rospy.ROSInterruptException:
        pass