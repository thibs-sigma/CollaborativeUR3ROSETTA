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

import actionlib.msg 

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
    pose_box.pose.position.z = 0.60
    pose_box.pose.orientation.w = 1.0

    # Remove leftover objects from a previous run

    scene.remove_world_object('ground')
    scene.remove_world_object('UR3_support')

    # scene.add_plane(Ground_id, pose_Ground, normal=(0,0,1), offset=0.1)
    scene.add_box("ground", pose_Ground, (0.8, 1.85, 0.54))
    # scene.add_box("UR3_support", pose_Ground, (0.2, 0.2, 0.05))
    scene.add_box("box", pose_box, (0.25, 0.5, 0.125)) #0.450 on y without extremities
    

    scene_pub.publish(PlanningScene)

def erase_box():


    # Use the planning scene object to add or remove objects //Interface
    scene = moveit_commander.PlanningSceneInterface()



    # Remove leftover objects from a previous run

    scene.remove_world_object('box')

 

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

    # Test 1 (gripper TOP view) - UR3 1
    default_joint_states_UR3_1[0] = 3.1272690296173096
    default_joint_states_UR3_1[1] = -0.7842642664909363
    default_joint_states_UR3_1[2] = 0.0
    default_joint_states_UR3_1[3] = -0.7811948657035828
    default_joint_states_UR3_1[4] = -1.5707422494888306
    default_joint_states_UR3_1[5] = -3.1272690296173096

    # Test 1 (gripper TOP view) - UR3 2
    default_joint_states_UR3_2[0] = 0.0
    default_joint_states_UR3_2[1] = -0.7842642664909363
    default_joint_states_UR3_2[2] = 0.0
    default_joint_states_UR3_2[3] = -0.7811948657035828
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

    

def dual_grasping_jointStates():

    # Request object
    objectRequested_pub.publish('BoxAssembly1')

    # Pause
    rospy.sleep(1)
 
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

    # Positioning for grasping box - UR3 1
    default_joint_states_UR3_1[0] = 2.9477999210357666
    default_joint_states_UR3_1[1] = -1.6932964324951172
    default_joint_states_UR3_1[2] = 1.743145227432251
    default_joint_states_UR3_1[3] = -1.6150071620941162
    default_joint_states_UR3_1[4] = -1.5722053050994873
    default_joint_states_UR3_1[5] = 2.9474127292633057

    # Positioning for grasping box - UR3 2
    default_joint_states_UR3_2[0] = -2.6021482944488525
    default_joint_states_UR3_2[1] = -1.538029670715332
    default_joint_states_UR3_2[2] = -1.5700857639312744
    default_joint_states_UR3_2[3] = -1.6108310222625732
    default_joint_states_UR3_2[4] = 1.5747594833374023
    default_joint_states_UR3_2[5] = -2.588318109512329

    # Initial position
    arm1.set_joint_value_target(default_joint_states_UR3_1)
    arm2.set_joint_value_target(default_joint_states_UR3_2)

    # Erase box
    erase_box()

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

    # Shift the end-effector to the right 5cm
    # arm1.shift_pose_target(1, -0.05, end_effector_link_UR3_1)
    # arm1.go()
    # rospy.sleep(1)

def dual_grasping_AdjustjointStates():
 
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

    # Positioning for grasping box - UR3 1
    default_joint_states_UR3_1[0] = 2.9707658290863037
    default_joint_states_UR3_1[1] = -1.4824342727661133
    default_joint_states_UR3_1[2] = 1.79585599899292
    default_joint_states_UR3_1[3] = -1.8676214218139648
    default_joint_states_UR3_1[4] = -1.571993112564087
    default_joint_states_UR3_1[5] = 2.9708669185638428


    # Positioning for grasping box - UR3 2
    default_joint_states_UR3_2[0] = -2.698265790939331
    default_joint_states_UR3_2[1] = -1.7230095863342285
    default_joint_states_UR3_2[2] = -1.749833106994629
    default_joint_states_UR3_2[3] = -1.2479557991027832
    default_joint_states_UR3_2[4] = 1.574021339416504
    default_joint_states_UR3_2[5] = -2.6841063499450684


    # Initial position
    arm1.set_joint_value_target(default_joint_states_UR3_1)
    arm2.set_joint_value_target(default_joint_states_UR3_2)

    # Erase box
    erase_box()

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

    # Shift the end-effector to the right 5cm
    # arm1.shift_pose_target(1, -0.05, end_effector_link_UR3_1)
    # arm1.go()
    # rospy.sleep(1)

def dual_grasping_LiftjointStates():
 
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

    # Positioning for grasping box - UR3 1
    default_joint_states_UR3_1[0] = 2.9601311683654785
    default_joint_states_UR3_1[1] = -1.6071155071258545
    default_joint_states_UR3_1[2] = 1.6323480606079102
    default_joint_states_UR3_1[3] = -1.5840208530426025
    default_joint_states_UR3_1[4] = -1.5726039409637451
    default_joint_states_UR3_1[5] = 2.9597818851470947

    # Positioning for grasping box - UR3 2
    default_joint_states_UR3_2[0] = -2.7149484157562256
    default_joint_states_UR3_2[1] = -1.735840082168579
    default_joint_states_UR3_2[2] = -1.4903714656829834
    default_joint_states_UR3_2[3] = -1.4953033924102783
    default_joint_states_UR3_2[4] = 1.5739901065826416
    default_joint_states_UR3_2[5] = -2.7004878520965576


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

def dual_grasping_MovejointStates():
 
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

    # Positioning for grasping box - UR3 1
    default_joint_states_UR3_1[0] = 2.934258460998535
    default_joint_states_UR3_1[1] = -1.6071155071258545
    default_joint_states_UR3_1[2] = 1.6323480606079102
    default_joint_states_UR3_1[3] = -1.5840208530426025
    default_joint_states_UR3_1[4] = -1.5726039409637451
    default_joint_states_UR3_1[5] = 2.9597818851470947

    # Positioning for grasping box - UR3 2
    default_joint_states_UR3_2[0] = -2.7449484157562256
    default_joint_states_UR3_2[1] = -1.735840082168579
    default_joint_states_UR3_2[2] = -1.4903714656829834
    default_joint_states_UR3_2[3] = -1.4953033924102783
    default_joint_states_UR3_2[4] = 1.5739901065826416
    default_joint_states_UR3_2[5] = -2.7004878520965576

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


def dual_grasping_pose():
    
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
    arm1.set_goal_position_tolerance(0.02) # Changed from 0.01 to 0.05
    arm1.set_goal_orientation_tolerance(0.02)

    arm2.set_goal_position_tolerance(0.02) # Changed from 0.01 to 0.05
    arm2.set_goal_orientation_tolerance(0.02)

    # Set velocity scaling factor
    arm1.set_max_velocity_scaling_factor(0.3)
    arm1.set_max_acceleration_scaling_factor(0.3)

    arm2.set_max_velocity_scaling_factor(0.3)
    arm2.set_max_acceleration_scaling_factor(0.3)

    default_joint_states_UR3_1 = arm1.get_current_joint_values()
    default_joint_states_UR3_2 = arm2.get_current_joint_values()

    # Set the target pose.
    target_pose_UR3_1 = PoseStamped()
    target_pose_UR3_2 = PoseStamped()

    target_pose_UR3_1.header.frame_id = reference_frame_UR3_1
    target_pose_UR3_2.header.frame_id = reference_frame_UR3_2

    target_pose_UR3_1.header.stamp = rospy.Time.now() 
    target_pose_UR3_2.header.stamp = rospy.Time.now() 

    # Positioning for grasping box - UR3 1
    target_pose_UR3_1.pose.position.x = -0.041
    target_pose_UR3_1.pose.position.y = 0.279
    target_pose_UR3_1.pose.position.z = 0.216

    target_pose_UR3_1.pose.orientation.x = 0.506
    target_pose_UR3_1.pose.orientation.y = 0.490
    target_pose_UR3_1.pose.orientation.z = -0.501
    target_pose_UR3_1.pose.orientation.w = 0.502


    # Positioning for grasping box - UR3 2
    target_pose_UR3_2.pose.position.x = 0.305
    target_pose_UR3_2.pose.position.y = 0.038
    target_pose_UR3_2.pose.position.z = 0.225

    target_pose_UR3_2.pose.orientation.x = -0.705
    target_pose_UR3_2.pose.orientation.y = 0.005
    target_pose_UR3_2.pose.orientation.z = 0.709
    target_pose_UR3_2.pose.orientation.w = 0.005

    # Set the start state to the current state
    arm1.set_start_state_to_current_state()
    arm2.set_start_state_to_current_state()
    
    # Set the goal pose of the end effector to the stored pose
    arm1.set_pose_target(target_pose_UR3_1, end_effector_link_UR3_1)
    arm2.set_pose_target(target_pose_UR3_2, end_effector_link_UR3_2)

    # Plan the trajectories
    plan_UR3_1 = arm1.plan()
    plan_UR3_2 = arm2.plan()

    # print('plan UR3_1:', plan_UR3_1.joint_trajectory)

    # arm1.execute(plan_UR3_1, wait=False)
    # arm2.execute(plan_UR3_2, wait=False)

    # Trick to allow synchronous movement of the two UR3
    # Publishes directly to controllers the MoveIt! plan 
    trajectory_UR3_1_pub.publish(plan_UR3_1.joint_trajectory)
    trajectory_UR3_2_pub.publish(plan_UR3_2.joint_trajectory)

def dual_grasping_Adjustpose():
    
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
    arm1.set_goal_position_tolerance(0.02) # Changed from 0.01 to 0.05
    arm1.set_goal_orientation_tolerance(0.02)

    arm2.set_goal_position_tolerance(0.02) # Changed from 0.01 to 0.05
    arm2.set_goal_orientation_tolerance(0.02)

    # Set velocity scaling factor
    arm1.set_max_velocity_scaling_factor(0.3)
    arm1.set_max_acceleration_scaling_factor(0.3)

    arm2.set_max_velocity_scaling_factor(0.3)
    arm2.set_max_acceleration_scaling_factor(0.3)

    default_joint_states_UR3_1 = arm1.get_current_joint_values()
    default_joint_states_UR3_2 = arm2.get_current_joint_values()

    # Set the target pose.
    target_pose_UR3_1 = PoseStamped()
    target_pose_UR3_2 = PoseStamped()

    target_pose_UR3_1.header.frame_id = reference_frame_UR3_1
    target_pose_UR3_2.header.frame_id = reference_frame_UR3_2

    target_pose_UR3_1.header.stamp = rospy.Time.now() 
    target_pose_UR3_2.header.stamp = rospy.Time.now() 

    # Set the start state to the current state
    arm1.set_start_state_to_current_state()
    arm2.set_start_state_to_current_state()
    
    # Set the goal pose of the end effector to the stored pose
    rospy.logwarn("Manual shifting")

    arm1.shift_pose_target(2, 0.05, end_effector_link_UR3_1) # Go down on Z axis
    arm2.shift_pose_target(2, 0.05, end_effector_link_UR3_1) # Go down on Z axis


    # Plan the trajectories
    plan_UR3_1 = arm1.plan()
    plan_UR3_2 = arm2.plan()

    # print('plan UR3_1:', plan_UR3_1.joint_trajectory)

    # arm1.execute(plan_UR3_1, wait=False)
    # arm2.execute(plan_UR3_2, wait=False)

    # Trick to allow synchronous movement of the two UR3
    # Publishes directly to controllers the MoveIt! plan 
    trajectory_UR3_1_pub.publish(plan_UR3_1.joint_trajectory)
    trajectory_UR3_2_pub.publish(plan_UR3_2.joint_trajectory)


def pickup_object():
    rospy.loginfo("Grasping object!")
    gripper_UR3_1_pub.publish(True)
    gripper_UR3_2_pub.publish(True)

def release_object():
    rospy.loginfo("Releasing object!")
    gripper_UR3_1_pub.publish(False)
    gripper_UR3_2_pub.publish(False)


if __name__ == "__main__":
    rospy.init_node('dual_ur3_motionPlanning_sync')
    
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

        raw_input("Press Enter to continue and position dual arm for grasping box (JointStates command)...")

        dual_grasping_jointStates()

        raw_input("Press Enter to continue and adjust the pose for grasping box (JointStates command)...")

        dual_grasping_AdjustjointStates()

        # raw_input("Press Enter to continue and position dual arm for grasping box (Pose command)...")

        # dual_grasping_pose()

        # raw_input("Press Enter to continue and adjust the pose for grasping box (Pose command)...")

        # dual_grasping_Adjustpose()

        raw_input("Press Enter to grasp the object...")

        pickup_object()

        raw_input("Press Enter to continue and lift the box (JointStates command)...")

        dual_grasping_LiftjointStates()

        # raw_input("Press Enter to continue and move the box (JointStates command)...")

        # dual_grasping_MovejointStates()

        # raw_input("Press Enter to continue and place back the box on table STEP1 (JointStates command)...")

        # dual_grasping_LiftjointStates()

        raw_input("Press Enter to continue and place back the box on table STEP2 (JointStates command)...")

        dual_grasping_AdjustjointStates()

        raw_input("Press Enter to release the object...")

        release_object()

        raw_input("Press Enter to reset the pose...")
        dual_grasping_jointStates()

        rospy.sleep(1)
        
        planning_setup()
        print ("Box republished in scene...")
        
        rospy.sleep(1)
        
        UR3_setup()

        # raw_input("Press Enter to reset the pose...")

        # reset_pose()



        rospy.spin()
    
    except rospy.ROSInterruptException:
        pass