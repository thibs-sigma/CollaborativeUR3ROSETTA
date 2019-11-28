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
from std_msgs.msg import Bool
from moveit_msgs.msg import RobotTrajectory, PlanningScene
from trajectory_msgs.msg import JointTrajectoryPoint

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

def UR3_motionPlanning():

    # Camera stuff
    camera_model = image_geometry.PinholeCameraModel()
    camera_model.fromCameraInfo(camera_info)

    # Debug
    rospy.loginfo("Camera parameters retrieved correctly")

    # Initialize the move_group API
    moveit_commander.roscpp_initialize(sys.argv)
    
    # Initialize the move group for the right arm
    arm = moveit_commander.MoveGroupCommander('manipulator')
            
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
    arm.set_goal_position_tolerance(0.01) # Changed from 0.01 to 0.05
    arm.set_goal_orientation_tolerance(0.01)

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
    pixel_size = .0001  # camera calibration (meter/pixels)
    #pixel_size_x = 0.000125  # camera calibration (meter/pixels)
    #pixel_size_y = 0.0000625  # camera calibration (meter/pixels)
    h = camera_z-zoffset  # height from table to camera
    x0 = camera_x  # x camera position
    y0 = camera_y  # y camera position
    x_offset = -0.08  # 0.08 offsets 
    y_offset = -0.08 # 0.07
    height = 800  # image frame dimensions
    width = 800
    cx = object_location[0]
    cy = object_location[1]

    # Convert pixel coordinates to UR3 coordinates
    xb = (cy - (height/2))*pixel_size*h + x0 + x_offset
    yb = (cx - (width/2))*pixel_size*h + y0 + y_offset

    print "Object Location (pixels):", (cx, cy)
    print "Object Location (world):", (xb, yb)
    # print "Object Orientation:", list(reversed(object_orientation))

    zsafe = 0.100 # FIRST TEST VALUE UR3
    zpick = -0.155  # CHANGE HEIGHT FOR PICKING UP

    # dsafe = [xb, yb, zsafe, 0.99, 0.01, 0.01, 0.01]
    # dsafe = [0.450, 0.180, zsafe, -0.479, 0.514, 0.505, 0.501] # WORKING
    dsafe = [xb, yb, zsafe, 0.002, 0.002, 0.707, 0.707]

    print "Sent pose:", dsafe

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
    traj = arm.plan()
    
    # Execute the planned trajectory
    rospy.logwarn("Executing target pose (IK)")
    arm.execute(traj)

    # Pause for a second
    rospy.logwarn("Waiting 3 sec")
    rospy.sleep(3)
        
    # Shift the end-effector to the front 5cm
    rospy.logwarn("Manual shifting") 
    arm.shift_pose_target(0, 0.05, end_effector_link) # Center on X 
    arm.go()

    rospy.sleep(2)

    arm.shift_pose_target(1, 0.1, end_effector_link) # Front on Y 
    # arm.shift_pose_target(2, 0.10, end_effector_link) # Down on Z
    arm.go()
    # rospy.sleep(2)
    # arm.shift_pose_target(0, -0.05, end_effector_link) # X axis
    # arm.go()
    # rospy.logwarn("Waiting 5 sec")
    # rospy.sleep(5)

    # # Shut down MoveIt cleanly
    # moveit_commander.roscpp_shutdown()
    
    # # Exit MoveIt
    # moveit_commander.os._exit(0)



    # ------------------------

def UR3_preDeterminedMotion():
    # Initialize the move_group API
    moveit_commander.roscpp_initialize(sys.argv)

    
    # Initialize the move group for the right arm
    arm = moveit_commander.MoveGroupCommander('manipulator')
            
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
    arm.set_goal_position_tolerance(0.05) # Changed from 0.01 to 0.05
    arm.set_goal_orientation_tolerance(0.05)

    # Set velocity scaling factor
    arm.set_max_velocity_scaling_factor(0.3)
    arm.set_max_acceleration_scaling_factor(0.3)

    # MOVING STUFF
    
    # Start the arm in the "home" pose stored in the SRDF file
    # rospy.logwarn("Going to up position...")
    # arm.set_named_target('up')
    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    # arm.go()

    rospy.loginfo(arm.get_current_pose(end_effector_link))
    rospy.logwarn("Waiting 5 sec")
    rospy.sleep(5)            
            
    # Set the target pose.  This particular pose has the gripper oriented horizontally
    # 0.85 meters above the ground, 0.10 meters to the right and 0.20 meters ahead of 
    # the center of the robot base.
    target_pose = PoseStamped()
    target_pose.header.frame_id = reference_frame
    target_pose.header.stamp = rospy.Time.now()   

    # Original  
    # target_pose.pose.position.x = 0.20
    # target_pose.pose.position.y = -0.1
    # target_pose.pose.position.z = 0.85
    # target_pose.pose.orientation.x = 0.0
    # target_pose.pose.orientation.y = 0.0
    # target_pose.pose.orientation.z = 0.0
    # target_pose.pose.orientation.w = 1.0

    # Test for UR3
    target_pose.pose.position.x = 0.464
    target_pose.pose.position.y = 0.180
    target_pose.pose.position.z = 0.281

    target_pose.pose.orientation.x = -0.479
    target_pose.pose.orientation.y = 0.514
    target_pose.pose.orientation.z = 0.505
    target_pose.pose.orientation.w = 0.501



    
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
    traj = arm.plan()
    
    # Execute the planned trajectory
    rospy.logwarn("Executing target pose (IK)")
    arm.execute(traj)

    # Pause for a second
    rospy.logwarn("Waiting 5 sec")
    rospy.sleep(5)
        
    # # Shift the end-effector to the right 5cm
    # arm.shift_pose_target(1, -0.05, end_effector_link)
    # arm.go()
    # rospy.sleep(1)

    # # Rotate the end-effector 90 degrees
    # arm.shift_pose_target(3, -1.57, end_effector_link)
    # arm.go()
    # rospy.sleep(1)
        
    # # Store this pose as the new target_pose
    # saved_target_pose = arm.get_current_pose(end_effector_link)
        
    # Move to the named pose "up"
    # rospy.logwarn("Going to up position...")
    # arm.set_named_target('up')
    # arm.go()
    # rospy.sleep(1)
        
    # # Go back to the stored target
    # arm.set_pose_target(saved_target_pose, end_effector_link)
    # arm.go()
    # rospy.sleep(1)
        
    # Finish up in the home position  
    # rospy.logwarn("Going back home position...")
    # arm.set_named_target('home')
    # arm.go()

    # Shut down MoveIt cleanly
    moveit_commander.roscpp_shutdown()
    
    # Exit MoveIt
    moveit_commander.os._exit(0)

def UR3_setup():
    # Initialize the move_group API
    moveit_commander.roscpp_initialize(sys.argv)

    # Use the planning scene object to add or remove objects //Interface
    scene = moveit_commander.PlanningSceneInterface()

    rospy.sleep(2)

    REFERENCE_FRAME = '/world'
    p = PlanningScene()
    p.is_diff = True    

    # Create a scene publisher to push changes to the scene //PlanningScene
    scene_pub = rospy.Publisher('/move_group/monitored_planning_scene', PlanningScene, queue_size=1)

  

    pose_Ground = PoseStamped()
    pose_Ground.header.frame_id = REFERENCE_FRAME
    pose_Ground.pose.position.x = 0.0
    pose_Ground.pose.position.y = 0.0
    pose_Ground.pose.position.z = 0.0
    pose_Ground.pose.orientation.w = 1.0

    # Remove leftover objects from a previous run

    scene.remove_world_object('ground')
    scene.remove_world_object('UR3_support')

    # scene.add_plane(Ground_id, pose_Ground, normal=(0,0,1), offset=0.1)
    scene.add_box("ground", pose_Ground, (1.0, 1.0, 0.001))
    # scene.add_box("UR3_support", pose_Ground, (0.2, 0.2, 0.05))
    

    scene_pub.publish(PlanningScene)

    


    
    # Initialize the move group for the right arm
    arm = moveit_commander.MoveGroupCommander('manipulator')
            
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
    arm.set_goal_position_tolerance(0.05) # Changed from 0.01 to 0.05
    arm.set_goal_orientation_tolerance(0.05)

    # Set velocity scaling factor
    arm.set_max_velocity_scaling_factor(0.3)
    arm.set_max_acceleration_scaling_factor(0.3)

    default_joint_states = arm.get_current_joint_values()

    # Test 1 (gripper TOP view)
    default_joint_states[0] = 0.1388552188873291
    default_joint_states[1] = -0.569965124130249
    default_joint_states[2] = 0.17023944854736328
    default_joint_states[3] = -1.1359660625457764
    default_joint_states[4] = -1.5523438453674316
    default_joint_states[5] = -1.4006805419921875

    # Test 2 (gripper LATERAL view)
    # default_joint_states[0] = -0.1791846752166748
    # default_joint_states[1] = 0.05713152885437012
    # default_joint_states[2] = 3.048954486846924
    # default_joint_states[3] = 0.17141366004943848
    # default_joint_states[4] = -0.017139196395874023
    # default_joint_states[5] = -0.025852113962173462


    # Initial position
    arm.set_joint_value_target(default_joint_states)

    # Set the internal state to the current state
    arm.set_start_state_to_current_state()
    plan = arm.plan()

    arm.execute(plan)

def reset_pose():
    # Initialize the move_group API
    moveit_commander.roscpp_initialize(sys.argv)

    # Use the planning scene object to add or remove objects //Interface
    scene = moveit_commander.PlanningSceneInterface()

    rospy.sleep(2)

    REFERENCE_FRAME = '/world'
    p = PlanningScene()
    p.is_diff = True    

    # Create a scene publisher to push changes to the scene //PlanningScene
    scene_pub = rospy.Publisher('/move_group/monitored_planning_scene', PlanningScene, queue_size=1)

  

    pose_Ground = PoseStamped()
    pose_Ground.header.frame_id = REFERENCE_FRAME
    pose_Ground.pose.position.x = 0.0
    pose_Ground.pose.position.y = 0.0
    pose_Ground.pose.position.z = 0.0
    pose_Ground.pose.orientation.w = 1.0

    # Remove leftover objects from a previous run

    scene.remove_world_object('ground')
    scene.remove_world_object('UR3_support')

    # scene.add_plane(Ground_id, pose_Ground, normal=(0,0,1), offset=0.1)
    scene.add_box("ground", pose_Ground, (1.0, 1.0, 0.001))
    # scene.add_box("UR3_support", pose_Ground, (0.2, 0.2, 0.05))
    

    scene_pub.publish(PlanningScene)

    
    # Initialize the move group for the right arm
    arm = moveit_commander.MoveGroupCommander('manipulator')
            
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
    arm.set_goal_position_tolerance(0.05) # Changed from 0.01 to 0.05
    arm.set_goal_orientation_tolerance(0.05)

    # Set velocity scaling factor
    arm.set_max_velocity_scaling_factor(0.3)
    arm.set_max_acceleration_scaling_factor(0.3)

    default_joint_states = arm.get_current_joint_values()

    # Set initial pose as when opening V-REP
    default_joint_states[0] = 0.0008459091186523438
    default_joint_states[1] = 0.001798868179321289
    default_joint_states[2] = -4.291534423828125e-06
    default_joint_states[3] = 0.002761363983154297
    default_joint_states[4] = -0.0001881122589111328
    default_joint_states[5] = -3.141594171524048

    # Initial position
    arm.set_joint_value_target(default_joint_states)

    # Set the internal state to the current state
    arm.set_start_state_to_current_state()
    plan = arm.plan()

    arm.execute(plan)

def up_position():

    # Initialize the move_group API
    moveit_commander.roscpp_initialize(sys.argv)

    # Use the planning scene object to add or remove objects //Interface
    scene = moveit_commander.PlanningSceneInterface()

    rospy.sleep(2)

    REFERENCE_FRAME = '/world'
    p = PlanningScene()
    p.is_diff = True    

    # Create a scene publisher to push changes to the scene //PlanningScene
    scene_pub = rospy.Publisher('/move_group/monitored_planning_scene', PlanningScene, queue_size=1)

  

    pose_Ground = PoseStamped()
    pose_Ground.header.frame_id = REFERENCE_FRAME
    pose_Ground.pose.position.x = 0.0
    pose_Ground.pose.position.y = 0.0
    pose_Ground.pose.position.z = 0.0
    pose_Ground.pose.orientation.w = 1.0

    # Remove leftover objects from a previous run

    scene.remove_world_object('ground')
    scene.remove_world_object('UR3_support')

    # scene.add_plane(Ground_id, pose_Ground, normal=(0,0,1), offset=0.1)
    scene.add_box("ground", pose_Ground, (1.0, 1.0, 0.001))
    # scene.add_box("UR3_support", pose_Ground, (0.2, 0.2, 0.05))
    

    scene_pub.publish(PlanningScene)

    
    # Initialize the move group for the right arm
    arm = moveit_commander.MoveGroupCommander('manipulator')
            
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
    arm.set_goal_position_tolerance(0.05) # Changed from 0.01 to 0.05
    arm.set_goal_orientation_tolerance(0.05)

    # Set velocity scaling factor
    arm.set_max_velocity_scaling_factor(0.3)
    arm.set_max_acceleration_scaling_factor(0.3)

    # Start the arm in the "home" pose stored in the SRDF file
    rospy.logwarn("Going to up position...")
    arm.set_named_target('up')
    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    arm.go()

def pickup_object():
    rospy.loginfo("Grasping object!")
    gripper_pub.publish(True)

def release_object():
    rospy.loginfo("Releasing object!")
    gripper_pub.publish(False)


if __name__ == "__main__":
    rospy.init_node('moveit_demo')
    
    try:

        # ROS stuff
        rospy.Subscriber("/ur3_camera/camera_info", CameraInfo, initCamera)
        rospy.Subscriber("/endpoint_state", Pose, getCameraState)

        rospy.Subscriber('cxy', Tracker, getObjectLocation, queue_size=1) # Position center of object in image

        # scene_pub = rospy.Publisher('/move_group/monitored_planning_scene', PlanningScene, queue_size=1)
        gripper_pub = rospy.Publisher('/gripper_command', Bool, queue_size=1)

        rate = rospy.Rate(50)

        while (camera_info is None):
            rate.sleep()

        rospy.loginfo("Moving to setup pose...")

        UR3_setup()
        # up_position()
        # reset_pose()

        # !! MODIFY THIS LOGIC SOMEHOW... like menu/choices or even SMACH !! #

        raw_input("Press Enter to continue and execute the motion planning...")

        # UR3_preDeterminedMotion()
        UR3_motionPlanning()

        raw_input("Press Enter to grasp the object...")

        pickup_object()

        raw_input("Press Enter to lift the object...")

        UR3_setup()

        raw_input("Press Enter to reset the pose...")

        reset_pose()

        raw_input("Press Enter to release the object...")

        release_object()

        rospy.spin()
    
    except rospy.ROSInterruptException:
        pass