#!/usr/bin/env python

"""
    Cooperative manipulation with two UR3 demo script.
"""

import rospy, sys
import moveit_commander
from std_msgs.msg import Bool, String
from moveit_msgs.msg import RobotTrajectory, PlanningScene
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryActionResult

from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import CameraInfo
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from cooperative_demo.msg import Tracker

import image_geometry
import tf

import actionlib.msg 

# Initialization
status_controller_UR3_1 = None
status_controller_UR3_1_step1 = None
status_controller_UR3_2 = None
status_controller_UR3_2_step1 = None

reset = None

# class UR3_motionPlanning:

def callbackStatus_UR3_1(data):
    '''
    Callback for trajectory execution status.
    It creates a new 'status' entry in the 'status_list' every time a goal is sent to the controller.
    So we have to get the relevant goal status in the list.
    '''
    global status_controller_UR3_1
    # global status_controller_UR3_1_step1
    status_controller_UR3_1 = data.status.status
    # status_controller_UR3_1 = data
    # status_controller_UR3_1 = data.status_list[1].status

    # DEBUG 
    # PENDING=0 ACTIVE=1 PREEMPTED=2 SUCCEEDED=3 ABORTED=4 REJECTED=5 PREEMPTING=6 RECALLING=7 RECALLED=8 LOST=9
    # print('----- STATUS UR3_1: ', status_controller_UR3_1)    

def callbackStatus_UR3_2(data):
    global status_controller_UR3_2
    # global status_controller_UR3_2_step1
    status_controller_UR3_2 = data.status.status
    # status_controller_UR3_2 = data
    # status_controller_UR3_2 = data.status_list[1].status
    # DEBUG 
    # PENDING=0 ACTIVE=1 PREEMPTED=2 SUCCEEDED=3 ABORTED=4 REJECTED=5 PREEMPTING=6 RECALLING=7 RECALLED=8 LOST=9
    # print('----- STATUS UR3_2: ', status_controller_UR3_2)

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

def erase_box():


    # Use the planning scene object to add or remove objects //Interface
    scene = moveit_commander.PlanningSceneInterface()



    # Remove leftover objects from a previous run

    scene.remove_world_object('box')

def make_trajectory_goal_msg(joint_trajectory_plan=[], seq=0, secs=0, nsecs=0, dt=2, frame_id='/world'):
    ''' 
    Converts a joint trajectory plan to a FollowJoinTrajectoryActionGoal message
    which is compatible with the topic /UR3_N_controller/follow_joint_trajectory/goal
    '''

    # Goal ID Generator
    id_gen = actionlib.GoalIDGenerator()

    # create joint trajectory message
    joint_trajectory_goal = FollowJointTrajectoryActionGoal(goal_id=id_gen.generate_ID())
    
    # fill the header
    joint_trajectory_goal.header.seq = seq
    joint_trajectory_goal.header.stamp.secs = 0 #secs
    joint_trajectory_goal.header.stamp.nsecs = 0 #nsecs
    joint_trajectory_goal.header.frame_id = frame_id
    
    # specify the joint names
    joint_trajectory_goal.goal.trajectory.joint_names = joint_trajectory_plan.joint_trajectory.joint_names
    
    # fill the trajectory points and computer constraint times
    joint_trajectory_goal.goal.trajectory.points = joint_trajectory_plan.joint_trajectory.points

    return joint_trajectory_goal  
  

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

    # trajectory_UR3_1_pub.publish(plan_UR3_1.joint_trajectory)
    # trajectory_UR3_2_pub.publish(plan_UR3_2.joint_trajectory)

    joint_trajectory_UR3_1 = make_trajectory_goal_msg(joint_trajectory_plan=plan_UR3_1, secs=1, dt=0.1, frame_id=reference_frame_UR3_1)
    print('Trajectory UR3_1 generated!')

    joint_trajectory_UR3_2 = make_trajectory_goal_msg(joint_trajectory_plan=plan_UR3_2, secs=1, dt=0.1, frame_id=reference_frame_UR3_2)
    print('Trajectory UR3_2 generated!')

    # Publish trajectory to action server
    trajectory_UR3_1_pub.publish(joint_trajectory_UR3_1)
    trajectory_UR3_2_pub.publish(joint_trajectory_UR3_2)

    rospy.sleep(1)

    if reset == True:
        while status_controller_UR3_1 != 3 and status_controller_UR3_2 != 3 :
            rospy.logwarn_throttle(5, 'Executing trajectory...')

        rospy.logwarn('Pose reset! Going to next step...')

    else:
        while status_controller_UR3_1 != 3 and status_controller_UR3_2 != 3 :
            rospy.logwarn_throttle(5, 'Executing trajectory...')
    
        rospy.logwarn('Trajectory executed! Going to next step...')
        
    # Wait
    rospy.sleep(1)

    # DEBUG wait
    # raw_input('Confirm STEP1?')

    return

    

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
    # trajectory_UR3_1_pub.publish(plan_UR3_1.joint_trajectory)
    # trajectory_UR3_2_pub.publish(plan_UR3_2.joint_trajectory)

    # Convert joint_trajectory message to action goal
    joint_trajectory_UR3_1 = make_trajectory_goal_msg(joint_trajectory_plan=plan_UR3_1, secs=1, dt=0.1, frame_id=reference_frame_UR3_1)
    print('Trajectory UR3_1 generated!')

    joint_trajectory_UR3_2 = make_trajectory_goal_msg(joint_trajectory_plan=plan_UR3_2, secs=1, dt=0.1, frame_id=reference_frame_UR3_2)
    print('Trajectory UR3_2 generated!')

    # Publish trajectory to action server
    trajectory_UR3_1_pub.publish(joint_trajectory_UR3_1)
    trajectory_UR3_2_pub.publish(joint_trajectory_UR3_2)

    rospy.sleep(1)

    while status_controller_UR3_1 != 3 and status_controller_UR3_2 != 3 :
        rospy.logwarn_throttle(5, 'Executing trajectory...')

    rospy.logwarn('Trajectory executed! Going to next step...')
        
    # Wait
    rospy.sleep(2)

    # DEBUG wait
    # raw_input('Confirm STEP2?')

    return

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
    # trajectory_UR3_1_pub.publish(plan_UR3_1.joint_trajectory)
    # trajectory_UR3_2_pub.publish(plan_UR3_2.joint_trajectory)

    # Shift the end-effector to the right 5cm
    # arm1.shift_pose_target(1, -0.05, end_effector_link_UR3_1)
    # arm1.go()
    # rospy.sleep(1)

    # Convert joint_trajectory message to action goal
    joint_trajectory_UR3_1 = make_trajectory_goal_msg(joint_trajectory_plan=plan_UR3_1, secs=1, dt=0.1, frame_id=reference_frame_UR3_1)
    print('Trajectory UR3_1 generated!')

    joint_trajectory_UR3_2 = make_trajectory_goal_msg(joint_trajectory_plan=plan_UR3_2, secs=1, dt=0.1, frame_id=reference_frame_UR3_2)
    print('Trajectory UR3_2 generated!')

    # Publish trajectory to action server
    trajectory_UR3_1_pub.publish(joint_trajectory_UR3_1)
    trajectory_UR3_2_pub.publish(joint_trajectory_UR3_2)

    rospy.sleep(1)

    while status_controller_UR3_1 != 3 and status_controller_UR3_2 != 3 :
        rospy.logwarn_throttle(5, 'Executing trajectory...')

    rospy.logwarn('Trajectory executed! Going to next step...')
        
    # Wait
    rospy.sleep(2)

    # DEBUG wait
    # raw_input('Confirm STEP3?')

    return

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
    # trajectory_UR3_1_pub.publish(plan_UR3_1.joint_trajectory)
    # trajectory_UR3_2_pub.publish(plan_UR3_2.joint_trajectory)

    # Convert joint_trajectory message to action goal
    joint_trajectory_UR3_1 = make_trajectory_goal_msg(joint_trajectory_plan=plan_UR3_1, secs=1, dt=0.1, frame_id=reference_frame_UR3_1)
    print('Trajectory UR3_1 generated!')

    joint_trajectory_UR3_2 = make_trajectory_goal_msg(joint_trajectory_plan=plan_UR3_2, secs=1, dt=0.1, frame_id=reference_frame_UR3_2)
    print('Trajectory UR3_2 generated!')

    # Publish trajectory to action server
    trajectory_UR3_1_pub.publish(joint_trajectory_UR3_1)
    trajectory_UR3_2_pub.publish(joint_trajectory_UR3_2)

    rospy.sleep(1)

    while status_controller_UR3_1 != 3 and status_controller_UR3_2 != 3 :
        rospy.logwarn_throttle(5, 'Executing trajectory...')

    rospy.logwarn('Trajectory executed! Going to next step...')
        
    # Wait
    rospy.sleep(1)

    # DEBUG wait
    # raw_input('Confirm STEP4?')

    return

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
    # # Publishes directly to controllers the MoveIt! plan 
    # trajectory_UR3_1_pub.publish(plan_UR3_1.joint_trajectory)
    # trajectory_UR3_2_pub.publish(plan_UR3_2.joint_trajectory)

    # Convert joint_trajectory message to action goal
    joint_trajectory_UR3_1 = make_trajectory_goal_msg(joint_trajectory_plan=plan_UR3_1, secs=1, dt=0.1, frame_id=reference_frame_UR3_1)
    print('Trajectory UR3_1 generated!')

    joint_trajectory_UR3_2 = make_trajectory_goal_msg(joint_trajectory_plan=plan_UR3_2, secs=1, dt=0.1, frame_id=reference_frame_UR3_2)
    print('Trajectory UR3_2 generated!')

    # Publish trajectory to action server
    trajectory_UR3_1_pub.publish(joint_trajectory_UR3_1)
    trajectory_UR3_2_pub.publish(joint_trajectory_UR3_2)

    rospy.sleep(1)

    while status_controller_UR3_1 != 3 and status_controller_UR3_2 != 3 :
        rospy.logwarn_throttle(5, 'Executing trajectory...')

    rospy.logwarn('Trajectory executed! Going to next step...')
        
    # Wait
    rospy.sleep(1)

    # DEBUG wait
    # raw_input('Confirm STEP5?')

    return

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
    # trajectory_UR3_1_pub.publish(plan_UR3_1.joint_trajectory)
    # trajectory_UR3_2_pub.publish(plan_UR3_2.joint_trajectory)

    # Convert joint_trajectory message to action goal
    joint_trajectory_UR3_1 = make_trajectory_goal_msg(joint_trajectory_plan=plan_UR3_1, secs=1, dt=0.1, frame_id=reference_frame_UR3_1)
    print('Trajectory UR3_1 generated!')

    joint_trajectory_UR3_2 = make_trajectory_goal_msg(joint_trajectory_plan=plan_UR3_2, secs=1, dt=0.1, frame_id=reference_frame_UR3_2)
    print('Trajectory UR3_2 generated!')

    # Publish trajectory to action server
    trajectory_UR3_1_pub.publish(joint_trajectory_UR3_1)
    trajectory_UR3_2_pub.publish(joint_trajectory_UR3_2)

    rospy.sleep(1)

    while status_controller_UR3_1 != 3 and status_controller_UR3_2 != 3 :
        rospy.logwarn_throttle(5, 'Executing trajectory...')

    rospy.logwarn('Trajectory executed! Going to next step...')
        
    # Wait
    rospy.sleep(1)

    # DEBUG wait
    # raw_input('Confirm STEP6?')

    return

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
    # trajectory_UR3_1_pub.publish(plan_UR3_1.joint_trajectory)
    # trajectory_UR3_2_pub.publish(plan_UR3_2.joint_trajectory)

    # Convert joint_trajectory message to action goal
    joint_trajectory_UR3_1 = make_trajectory_goal_msg(joint_trajectory_plan=plan_UR3_1, secs=1, dt=0.1, frame_id=reference_frame_UR3_1)
    print('Trajectory UR3_1 generated!')

    joint_trajectory_UR3_2 = make_trajectory_goal_msg(joint_trajectory_plan=plan_UR3_2, secs=1, dt=0.1, frame_id=reference_frame_UR3_2)
    print('Trajectory UR3_2 generated!')

    # Publish trajectory to action server
    trajectory_UR3_1_pub.publish(joint_trajectory_UR3_1)
    trajectory_UR3_2_pub.publish(joint_trajectory_UR3_2)

    rospy.sleep(1)

    while status_controller_UR3_1 != 3 and status_controller_UR3_2 != 3 :
        rospy.logwarn_throttle(5, 'Executing trajectory...')

    rospy.logwarn('Trajectory executed! Going to next step...')
        
    # Wait
    rospy.sleep(1)

    # DEBUG wait
    # raw_input('Confirm STEP7?')

    return


def pickup_object():
    rospy.loginfo("Grasping object!")

    gripper_UR3_1_pub.publish(True)
    rospy.logwarn("Gripper 1 CLOSED!")
    gripper_UR3_2_pub.publish(True)
    rospy.logwarn("Gripper 2 CLOSED!")

    # Wait a bit
    rospy.sleep(1)


def release_object():
    rospy.loginfo("Releasing object!")
    gripper_UR3_1_pub.publish(False)
    rospy.logwarn("Gripper 1 OPEN!")
    gripper_UR3_2_pub.publish(False)
    rospy.logwarn("Gripper 2 OPEN!")

    # Wait a bit
    rospy.sleep(1)


if __name__ == "__main__":
    rospy.init_node('dual_ur3_motionPlanning_sync')
    
    try:

        # ROS stuff
        # rospy.Subscriber("/UR3_1/camera/camera_info", CameraInfo, initCamera)
        # rospy.Subscriber("/UR3_1/endpoint_state", Pose, getCameraState)

        # rospy.Subscriber('UR3_1/cxy', Tracker, getObjectLocation, queue_size=1) # Position center of object in image

        # rospy.Subscriber("/UR3_2/camera/camera_info", CameraInfo, initCamera)
        # rospy.Subscriber("/UR3_2/endpoint_state", Pose, getCameraState)

        # rospy.Subscriber('UR3_2/cxy', Tracker, getObjectLocation, queue_size=1) # Position center of object in image

         # Callback status controllers
        rospy.Subscriber('/UR3_1_controller/follow_joint_trajectory/result', FollowJointTrajectoryActionResult, callbackStatus_UR3_2, queue_size=None)
        rospy.Subscriber('/UR3_2_controller/follow_joint_trajectory/result', FollowJointTrajectoryActionResult, callbackStatus_UR3_2, queue_size=None)

        # scene_pub = rospy.Publisher('/move_group/monitored_planning_scene', PlanningScene, queue_size=1)
        gripper_UR3_1_pub = rospy.Publisher('/UR3_1/gripper_command', Bool, queue_size=1)
        gripper_UR3_2_pub = rospy.Publisher('/UR3_2/gripper_command', Bool, queue_size=1)

        trajectory_UR3_1_pub = rospy.Publisher('/UR3_1_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)
        trajectory_UR3_2_pub = rospy.Publisher('/UR3_2_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)

        objectRequested_pub = rospy.Publisher('/object_requested', String, queue_size=1)

        rate = rospy.Rate(50)

        # while (camera_info is None):
        #     rate.sleep()

        rospy.logwarn("Initializing MoveIt! planning scene...")

        planning_setup()

        rospy.logwarn("MoveIt! planning scene ready!")

        rospy.loginfo("Moving to setup pose...")

        status_controller_UR3_1 = 0
        status_controller_UR3_2 = 0

        UR3_setup()
        # up_position()
        # reset_pose()

        # In addition to logwarn, publish to a topic for interface output then #

        rospy.logwarn("Positioning dual arm for grasping box...")

        status_controller_UR3_1 = 0
        status_controller_UR3_2 = 0        

        dual_grasping_jointStates()

        rospy.logwarn("Adjusting the pose for grasping box...")

        status_controller_UR3_1 = 0
        status_controller_UR3_2 = 0

        dual_grasping_AdjustjointStates()

        # raw_input("Press Enter to continue and position dual arm for grasping box (Pose command)...")

        # status_controller_UR3_1 = 0
        # status_controller_UR3_2 = 0

        # dual_grasping_pose()

        # raw_input("Press Enter to continue and adjust the pose for grasping box (Pose command)...")
    
        # status_controller_UR3_1 = 0
        # status_controller_UR3_2 = 0

        # dual_grasping_Adjustpose()

        rospy.logwarn("Grasping the object...")
    
        status_controller_UR3_1 = 0
        status_controller_UR3_2 = 0

        pickup_object()

        rospy.logwarn("Lifting the box...")

        status_controller_UR3_1 = 0
        status_controller_UR3_2 = 0

        dual_grasping_LiftjointStates()

        # raw_input("Press Enter to continue and move the box (JointStates command)...")
        
        # status_controller_UR3_1 = 0
        # status_controller_UR3_2 = 0
        
        # dual_grasping_MovejointStates()

        # raw_input("Press Enter to continue and place back the box on table STEP1 (JointStates command)...")

        # status_controller_UR3_1 = 0
        # status_controller_UR3_2 = 0

        # dual_grasping_LiftjointStates()

        rospy.logwarn("Placing back the box on table STEP2 (JointStates command)...")

        status_controller_UR3_1 = 0
        status_controller_UR3_2 = 0

        dual_grasping_AdjustjointStates()

        rospy.logwarn("Releasing the object...")

        status_controller_UR3_1 = 0
        status_controller_UR3_2 = 0

        release_object()

        rospy.logwarn("Reset the pose...")
        
        reset = True
        status_controller_UR3_1 = 0
        status_controller_UR3_2 = 0

        dual_grasping_jointStates()
    
        rospy.sleep(1)

        planning_setup()
        print ("Box republished in scene...")
        
        rospy.sleep(1)

        status_controller_UR3_1 = 0
        status_controller_UR3_2 = 0

        UR3_setup()

        # raw_input("Press Enter to reset the pose...")

        rospy.signal_shutdown("Moving box OK")

        # reset_pose()

        rospy.spin()
    
    except rospy.ROSInterruptException:
        pass