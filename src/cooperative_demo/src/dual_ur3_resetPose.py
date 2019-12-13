#!/usr/bin/env python

"""
    dual_ur3_resetPose.py - Version 0.1 02-12-2019
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

from actionlib_msgs.msg import GoalStatusArray
import actionlib.msg
from actionlib import SimpleActionClient

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

def UR3_resetPose():
    
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

    # INITIAL POSITION - UR3 1
    default_joint_states_UR3_1[0] = 1.5710314512252808
    default_joint_states_UR3_1[1] = 0.0035011768341064453
    default_joint_states_UR3_1[2] = 0.0005176067352294922
    default_joint_states_UR3_1[3] = 0.0004112720489501953
    default_joint_states_UR3_1[4] = -0.0008955001831054688
    default_joint_states_UR3_1[5] = 3.12717342376709

    # INITIAL POSITION - UR3 2
    default_joint_states_UR3_2[0] = 1.570970892906189
    default_joint_states_UR3_2[1] = 0.00843501091003418
    default_joint_states_UR3_2[2] = 0.00045680999755859375
    default_joint_states_UR3_2[3] = 0.0002288818359375
    default_joint_states_UR3_2[4] = -0.0004999637603759766
    default_joint_states_UR3_2[5] = 3.127167224884033


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

    if reset == True:
        while status_controller_UR3_1 != 3 and status_controller_UR3_2 != 3 :
            rospy.logwarn_throttle(5, 'Executing trajectory...')

        rospy.logwarn('Pose reset! Going to next step...')

    else:
        while status_controller_UR3_1 != 3 and status_controller_UR3_2 != 3 :
            rospy.logwarn_throttle(5, 'Executing trajectory...')
    
        rospy.logwarn('Trajectory executed! Going to next step...')
        
    # Wait
    rospy.sleep(0.25)

    # DEBUG wait
    # raw_input('Confirm STEP1?')

    return


if __name__ == "__main__":
    rospy.init_node('dual_ur3_getObject')
    try:

        # ROS stuff

        # Callback status controllers
        rospy.Subscriber('/UR3_2_controller/follow_joint_trajectory/result', FollowJointTrajectoryActionResult, callbackStatus_UR3_2, queue_size=None)
        rospy.Subscriber('/UR3_2_controller/follow_joint_trajectory/result', FollowJointTrajectoryActionResult, callbackStatus_UR3_2, queue_size=None)
        # rospy.Subscriber('/UR3_1_controller/follow_joint_trajectory/status', GoalStatusArray, callbackStatus_UR3_1, queue_size=None)
        # rospy.Subscriber('/UR3_2_controller/follow_joint_trajectory/status', GoalStatusArray, callbackStatus_UR3_2, queue_size=None)

        # scene_pub = rospy.Publisher('/move_group/monitored_planning_scene', PlanningScene, queue_size=1)


        # trajectory_UR3_1_pub = rospy.Publisher('/UR3_1_controller/command', JointTrajectory, queue_size=10)
        # trajectory_UR3_2_pub = rospy.Publisher('/UR3_2_controller/command', JointTrajectory, queue_size=10)

        trajectory_UR3_1_pub = rospy.Publisher('/UR3_1_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)
        trajectory_UR3_2_pub = rospy.Publisher('/UR3_2_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)


        UR3_1_ismoving_pub = rospy.Publisher('/UR3_1_is_moving', Bool, queue_size=10)
        UR3_2_ismoving_pub = rospy.Publisher('/UR3_2_is_moving', Bool, queue_size=10)

        rate = rospy.Rate(50)

        # while (camera_info is None):
        #     rate.sleep()

        rospy.logwarn("Initializing MoveIt! planning scene...")

        planning_setup()

        rospy.logwarn("MoveIt! planning scene ready!")

        rospy.logwarn("Reseting the pose...")

        # STEP1
        status_controller_UR3_1 = 0
        status_controller_UR3_2 = 0
        reset = True

        UR3_resetPose()

        # Exit program
        rospy.signal_shutdown("Reset pose OK")

        rospy.spin()
    
    except rospy.ROSInterruptException:
        pass