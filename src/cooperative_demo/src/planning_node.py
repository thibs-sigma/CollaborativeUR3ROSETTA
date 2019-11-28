#!/usr/bin/env python
import struct
import numpy as np

# REQUIRED TO CALL ROS PACKAGES
import rospy

# REQUIRED FOR PLANNING
import moveit_commander

# MESSAGE TYPES
from std_msgs.msg import Header

# TO CALL IK_SOLVER SERVICE
# from baxter_core_msgs.srv import (
    # SolvePositionIK,
    # SolvePositionIKRequest)

# TO WRITE TO BAXTER
# import baxter_interface

# MESSAGE TYPES
from sensor_msgs.msg import JointState

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion)

# TO USE AS A CALLBACK NODE IN ANOTHER PYTHON FILE
def initplannode(goal, limb):
    print "Inside Planning Node"
    print goal

    # Actual node that solves IK and calls the MoveIt! planner
    # in joint-space
    move_pos(goal, limb)
    return

# Uses Baxter's IK service to find the required joint angles,
# It then calls MoveIT! planners to plan to these joint angles
def move_pos(goal, limb, timeout=3):
    # Call Baxter's inbuilt service
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)

    # Request for same service
    ikreq = SolvePositionIKRequest()

    # Fill in all message parts
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')

    # Define  required x, y, z parts
    pose = Pose()

    # Define orientation of gripper.
    quat = Quaternion()
    quat.x = goal[3]
    quat.y = goal[4]
    quat.z = goal[5]
    quat.w = goal[6]

    # Fill the position vectors from goal configuration
    pose.orientation = quat
    pose.position.x = goal[0]
    pose.position.y = goal[1]
    pose.position.z = goal[2]

    ikreq.pose_stamp = [PoseStamped(header=hdr, pose=pose)]

    # Using Random seeds till a "VALID" IK solution is found
    # FOR SEED METHOD:
    # cite1: IK_Service_Client in package: Baxter_Examples
    # cite2: test_moveit.py from NXR repo linked in Course notes
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.loginfo("Service exception")

    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))

        # Initialize joint angles
        des_joints = [0]*7

        # Update all joint angles
        for i in range(7):
            des_joints[i] = resp.joints[0].position[i]

        # Begin interaction with move_it joint_action_server.
        # Object of type move_group_commander is used to
        # interact with Baxter using joint_action_server
        la = moveit_commander.MoveGroupCommander("left_arm")

        # Set desired joint angles to plan in joint space
        la.set_joint_value_target(des_joints)

        # FOR JOINT SPACE PLANNING:
        # cite1: test_moveit.py from NXR repo linked in Course notes
        la.plan()

        la.set_max_velocity_scaling_factor(0.3) # Adjust speed here

        # Sets Baxter arm in Motion
        la.go()

    else:
        print("INVALID POSE")
        print("Trying random seeds until timeout is reached")
        ikt = ik_timeout(ikreq, limb, 5)
        if ikt is not None and len(ikt.joints[0].position) != 0:
            des_joints = [0]*7
            for i in range(7):
                print i
                des_joints[i] = ikt.joints[0].position[i]

            # Same moveit! calling strategy as is written in if code block
            la = moveit_commander.MoveGroupCommander("left_arm")
            la.set_joint_value_target(des_joints)
            la.plan()
            la.go()

        else:
            print "-------------INVALID POSITION------------"
    return

# REFERENCES:
# test_moveit.py from NXR LAB BAXTER REPO (Linked in course notes and README file of this repo)
# baxter_examples package --> ik_service_client.py example which
# ... tries to find ik solution using Baxter's inbult ik service and
# ... trying to find valid solution by using random trials of intial conditions
# ... called "RANDOM SEED" in this particular reference
