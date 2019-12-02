#!/usr/bin/env python
import sys
import rospy
import roslaunch

from std_msgs.msg import String, Bool, Float32, UInt32


# is_moving = False

# def check_moving(data):
#     global is_moving
#     is_moving = data.data

# Callback UR3 supervision interface for action 
# def wheelIndexCallback(data):
#     global wheelIndex_value
#     wheelIndex_value = data.data
#     # print (wheelIndex_value % 3)

def poll_action_request():
    while True:
       
        if (wheelIndex_value % 6) == 2 :
            # Debug terminal
            # print "Assembly task"
            desired_action = "assembly"
            # BAXTER SCREEN OUTPUT
            image_pub.publish(msg_assemblyTask)

        elif (wheelIndex_value % 6) == 3 :
            # Debug terminal
            # print "Pickup task"
            desired_action = "pickup"
            # BAXTER SCREEN OUTPUT
            image_pub.publish(msg_pickupObject)

        elif (wheelIndex_value % 6) == 4 :
            # Debug terminal
            # print "Go to home"
            desired_action = "home"
            # BAXTER SCREEN OUTPUT
            image_pub.publish(msg_goHome)
        
        elif (wheelIndex_value % 6) == 5 :
            # Debug terminal
            # print "EXIT!"
            desired_action = "q"
            # BAXTER SCREEN OUTPUT
            image_pub.publish(msg_exit)



        # desired_action = raw_input('Enter the object you would like to pick up (q to quit): ')


            if desired_action == 'exit':
                print "EXIT requested"
                image_pub.publish(msg_sleeping)
                desired_action_pub.publish(desired_action)
                leftInnerLight_pub.publish('left_inner_light', False)
                rospy.sleep(1)
                break
            
            elif desired_action == 'tuck':
                # Debug terminal
                # print "Finding and picking up ",desired_obobject
                break

            elif desired_action == 'untuck':
                # Debug terminal
                # print "Finding and picking up ",desired_action
                # Switch off button light
                image_pub.publish(msg_untuckingArms)
                desired_action_pub.publish(desired_action)
                leftInnerLight_pub.publish('left_inner_light', False)
                rospy.sleep(1)
                break          

            elif desired_action == 'home':
                # Debug terminal
                # Switch off button light
                # image_pub.publish(msg_readyPickup)
                leftInnerLight_pub.publish('left_inner_light', False)
                desired_action_pub.publish(desired_action)
                print "GOINGHOME requested"
                rospy.sleep(1)
                break


            elif desired_action == 'assembly':
                # Debug terminal
                # print "Finding and picking up ",desired_action
                # Switch off button light
                image_pub.publish(msg_readyAssembly)
                leftInnerLight_pub.publish('left_inner_light', False)
                desired_action_pub.publish(desired_action)
                print "ASSEMBLY requested"
                rospy.sleep(1)
                break

            elif desired_action == 'pickup':
                # Debug terminal
                # print "Finding and picking up ",desired_action
                # Switch off button light
                image_pub.publish(msg_readyPickup)
                leftInnerLight_pub.publish('left_inner_light', False)
                desired_action_pub.publish(desired_action)
                print "PICKUP requested"
                rospy.sleep(1)
                break

        while is_moving:
            pass

    print "Done!"

if __name__ == '__main__':
    try:
        rospy.init_node('request_action', log_level=rospy.INFO)

        rate = rospy.Rate(100)
        desired_action_pub = rospy.Publisher("/desired_action",String,queue_size=10)

        # rospy.Subscriber("/is_moving",Bool,check_moving)

        poll_action_request()

    except rospy.ROSInterruptException:
        pass