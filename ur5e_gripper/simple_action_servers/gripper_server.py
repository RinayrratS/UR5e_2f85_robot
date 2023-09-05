#!/usr/bin/env python
import rospy
import actionlib
import robotiq_gripper_msgs.msg

def fibonacci_server():
    rospy.init_node('GripperCommand_node')
    server = actionlib.SimpleActionServer('GripperCommand', GripperCommand, execute_cb, False)
    server.start()
    rospy.loginfo("Fibonacci Server is up and running.")
    rospy.spin()
