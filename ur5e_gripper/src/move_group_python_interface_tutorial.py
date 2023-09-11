#!/usr/bin/env python

<<<<<<< HEAD
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

=======
>>>>>>> update
# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
<<<<<<< HEAD
=======
import math
>>>>>>> update
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import robotiq_gripper_msgs.msg
<<<<<<< HEAD
import actionlib
=======
import time
import actionlib

>>>>>>> update
try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""
<<<<<<< HEAD
=======
    pub: object
    move_group: object
>>>>>>> update

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        self.robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        self.scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
<<<<<<< HEAD
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        # group_name = "panda_arm"
        self.group_name = "manipulator"

=======
        ## arm joints in the ur5e robot, so we set the group's name to "arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        self.group_name = "manipulator"
>>>>>>> update
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
<<<<<<< HEAD
=======

>>>>>>> update
        self.joint = [moveit_commander.RobotCommander.Joint(
            self.robot, 'shoulder_pan_joint')]
        self.joint.append(moveit_commander.RobotCommander.Joint(
            self.robot, 'shoulder_lift_joint'))
        self.joint.append(moveit_commander.RobotCommander.Joint(
            self.robot, 'elbow_joint'))
        self.joint.append(moveit_commander.RobotCommander.Joint(
            self.robot, 'wrist_1_joint'))
        self.joint.append(moveit_commander.RobotCommander.Joint(
            self.robot, 'wrist_2_joint'))
        self.joint.append(moveit_commander.RobotCommander.Joint(
            self.robot, 'wrist_3_joint'))
        self.pub = rospy.Publisher('gripper_command', robotiq_gripper_msgs.msg.GripperCommand, queue_size=10)
<<<<<<< HEAD
        self.rate = rospy.Rate(40) # 10hz
=======
        self.rate = rospy.Rate(40)  # 10hz
>>>>>>> update
        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        self.planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % self.planning_frame)

        # We can also print the name of the end-effector link for this group:
        self.eef_link = self.move_group.get_end_effector_link()
        print("============ End effector link: %s" % self.eef_link)

        # We can get a list of all the groups in the robot:
        self.group_names = self.robot.get_group_names()
        print("============ Available Planning Groups:", self.robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(self.robot.get_current_state())
<<<<<<< HEAD
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        #self.box_name = ""
        #self.robot = robot
        #self.scene = scene
        #self.move_group = move_group
        #self.display_trajectory_publisher = display_trajectory_publisher
        #self.planning_frame = planning_frame
        #self.eef_link = eef_link
        #self.group_names = group_names
=======
        print(self.move_group.get_current_pose())
        ## END_SUB_TUTORIAL

        # # Misc variables
        # self.box_name = ""
        # self.robot = robot
        # self.scene = scene
        # self.move_group = move_group
        # self.display_trajectory_publisher = display_trajectory_publisher
        # self.planning_frame = planning_frame
        # self.eef_link = eef_link
        # self.group_names = group_names

    def go_to_home_state(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        # Put the arm in the start position
        self.move_group.set_max_velocity_scaling_factor(0.5)  # Adjust the value as needed
        #self.move_group.set_named_target("up")
        pose_goal = self.move_group.get_current_pose().pose
        pose_goal.position.x = 0.5
        pose_goal.position.y = 0.0
        pose_goal.position.z = 1.0
        self.move_group.set_pose_target(pose_goal)

        success = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        self.move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose
        time.sleep(2)
        print(current_pose)
        print(all_close(pose_goal, current_pose, 0.005))

>>>>>>> update

    def go_to_joint_state(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

<<<<<<< HEAD
        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -tau / 8
        joint_goal[2] = 0
        joint_goal[3] = -tau / 4
        joint_goal[4] = 0
        joint_goal[5] = tau / 6  # 1/6 of a turn
        # joint_goal[6] = 0
=======
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = math.radians(-120)
        joint_goal[1] = math.radians(-130)
        joint_goal[2] = math.radians(50)
        joint_goal[3] = math.radians(-70)
        joint_goal[4] = math.radians(-80)
        joint_goal[5] = math.radians(90)
>>>>>>> update

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

<<<<<<< HEAD
    def go_to_pose_goal(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group
=======

    def go_to_pose_goal(self,pose_goal):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
>>>>>>> update

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
<<<<<<< HEAD
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4

        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL
=======
        pose_goal = self.move_group.get_current_pose().pose
        print("============ Printing robot position")
        print(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z)
        print("")
        #pose_goal.orientation.w = 1.0

        pose_goal.position.x = 0.5
        pose_goal.position.y = -0.1
        pose_goal.position.z = 0.78

        print("============ Printing robot position")
        print(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z)
        print("")

        self.move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        self.move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose
        time.sleep(2)
        print(current_pose)
        print(all_close(pose_goal, current_pose, 0.005))

    def linear_motion_absolute(self, length1: object, length2: object, length3: object) -> object:

        current_pose = self.move_group.get_current_pose().pose

        pose_goal = current_pose

        pose_goal.position.x = length1
        pose_goal.position.y = length2
        pose_goal.position.z = length3

        self.move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        plan = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()
>>>>>>> update

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
<<<<<<< HEAD
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
=======
        time.sleep(2)
        print(current_pose)
        print(all_close(pose_goal, current_pose, 0.005))

    def pick_and_place(self) -> object:

        self.linear_motion_absolute(0.4, -0.1, 0.895)
        time.sleep(1)
        self.linear_motion_absolute(0.4, -0.1, 0.78)
        self.gripper_close()
        time.sleep(1)
        self.linear_motion('z', 0.15)

        self.linear_motion_absolute(0.4, 0.5, 0.895)
        self.linear_motion_absolute(0.4, 0.5, 0.8)
        self.gripper_open()
        time.sleep(1)
        self.linear_motion('z', 0.15)

    def go_to_goal_state(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = self.move_group.get_current_pose().pose
        print("============ Printing robot position")
        print(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z)
        print("")
        #pose_goal.orientation.w = 1.0

        pose_goal.position.x = 0.5
        pose_goal.position.y = 0.3
        pose_goal.position.z = 0.78

        print("============ Printing robot position")
        print(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z)
        print("")

        self.move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        self.move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose
        time.sleep(2)
        print(current_pose)
        print(all_close(pose_goal, current_pose, 0.005))

    def plan_cartesian_path(self, scale=1):
        move_group = self.move_group

>>>>>>> update
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

<<<<<<< HEAD
        ## END_SUB_TUTORIAL
=======
>>>>>>> update

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL

<<<<<<< HEAD
    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Received
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node was just created (https://github.com/ros/ros_comm/issues/176),
        ## or dies before actually publishing the scene update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed.
        ## To avoid waiting for scene updates like this at all, initialize the
        ## planning scene interface with  ``synchronous = True``.
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL
    def grasp_client(self):
    # Creates the SimpleActionClient, passing the type of the action
    # (GraspAction) to the constructor.
        client = actionlib.SimpleActionClient('gripper_command', robotiq_gripper_msgs.msg.GripperCommand)

    def gripper_open(self):
        goal = self.pub
        goal.position = 0.0
        goal.speed = 1.0
        goal.force = 0.0
        start1 = rospy.get_time()
        seconds2 = rospy.get_time()
        while (seconds2-start1)<0.3:
            self.pub.publish(goal)
            self.rate.sleep()

        
    # rate = rospy.Rate(40) # 10hz
    # #pub.publish(0.85, 1.0, 0.0)
    # #rate.sleep()
    # #pub.publish(0.0, 1.0, 0.0)
    # # return
    # while (seconds2 - start1 < 0.3) and not rospy.is_shutdown():
    #     # command = {"position": 1.0,
    #     #            "speed": 1.0,
    #     #            "force": 0.0}
    #     # command = ["0.85","1.0", "0.0"]
    #     # # command = "position: 0.85\nspeed: 1.0\nforce: 0.0"
    #     # rospy.loginfo(command)
    #     pub.publish(0.85, 1.0, 0.0)
    #     # break
    #     rate.sleep()
    #     seconds2 = rospy.get_time()
=======
    # def wait_for_state_update(
    #     self, box_is_known=False, box_is_attached=False, timeout=4
    # ):
    #     box_name = self.box_name
    #     scene = self.scene
    #
    #     start = rospy.get_time()
    #     seconds = rospy.get_time()
    #     while (seconds - start < timeout) and not rospy.is_shutdown():
    #         # Test if the box is in attached objects
    #         attached_objects = scene.get_attached_objects([box_name])
    #         is_attached = len(attached_objects.keys()) > 0
    #
    #         # Test if the box is in the scene.
    #         # Note that attaching the box will remove it from known_objects
    #         is_known = box_name in scene.get_known_object_names()
    #
    #         # Test if we are in the expected state
    #         if (box_is_attached == is_attached) and (box_is_known == is_known):
    #             return True
    #
    #         # Sleep so that we give other threads time on the processor
    #         rospy.sleep(0.1)
    #         seconds = rospy.get_time()
    #
    #     # If we exited the while loop without returning then we timed out
    #     return False
    #     ## END_SUB_TUTORIAL
    #
    # def add_box(self, timeout=4):
    #     box_name = self.box_name
    #     scene = self.scene
    #     ## Adding Objects to the Planning Scene
    #     ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    #     ## First, we will create a box in the planning scene between the fingers:
    #     box_pose = geometry_msgs.msg.PoseStamped()
    #     box_pose.header.frame_id = "world"
    #     box_pose.pose.orientation.w = 1.0
    #     box_pose.pose.position.x = 0.65
    #     box_pose.pose.position.y = 0
    #     box_pose.pose.position.z = 0.6  # above the panda_hand frame
    #     box_name = "box"
    #     scene.add_box(box_name, box_pose, size=(0.075, 0.075, 0.075))
    #
    #     self.box_name = box_name
    #     return self.wait_for_state_update(box_is_known=True, timeout=timeout)
    #
    # def attach_box(self, timeout=4):
    #     box_name = self.box_name
    #     robot = self.robot
    #     scene = self.scene
    #     eef_link = self.eef_link
    #     group_names = self.group_names
    #
    #     touch_links = robot.get_link_names(group=tool0)
    #     scene.attach_box(eef_link, box_name, touch_links=touch_links)
    #
    #     return self.wait_for_state_update(
    #         box_is_attached=True, box_is_known=False, timeout=timeout
    #     )
    #
    # def detach_box(self, timeout=4):
    #     box_name = self.box_name
    #     scene = self.scene
    #     eef_link = self.eef_link
    #
    #     ## Detaching Objects from the Robot
    #     ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    #     ## We can also detach and remove the object from the planning scene:
    #     scene.remove_attached_object(eef_link, name=box_name)
    #
    #     return self.wait_for_state_update(
    #         box_is_known=True, box_is_attached=False, timeout=timeout
    #     )
    #
    # def remove_box(self, timeout=4):
    #     box_name = self.box_name
    #     scene = self.scene
    #
    #     ## Removing Objects from the Planning Scene
    #     ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    #     ## We can remove the box from the world.
    #     scene.remove_world_object(box_name)
    #
    #     ## **Note:** The object must be detached before we can remove it from the world
    #
    #     return self.wait_for_state_update(
    #         box_is_attached=False, box_is_known=False, timeout=timeout
    #     )
    def grasp_client(self):
        # Creates the SimpleActionClient, passing the type of the action
        # (GraspAction) to the constructor.
        client = actionlib.SimpleActionClient('gripper_command', robotiq_gripper_msgs.msg.GripperCommand)

    def gripper_close(self):
        goal = self.pub
        start1 = rospy.get_time()
        seconds2 = rospy.get_time()
        while (seconds2 - start1) < 0.3 and not rospy.is_shutdown():
            goal.publish(0.85, 1.0, 0.0)
            self.rate.sleep()
            seconds2 = rospy.get_time()

    def gripper_open(self):
        goal = self.pub
        start1 = rospy.get_time()
        seconds2 = rospy.get_time()
        while (seconds2 - start1) < 0.3 and not rospy.is_shutdown():
            goal.publish(0.0, 1.0, 0.0)
            self.rate.sleep()
            seconds2 = rospy.get_time()
>>>>>>> update

def main():
    try:
        print("")
<<<<<<< HEAD
        print("----------------------------------------------------------")
        print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        input(
            "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
        )
        tutorial = MoveGroupPythonInterfaceTutorial()

        input(
            "============ Press `Enter` to execute a movement using a joint state goal ..."
        )
        tutorial.go_to_joint_state()

        input("============ Press `Enter` to execute a movement using a pose goal ...")
        #tutorial.grasp()
        input()
        tutorial.gripper_open()

        input(
            "============ Press `Enter` to plan and execute a path with an attached collision object ..."
        )
        cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
        tutorial.execute_plan(cartesian_plan)


=======
        print("-----------------------------------")
        print("UR5e with robotiq_85_gripper robot")
        print("-----------------------------------")
        print("")

        input(
            "============ Press `Enter` to initialize the pose of robot ..."
        )
        tutorial = MoveGroupPythonInterfaceTutorial()
        tutorial.go_to_home_state()
        tutorial.gripper_open()
        #
        # input(
        #     "============ Press `Enter` to execute a movement using a joint state goal ..."
        # )
        #tutorial.go_to_joint_state()

        input("============ Press `Enter` to execute a movement using a pose goal ...")
        tutorial.go_to_pose_goal()
        tutorial.gripper_close()
        input(
            "============ Press `Enter` to go to the goal of robot ..."
        )
        tutorial.go_to_home_state()
        tutorial.go_to_goal_state()
        tutorial.gripper_open()



        # input("============ Press `Enter` to plan and display a Cartesian path ...")
        # cartesian_plan, fraction = tutorial.plan_cartesian_path()

        # input(
        #     "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
        # )
        #tutorial.display_trajectory(cartesian_plan)

        #input("============ Press `Enter` to execute a saved path ...")
        # tutorial.execute_plan(cartesian_plan)

        # input("============ Press `Enter` to add a box to the planning scene ...")
        # tutorial.add_box()
        #
        # input("============ Press `Enter` to attach a Box to the UR5e robot ...")
        # tutorial.attach_box()
        #
        # input(
        #     "============ Press `Enter` to plan and execute a path with an attached collision object ..."
        # )
        # cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
        # tutorial.execute_plan(cartesian_plan)
        #
        # input("============ Press `Enter` to detach the box from the UR5e robot ...")
        # tutorial.detach_box()
        #
        # input(
        #     "============ Press `Enter` to remove the box from the planning scene ..."
        # )
        # tutorial.remove_box()

>>>>>>> update
        print("============ Python tutorial demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()

## BEGIN_TUTORIAL
## .. _moveit_commander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/namespacemoveit__commander.html
##
## .. _MoveGroupCommander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
## .. _RobotCommander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _PlanningSceneInterface:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
## .. _DisplayTrajectory:
##    http://docs.ros.org/noetic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
## .. _RobotTrajectory:
##    http://docs.ros.org/noetic/api/moveit_msgs/html/msg/RobotTrajectory.html
##
## .. _rospy:
##    http://docs.ros.org/noetic/api/rospy/html/
## CALL_SUB_TUTORIAL imports
## CALL_SUB_TUTORIAL setup
## CALL_SUB_TUTORIAL basic_info
## CALL_SUB_TUTORIAL plan_to_joint_state
## CALL_SUB_TUTORIAL plan_to_pose
## CALL_SUB_TUTORIAL plan_cartesian_path
## CALL_SUB_TUTORIAL display_trajectory
## CALL_SUB_TUTORIAL execute_plan
## CALL_SUB_TUTORIAL add_box
## CALL_SUB_TUTORIAL wait_for_scene_update
## CALL_SUB_TUTORIAL attach_object
## CALL_SUB_TUTORIAL detach_object
## CALL_SUB_TUTORIAL remove_object
## END_TUTORIAL
