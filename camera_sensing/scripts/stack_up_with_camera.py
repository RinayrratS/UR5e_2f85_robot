<<<<<<< HEAD
#!/usr/bin/env python
=======
#!/usr/bin/env python3
>>>>>>> getting depth data

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input
from get_data import Depth_Camera
<<<<<<< HEAD
=======
from detect_simple import DetectionResult
>>>>>>> getting depth data

import sys
import copy
import rospy
import math
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import robotiq_gripper_msgs.msg
import time
import actionlib
import cv2
import pyrealsense2 as rs
import numpy as np
import threading

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
    pub: object
    move_group: object

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
        ## arm joints in the ur5e robot, so we set the group's name to "arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

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
        self.rate = rospy.Rate(40)  # 10hz
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
        print(self.move_group.get_current_pose())
        ## END_SUB_TUTORIAL


    def go_to_home_state(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.

        # Put the arm in the start position
        self.move_group.set_max_velocity_scaling_factor(0.5)  # Adjust the value as needed
        #self.move_group.set_named_target("up")
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = math.radians(0)
        joint_goal[1] = math.radians(-90)
        joint_goal[2] = math.radians(90)
        joint_goal[3] = math.radians(-90)
        joint_goal[4] = math.radians(-90)
        joint_goal[5] = math.radians(0)

        success = self.move_group.go(joint_goal,wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        self.move_group.clear_pose_targets()
        current_joint = self.move_group.get_current_joint_values()
        time.sleep(2)
        print(current_joint)
        print(all_close(joint_goal, current_joint, 0.01))


    def go_to_pose_goal(self):
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

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        time.sleep(2)
        print(current_pose)
        print(all_close(pose_goal, current_pose, 0.005))


    def go_to_goal_state(self, height: object) -> object:

        pose_goal = self.move_group.get_current_pose().pose

        pose_goal.position.x = 0.5
        pose_goal.position.y = 0.3
        pose_goal.position.z = height


        print("============ Goal position")
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


    def pick_and_place(self, xaxis: object, yaxis: object, zaxis: object, goal_zaxis: object) -> object:

        self.linear_motion_absolute(xaxis, yaxis, goal_zaxis + 0.15)
<<<<<<< HEAD
        time.sleep(1)
        self.linear_motion_absolute(xaxis, yaxis, zaxis)
        self.gripper_close()
        time.sleep(1)
        self.linear_motion_absolute(xaxis, yaxis, goal_zaxis + 0.15)
        initial_pose = self.move_group.get_current_pose().pose
=======
        self.linear_motion_absolute(xaxis, yaxis, zaxis)
        self.gripper_close()
        self.linear_motion_absolute(xaxis, yaxis, goal_zaxis + 0.15)
        #initial_pose = self.move_group.get_current_pose().pose
>>>>>>> getting depth data

        self.go_to_goal_state(goal_zaxis + 0.15)
        self.go_to_goal_state(goal_zaxis+0.0015)
        self.gripper_open()
<<<<<<< HEAD
        time.sleep(1)
        self.go_to_goal_state(goal_zaxis + 0.15)
=======
        self.go_to_goal_state(goal_zaxis + 0.15)
        time.sleep(2)
>>>>>>> getting depth data

        #final_pose = self.move_group.get_current_pose().pose
        #if initial_pose.position.x == length1 and initial_pose.position.y == length2:



    def stack(self, rows: object, columns: object) -> object:

        row_num = 0
        column_num = 0
        count = 0
        box_height = 0.04
        goal_height = 0.78

        # initial axis
        row_length = 0.5

        while row_num < rows:
            column_num = 0
            column_length = -0.1
            row_length += row_num * 0.1

            while column_num < columns:
                column_length += column_num * 0.1

                self.pick_and_place(row_length, column_length, 0.78, goal_height)
                column_num += 1
                count += 1
                print(count, "block pick and place complete!")
                goal_height = 0.78 + (count * box_height)
                input("============ Press `Enter` to move on")
            row_num += 1


<<<<<<< HEAD
=======
    def yolo_stack(self, x_axis: object, y_axis: object, z_axis: object, goal_z: object) -> object:

        row_num = 0
        column_num = 0
        count = 0
        box_height = 0.04
        goal_height = 0.78

        # initial axis
        row_length = 0.5

        while row_num < rows:
            column_num = 0
            column_length = -0.1
            row_length += row_num * 0.1

            while column_num < columns:
                column_length += column_num * 0.1

                self.pick_and_place(row_length, column_length, 0.78, goal_height)
                column_num += 1
                count += 1
                print(count, "block pick and place complete!")
                goal_height = 0.78 + (count * box_height)
                input("============ Press `Enter` to move on")
            row_num += 1
>>>>>>> getting depth data


    def execute_data():
        # while 추가해야함

        depth_camera = Depth_Camera()
        depth_camera.execute()

<<<<<<< HEAD
=======

>>>>>>> getting depth data
    def get_depth(self):
        depth_camera = Depth_Camera()
        self.depth_data = depth_camera.get_depth_data()
        return self.depth_data

<<<<<<< HEAD
    def yolo_stacking(self):
        row_axis=
        column_axis=
        current_zaxis=
        goal_zaxis=
        box_height = 0.04
        block_count=
        count = 0
        depth_data = self.get_depth()

        while(block_count!=0):
            self.pick_and_place(row_axis, column_axis, current_zaxis-depth_data, goal_zaxis)
            goal_zaxis = 0.78 + (count * box_height)
=======

    def yolo_stacking(self):

        detect = DetectionResult()
        opt = detect.parse_opt()
        detect.detect_simple(**vars(opt))
        block_count = detect.count
        block_list = detect.blocks
        print("block_list: ", block_list)
        #
        i = 0
        while i < block_count:
            x = block_list[2 * i + 1]
            y = block_list[2 * i] - 0.1
            z = 0.78
            box_height = 0.04
            goal_z = 0.78 + (box_height * i)
            print("axis: ", x, y, z, goal_z)
            self.pick_and_place(x, y, z, goal_z)            
            i += 1  
        #return



def yolo_stacking():
    detect = DetectionResult()
    opt = detect.parse_opt()
    detect.main(opt)
    #time.sleep(10)
    #detect.main(opt)
    block_count = detect.count
    block_axis = detect.blocks
    print(block_axis[0], block_axis[1],"hi")
    row_axis = detect.center_x
    column_axis = detect.center_y
    print(row_axis, column_axis)
    #current_pose = self.move_group.get_current_pose().pose
    #current_zaxis = current_pose.position.z
    goal_zaxis = 0.78
    box_height = 0.04
    #block_count = detect.count
    count = 0
    #depth_data = self.get_depth()

    # while(block_count!=0):
    #     #self.pick_and_place(row_axis, column_axis, current_zaxis-depth_data, goal_zaxis)
    #     goal_zaxis = 0.78 + (count * box_height)
>>>>>>> getting depth data



def main():
    try:
        print("")
        print("-----------------------------------")
        print("UR5e with robotiq_85_gripper robot")
        print("-----------------------------------")
        print("")

        #depth_camera = Depth_Camera()
        # execute 메서드를 호출하여 데이터 수집 시작
        #depth_camera.execute()
        
<<<<<<< HEAD
        get_depth()
=======
        #get_depth()
>>>>>>> getting depth data

        input(
            "============ Press `Enter` to initialize the pose of robot ..."
        )
        tutorial = MoveGroupPythonInterfaceTutorial()

        tutorial.go_to_home_state()
        tutorial.gripper_open()

        input("============ Press `Enter` to stack...")
<<<<<<< HEAD
        tutorial.stack(3,2)

=======
        #tutorial.get_axis()
        tutorial.pick_and_place(0.7,0.1,0.78,0.78)
>>>>>>> getting depth data


        print("============ Python tutorial demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
