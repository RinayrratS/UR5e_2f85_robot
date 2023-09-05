#!/usr/bin/env python

import sys
import copy
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf2_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import franka_gripper.msg
import actionlib
import numpy as np
import csv
import os
def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class PoseLevelMotion(object):
  def __init__(self):
    super(PoseLevelMotion, self).__init__()
    
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('0105_move_group', anonymous=True) #node name

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    self.robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    self.scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    self.group_name = "manipulator"
    self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

  

    
    self.grasp_name = "panda_hand"
    self.grasp_state = False
    self.hand_group = moveit_commander.MoveGroupCommander(self.grasp_name)

    # Joint
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
    # self.joint.append(moveit_commander.RobotCommander.Joint(
    #     self.robot, 'panda_joint7'))
    # self.joint.append(moveit_commander.RobotCommander.Joint(
    #     self.robot, 'panda_finger_joint1'))
    # self.joint.append(moveit_commander.RobotCommander.Joint(
    #     self.robot, 'panda_finger_joint2'))
    
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
    print("============ Available Planning Groups:", self.group_names) 

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print ("============ Printing robot state")
    print (self.robot.get_current_state())
    print (self.move_group.get_current_pose())
    print ("")
    self.obj_to_idx = {"hand": 0,
                       "box1": 1,
                       "box2": 2,
                       "box3": 3,
                       "box4": 4,
                       "box5": 5,
                       "bowl1": 6,
                       "bowl2": 7,
                       "table": 8,}





  def add_object(self, object_name, object_pose, object_mesh, object_size, frame_id="world"): #object_mesh x?
    obj_pose = geometry_msgs.msg.PoseStamped()
    obj_pose.header.frame_id = frame_id
    obj_pose.pose = list_to_pose(object_pose)
    self.scene.add_mesh(object_name, obj_pose, object_mesh, object_size)
    
  def get_object_pose(self, object_name): 
    try:
      obj_pose = self.scene.get_object_poses([object_name])
      obj_pose = obj_pose.values()[0]        
      return obj_pose
    except Exception as e:
      rospy.logerr('cannot get object pose: '+object_name)
      rospy.logerr(e)
      obj_pose = []
      return obj_pose
  def get_object_poses(self): 
    obj_poses = dict()
    co = self.scene.get_objects() 
    for key, val in co.items():
      obj_poses[key] = val.mesh_poses[0]
    return obj_poses
  
  def update_object_pose(self, object_name, object_pose): 
    obj_pose = geometry_msgs.msg.PoseStamped()
    obj_pose.header.frame_id = "world"
    obj_pose.pose = object_pose
    self.scene.update_pose(object_name, obj_pose)
    
    
  def hold_object(self, object_list, grasp_size):
    try:
      robot = self.robot
      hand_group = self.hand_group
      eef_link = self.eef_link
      touch_links = robot.get_link_names(group=self.grasp_name)
      hand_group.set_joint_value_target([grasp_size, 0])
      hand_group.go()
      for object_name in object_list:
        hand_group.attach_object(
          object_name, eef_link, touch_links=touch_links)
        time.sleep(0.1)
      self.grasp_state = True
      hold_status = True

      # info
      mp_info = dict()
      mp_info['planning_time'] = None
      mp_info['execution_time'] = None
      mp_info['success'] = hold_status

      return hold_status, mp_info

    except Exception as e:
      rospy.logerr(e)
      hold_status = False

      # info
      mp_info = dict()
      mp_info['planning_time'] = None
      mp_info['execution_time'] = None
      mp_info['success'] = hold_status

      return hold_status, mp_info

  def release_object(self, object_list):
    try:
      hand_group = self.hand_group
      for object_name in object_list:
        hand_group.detach_object(object_name)
        #time.sleep(0.1)
      hand_group.set_joint_value_target([0.04, 0])
      hand_group.go()
      self.grasp_state = False
      release_status = True

      # info
      mp_info = dict()
      mp_info['planning_time'] = None
      mp_info['execution_time'] = None
      mp_info['success'] = release_status

      return release_status, mp_info

    except Exception as e:
      rospy.logerr(e)
      release_status = False

      # info
      mp_info = dict()
      mp_info['planning_time'] = None
      mp_info['execution_time'] = None
      mp_info['success'] = release_status

      return release_status, mp_info    
    
  def add_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = 'table'
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene between the fingers:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.z = 0.27
    box_pose.pose.position.x = 0.4  # above the panda_hand frame
    box_name = "table"
    scene.add_box(box_name, box_pose, size=(0.6, 1.2, 0.27))

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    self.box_name = box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)
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


  def go_to_joint_state(self,joint_goal):
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    
    #joint_goal = joint_angle_list
    #joint_goal[0] = pi/2
    #joint_goal[1] = -pi/4
    #joint_goal[2] = 0
    #joint_goal[3] = -pi/2
    #joint_goal[4] = 0
    #joint_goal[5] = pi/3
    #joint_goal[6] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    self.move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    self.move_group.stop()

    # For testing:
    current_joints = self.move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal_6d(self,px,py,pz,rot_x, rot_y, rot_z):
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()

    q_rot = quaternion_from_euler(rot_x, rot_y, rot_z, axes='sxyz')

    pose_goal.position.x = px
    pose_goal.position.y = py
    pose_goal.position.z = pz
    pose_goal.orientation.x = q_rot[0]
    pose_goal.orientation.y = q_rot[1]
    pose_goal.orientation.z = q_rot[2]
    pose_goal.orientation.w = q_rot[3]
    
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
    current_pose = self.move_group.get_current_pose().pose
    print (current_pose)

    return all_close(pose_goal, current_pose, 0.01)
  

  def go_to_pose_goal_from_sim(self, target_object):
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    sub_flag = True
    pose_goal = geometry_msgs.msg.Pose()
    '''
    rate = rospy.Rate(10)
    while sub_flag:
      rospy.Subscriber(target_object+'_pose', geometry_msgs.msg.Pose, self.position_callback)
      sub_flag=False
    '''
    msg = rospy.wait_for_message(target_object+'_pose', geometry_msgs.msg.Pose, timeout=None)
    self.position_callback(msg)  

  
  def position_callback(self, object):
    current_pose = self.move_group.get_current_pose().pose
    cur_qx = current_pose.orientation.x
    cur_qy = current_pose.orientation.y
    cur_qz = current_pose.orientation.z
    cur_qw = current_pose.orientation.w
    cur_rx, cur_ry, cur_rz = euler_from_quaternion([cur_qx, cur_qy, cur_qz, cur_qw])
    pose_goal = geometry_msgs.msg.Pose()

    pose_goal.position.x = object.transform.translation.x
    pose_goal.position.y = object.transform.translation.y
    pose_goal.position.z = object.transform.translation.z

    qx= object.transform.rotation.x
    qy= object.transform.rotation.y
    qz= object.transform.rotation.z
    qw= object.transform.rotation.w

    rx, ry, rz = euler_from_quaternion([qx, qy, qz, qw])
    rx = pi
    ry = 0
    rz -= pi/4
    '''
    while((cur_rz-rz)*180/pi>=-90 and (cur_rz-rz)*180/pi<=90):
      rz += (pi/2)
    '''
    new_q = quaternion_from_euler(rx, ry, rz)
    pose_goal.orientation.x = new_q[0]
    pose_goal.orientation.y = new_q[1]
    pose_goal.orientation.z = new_q[2]
    pose_goal.orientation.w = new_q[3]


    pose_goal.position.z = 0.18 + 0.1 + 0.2

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
    current_pose = self.move_group.get_current_pose().pose
    print (current_pose)
    return all_close(pose_goal, current_pose, 0.01)

  def grasp_client(self):
    # Creates the SimpleActionClient, passing the type of the action
    # (GraspAction) to the constructor.
    client = actionlib.SimpleActionClient('/franka_gripper/grasp', franka_gripper.msg.GraspAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = franka_gripper.msg.GraspGoal()
    goal.width = 0.022
    goal.epsilon.inner = 0.005
    goal.epsilon.outer = 0.005
    goal.speed = 0.1
    goal.force = 5

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A GraspResult
  
  def grasp(self,grasp_size=0.018, epsilon = 0.003):
      try:
          robot = self.robot 
          hand_group = self.hand_group 
          eef_link = self.eef_link
          hand_group.set_joint_value_target([grasp_size, 10000])
          hand_group.set_goal_joint_tolerance(epsilon)
          hand_group.go()
          
          print(hand_group.get_current_pose())
      except Exception as e:
          rospy.logerr(e)
  
  def _go_to_pose_goal(self, pose_goal):
    move_success = False
    while move_success is False:
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
      current_pose = self.move_group.get_current_pose().pose
      time.sleep(3)
      print (current_pose)
      move_success = all_close(pose_goal, current_pose, 0.005)

  def _linear_motion(self, dir, length):
    
    current_pose = self.move_group.get_current_pose().pose
    
    pose_goal = current_pose
    
    if dir=='x':
      pose_goal.position.x += length
    elif dir=='y':
      pose_goal.position.y += length
    elif dir=='z':
      pose_goal.position.z += length

    move_success = False
    while move_success is False:
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
      time.sleep(3)
      print (current_pose)
      move_success = all_close(pose_goal, current_pose, 0.005)
  def go_to_pose_goal(self, pose_goal):
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
    current_pose = self.move_group.get_current_pose().pose
    time.sleep(2)
    print (current_pose)
    print(all_close(pose_goal, current_pose, 0.005))

  def linear_motion(self, dir, length):
    
    current_pose = self.move_group.get_current_pose().pose
    
    pose_goal = current_pose
    
    if dir=='x':
      pose_goal.position.x += length
    elif dir=='y':
      pose_goal.position.y += length
    elif dir=='z':
      pose_goal.position.z += length

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
    print (current_pose)
    print(all_close(pose_goal, current_pose, 0.005))

  def Pick(self, target_obj):
    #go to pre_grasp_pose
    self.go_to_pose_goal(self.get_pre_grasp_pose(target_obj))
    #down to grasp_pose
    self.linear_motion('z', -0.09)
    self.linear_motion('x', +0.0025)
    time.sleep(2)
    #grasp execute
    self.grasp(grasp_size=0.015, epsilon=0.001)
    #up to pose_grasp_pose
    #self.linear_motion('z', +0.09)
    self.linear_motion('z', +0.2)
    #go back to initial pose
    #self.go_to_pose_goal_6d(0.25, 0, 0.5, pi, 0, -pi/4)

  def Place(self, dest_obj):
    #go to pre_grasp_pose
    self.go_to_pose_goal(self.get_pre_place_pose(dest_obj))
    #down to grasp_pose
    #self.linear_motion('z', -0.0475)
    self.linear_motion('z', -0.2475)
    time.sleep(2)
    #grasp execute
    self.grasp(grasp_size=0.04)
    #up to pose_grasp_pose
    self.linear_motion('z', +0.0475)
  
  def Pick_and_Place(self, target_obj, dest_obj):
    #go back to initial pose
    self.go_to_pose_goal_6d(0.25, 0, 0.5, pi, 0, -pi/4)
    self.grasp(grasp_size=0.04)

    self.Pick(target_obj)
    self.Place(dest_obj)

  def Stacking_5(self):
    #self.random_box_order()
    print(self.obj_to_idx)
    self.Pick_and_Place("box4", "box5")
    self.Pick_and_Place("box3", "box4")
    self.Pick_and_Place("box2", "box3")
    self.Pick_and_Place("box1", "box2")

  def get_pre_grasp_pose(self, target_obj):
    pose_goal = self.get_pose_from_sim(target_obj)
    
    pose_goal.position.z += 0.2
    return pose_goal
  
  def get_pre_place_pose(self, dest_obj):
    pose_goal = self.get_pose_from_sim(dest_obj)
    
    #pose_goal.position.z += 0.2
    pose_goal.position.z += 0.4
    return pose_goal

  def get_pose_from_sim(self, target_object):
    pose_goal = tf2_msgs.msg.TFMessage()

    tf_msg = rospy.wait_for_message('obj_pose', tf2_msgs.msg.TFMessage, timeout=None)
    tf_list = tf_msg.transforms

    return self.get_obj_pose(tf_list[self.obj_to_idx[target_object]])
  
  def get_obj_pose(self, object):
    current_pose = self.move_group.get_current_pose().pose
    cur_qx = current_pose.orientation.x
    cur_qy = current_pose.orientation.y
    cur_qz = current_pose.orientation.z
    cur_qw = current_pose.orientation.w
    cur_rx, cur_ry, cur_rz = euler_from_quaternion([cur_qx, cur_qy, cur_qz, cur_qw])

    pose_goal = geometry_msgs.msg.Pose()

    pose_goal.position.x = object.transform.translation.x
    pose_goal.position.y = object.transform.translation.y
    pose_goal.position.z = object.transform.translation.z - 0.375

    qx= object.transform.rotation.x
    qy= object.transform.rotation.y
    qz= object.transform.rotation.z
    qw= object.transform.rotation.w

    rx, ry, rz = euler_from_quaternion([qx, qy, qz, qw])
    rx = pi
    ry = 0
    rz -= pi/4
    if rz>pi/2:
      rz -= pi
    elif rz<-pi/2:
      rz += pi
    if rz>pi/4:
      rz-=pi/2
    elif rz<-pi/4:
      rz+=pi/2
    '''
    while((cur_rz-rz)*180/pi>=-90 and (cur_rz-rz)*180/pi<=90):
      rz += (pi/2)
    '''
    new_q = quaternion_from_euler(rx, ry, rz)
    pose_goal.orientation.x = new_q[0]
    pose_goal.orientation.y = new_q[1]
    pose_goal.orientation.z = new_q[2]
    pose_goal.orientation.w = new_q[3]

    return pose_goal
  
  def random_box_order(self):
    random_order = np.arange(0,5)
    np.random.shuffle(random_order)
    for i in range(5):
      key = str("box"+str(i+1))
      self.obj_to_idx[key] = random_order[i]
  
  def save_obj_pose(self, data_dir, i):
    time.sleep(0.5)
    tf_msg = rospy.wait_for_message('obj_pose', tf2_msgs.msg.TFMessage, timeout=None)
    tf_list = tf_msg.transforms
    pose_data = []

    for tf in tf_list:
      pose_data.append(self.pose_msg_to_6d(tf))



    f = open(data_dir+'/pose_data_'+str(i)+'.csv', 'w')
    writer = csv.writer(f)
    writer.writerows(pose_data)
    f.close
  
  def pose_msg_to_6d(self, pose_msg):
    x = pose_msg.transform.translation.x
    y = pose_msg.transform.translation.y
    z = pose_msg.transform.translation.z

    qx = pose_msg.transform.rotation.x
    qy = pose_msg.transform.rotation.y
    qz = pose_msg.transform.rotation.z
    qw = pose_msg.transform.rotation.w
    rx, ry, rz = euler_from_quaternion([qx, qy, qz, qw])
    return [x, y, z, rx, ry, rz]

  def stacking_5_with_pose_save(self):
    print(self.obj_to_idx)
    data_dir = '/home/byeon/cooking_branches/byeon_8/cooking/seq_dataset/tasks/stacking_5/pose/demo_'
    demo_idx = 0
    while os.path.exists(data_dir + str(demo_idx)):
      demo_idx += 1
    data_dir = data_dir + str(demo_idx)
    os.makedirs(data_dir)

    self.save_obj_pose(data_dir, 0)
    ##########################
    self.go_to_pose_goal_6d(0.25, 0, 0.5, pi, 0, -pi/4)
    self.grasp(grasp_size=0.04)

    self.Pick('box4')
    self.save_obj_pose(data_dir, 1)
    self.Place('box5')
    self.save_obj_pose(data_dir, 2)

    self.check_success('box4', 'box5')
    ##########################
    self.go_to_pose_goal_6d(0.25, 0, 0.5, pi, 0, -pi/4)
    self.grasp(grasp_size=0.04)

    self.Pick('box3')
    self.save_obj_pose(data_dir, 3)
    self.Place('box4')
    self.save_obj_pose(data_dir, 4)
    self.check_success('box3', 'box4')
    ##########################
    self.go_to_pose_goal_6d(0.25, 0, 0.5, pi, 0, -pi/4)
    self.grasp(grasp_size=0.04)

    self.Pick('box2')
    self.save_obj_pose(data_dir, 5)
    self.Place('box3')
    self.save_obj_pose(data_dir, 6)
    self.check_success('box2', 'box3')
    ##########################
    self.go_to_pose_goal_6d(0.25, 0, 0.5, pi, 0, -pi/4)
    self.grasp(grasp_size=0.04)

    self.Pick('box1')
    self.save_obj_pose(data_dir, 7)
    self.Place('box2')
    self.save_obj_pose(data_dir, 8)
    self.check_success('box1', 'box2')

    #check
    box1_pose = self.get_pose_from_sim("box1")
    #print("box1 pose:", box1_pose)
    if box1_pose.position.z >= (0.72-0.375):
      return True
    else:
      return False

  def check_success(self, pick_box, place_box):
    pick_box_pose = self.get_pose_from_sim(pick_box)
    place_box_pose = self.get_pose_from_sim(place_box)
    height_diff = pick_box_pose.position.z - place_box_pose.position.z
    low_b = 0.04-0.0002
    up_b = 0.04+0.0002
    if (height_diff>=low_b) and (height_diff<=up_b):
      return True
    else:
      return False
      






def main():
  try:
    print ("")
    print ("----------------------------------------------------------")
    print ("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
    print ("----------------------------------------------------------")
    print ("Press Ctrl-D to exit at any time")
    print ("")
    print ("============ Press `Enter` to begin the tutorial by setting up the moveit_commander ...")
    time.sleep(1)
    pose_level_motion = PoseLevelMotion()
    #pose_level_motion.add_box()
    print ("============ Press `Enter` to execute Pick_and_Place ...")
    input()
    #pose_level_motion.Stacking_5()
    pose_level_motion.stacking_5_with_pose_save()



    print ("============ Press `Enter` to execute a movement using a pose goal ...")
    input()
    pose_level_motion.go_to_pose_goal(0.25, 0, 0.5, pi, 0, -pi/4)
    pose_level_motion.grasp(grasp_size=0.03, epsilon=1)
    print ("============ Press `Enter` to execute a movement using a pose goal ...")
    input()
    #pose_level_motion.go_to_pose_goal_from_sim('box1')
    pose_level_motion.get_pose_from_sim('box3')
    print ("============ z linear motion ...")
    input()
    pose_level_motion.linear_motion('z', -0.09)
    print ("============ z linear motion ...")
    input()

    pose_level_motion.grasp(grasp_size=0.015, epsilon=0.001)

    print ("============ z linear motion ...")
    input()
    pose_level_motion.linear_motion('z', +0.09)
    print ("============ Press `Enter` to execute a movement using a pose goal ...")
    input()

    pose_level_motion.go_to_pose_goal(0.25, 0, 0.6, pi, 0, -pi/4)
    print ("============ Press `Enter` to execute a movement using a pose goal ...")
    input()
    #pose_level_motion.go_to_pose_goal_from_sim('box4')
    pose_level_motion.get_pose_from_sim('box1')
    print ("============ z linear motion ...")
    input()
    pose_level_motion.linear_motion('z', -0.0475)
    print ("============ z linear motion ...")
    input()

    pose_level_motion.grasp(grasp_size=0.04)

    print ("============ z linear motion ...")
    input()
    pose_level_motion.linear_motion('z', +0.0475)
    
    
    
    print ("============ Press `Enter` to execute a movement using a pose goal ...")
    input()
    pose_level_motion.go_to_pose_goal(0.25, 0, 0.5, pi, 0, -pi/4)
    pose_level_motion.grasp(grasp_size=0.03, epsilon=1)
    print ("============ Press `Enter` to execute a movement using a pose goal ...")
    input()
    #pose_level_motion.go_to_pose_goal_from_sim('box1')
    pose_level_motion.get_pose_from_sim('box2')
    print ("============ z linear motion ...")
    input()
    pose_level_motion.linear_motion('z', -0.09)
    print ("============ z linear motion ...")
    input()

    pose_level_motion.grasp(grasp_size=0.015, epsilon=0.001)

    print ("============ z linear motion ...")
    input()
    pose_level_motion.linear_motion('z', +0.09)
    print ("============ Press `Enter` to execute a movement using a pose goal ...")
    input()

    pose_level_motion.go_to_pose_goal(0.25, 0, 0.5, pi, 0, -pi/4)
    print ("============ Press `Enter` to execute a movement using a pose goal ...")
    input()
    #pose_level_motion.go_to_pose_goal_from_sim('box4')
    pose_level_motion.get_pose_from_sim('box3')
    print ("============ z linear motion ...")
    input()
    pose_level_motion.linear_motion('z', -0.0475)
    print ("============ z linear motion ...")
    input()

    pose_level_motion.grasp(grasp_size=0.04)

    print ("============ z linear motion ...")
    input()
    pose_level_motion.linear_motion('z', +0.0475)

    print ("============ Press `Enter` to execute a movement using a pose goal ...")
    input()
    pose_level_motion.go_to_pose_goal(0.25, 0, 0.5, pi, 0, -pi/4)
    pose_level_motion.grasp(grasp_size=0.03, epsilon=1)
    print ("============ Press `Enter` to execute a movement using a pose goal ...")
    input()
    #pose_level_motion.go_to_pose_goal_from_sim('box1')
    pose_level_motion.get_pose_from_sim('box4')
    print ("============ z linear motion ...")
    input()
    pose_level_motion.linear_motion('z', -0.09)
    print ("============ z linear motion ...")
    input()

    pose_level_motion.grasp(grasp_size=0.015, epsilon=0.001)

    print ("============ z linear motion ...")
    input()
    pose_level_motion.linear_motion('z', +0.09)
    print ("============ Press `Enter` to execute a movement using a pose goal ...")
    input()

    pose_level_motion.go_to_pose_goal(0.25, 0, 0.5, pi, 0, -pi/4)
    print ("============ Press `Enter` to execute a movement using a pose goal ...")
    input()
    #pose_level_motion.go_to_pose_goal_from_sim('box4')
    pose_level_motion.get_pose_from_sim('box2')
    print ("============ z linear motion ...")
    input()
    pose_level_motion.linear_motion('z', -0.0475)
    print ("============ z linear motion ...")
    input()

    pose_level_motion.grasp(grasp_size=0.04)

    print ("============ z linear motion ...")
    input()
    pose_level_motion.linear_motion('z', +0.0475)






    print ("============ z linear motion ...")
    input()
    pose_level_motion.linear_motion('z', -0.01)
    print ("============ z linear motion ...")
    input()
    pose_level_motion.linear_motion('z', -0.01)
    print ("============ z linear motion ...")
    input()
    pose_level_motion.linear_motion('z', -0.01)
    print ("============ z linear motion ...")
    input()
    pose_level_motion.linear_motion('z', -0.01)
    print ("============ z linear motion ...")
    input()
    pose_level_motion.linear_motion('z', -0.01)
    print ("============ z linear motion ...")
    input()
    pose_level_motion.linear_motion('z', -0.01)
    print ("============ z linear motion ...")
    input()
    pose_level_motion.linear_motion('z', -0.01)
    print ("============ z linear motion ...")
    input()
    pose_level_motion.linear_motion('z', -0.01)
    print ("============ z linear motion ...")
    input()
    pose_level_motion.linear_motion('z', -0.01)
    print ("============ z linear motion ...")
    input()
    pose_level_motion.linear_motion('z', -0.01)
    print ("============ z linear motion ...")
    input()
    pose_level_motion.linear_motion('z', -0.01)
    print ("============ z linear motion ...")
    input()
    pose_level_motion.linear_motion('z', -0.01)
    print( "============ x linear motion ...")
    input()
    pose_level_motion.linear_motion('x', 0.01)
    pose_level_motion.linear_motion('x', -0.01)
    pose_level_motion.linear_motion('x', 0.01)
    pose_level_motion.linear_motion('x', -0.01)
    print ("============ y linear motion ...")
    input()
    pose_level_motion.linear_motion('y', 0.01)
    pose_level_motion.linear_motion('y', -0.01)
    pose_level_motion.linear_motion('y', 0.01)
    pose_level_motion.linear_motion('y', -0.01)

    
    print ("============ Press `Enter` to execute a movement using a pose goal ...")
    input()
    pose_level_motion.go_to_pose_goal(0.4, 0.0, 0.75, 0.0)
    
    print ("============ Press `Enter` to execute a movement using a pose goal ...")
    input()
    pose_level_motion.go_to_pose_goal(0.6, 0.0, 0.75, 0.0)
    print ("============ Press `Enter` to execute a movement using a pose goal ...")
    input()
    pose_level_motion.go_to_pose_goal(0.4, 0.0, 0.75, 0.0)
    print ("============ Press `Enter` to execute a movement using a pose goal ...")
    input()
    pose_level_motion.go_to_pose_goal(0.4, 0.0, 0.75, 1.0)

    print ("============ Press `Enter` to execute a movement using a pose goal ...")
    input()
    pose_level_motion.go_to_pose_goal(0.4, 0.0, 0.75, -1.0)

    print ("============ Python tutorial demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return
  


if __name__ == '__main__':
  main()

## BEGIN_TUTORIAL
## .. _moveit_commander:
##    http://docs.ros.org/melodic/api/moveit_commander/html/namespacemoveit__commander.html
##
## .. _MoveGroupCommander:
##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
## .. _RobotCommander:
##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _PlanningSceneInterface:
##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
## .. _DisplayTrajectory:
##    http://docs.ros.org/melodic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
## .. _RobotTrajectory:
##    http://docs.ros.org/melodic/api/moveit_msgs/html/msg/RobotTrajectory.html
##
## .. _rospy:
##    http://docs.ros.org/melodic/api/rospy/html/
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
