# robotiq_gripper_ros
ROS Package for RobotiQ Gripper


## View Gripper using rviz

```
$ roslaunch robotiq_gripper_description view_gripper.launch
```

## Execute Gripper Controller

```
$ rosrun robotiq_gripper_controller robotiq_gripper_controller_node _port_name:=<device name>
```

## Topic

### /gripper_status
```
robotiq_gripper_msgs/GripperStatus

uint8 g_act             // Gripper Activated
uint8 g_gto             // Gripper Go To : 0 - Stop, 1 - Moving
uint8 g_sta             // Gripper Staus : 0 - in reset, 1 - Activating, 3 - Activated
uint8 g_obj             // Object Detection : 0 - not detected, 1 - detected opening, 2 - detected closing, 3 - no object
uint8 g_flt             // Gripper Fault
uint8 g_pr              // Position Requested
uint8 g_po              // Actual Position
uint8 g_cu              // Current
```

### /gripper_command
```
robotiq_gripper_msgs/GripperCommand

float32 position        // Gripper Position [0:1] : 0 - Open, 1 - Closed
float32 speed           // Gripper Speed [0:1]
float32 force           // Gripper Force [0:1]
```