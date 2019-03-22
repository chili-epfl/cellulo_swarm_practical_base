# ROS Cellulo API - Application Package

## Overview

This will be the main package for ROS Cellulo Swarm, it has two types of nodes:  
  
- ros_cellulo_reduced: this is a reduced version of the main RosCellulo API where no access to directly setting the final poses.  
- ros_cellulo_sensor: emulates proximity sensors for the cellulo, it calculates the distances to all other present robots, returns the number of robots within a specifed threshold (+ the distances, and their relative positions). 

#### Dependencies

- [Robot Operating System (ROS)],
- [Qt] (QT_LIBRARIES Gui, Widgets , Quick , Bluetooth)


#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

```
cd catkin_ws/src
git clone https://c4science.ch/diffusion/5556/ros_cellulo.git
cd ..  
catkin build
```
## Usage

```
roslaunch ros_cellulo_swarm ros_cellulo_swarm.launch
```

## Nodes

### type: ros_cellulo_reduced
#### Published Topics

* **`/cellulo_node_xx_xx/TouchKey`** ([ros_cellulo/cellulo_touch_key]) Array of 6 booleans published when one touch key is pressed

* **`/cellulo_node_xx_xx/LongTouchKey`** ([ros_cellulo/cellulo_touch_key]) Array of 6 booleans published when one touch key is long pressed
* **`/cellulo_node_xx_xx/Kidnapped`** ([ros_cellulo/cellulo_kidnapped_msg]) Message with the cellulo address and the kidnapped value. Published when kidnapped state change on the robot.

* **`/cellulo_node_xx_xx/ConnectionStatus`** ([std_msgs/Int8]) Connection status, published when ConnectionStatus is changed.

* **`/cellulo_node_xx_xx/LocalAdapterAdress`** ([std_msgs/String])	Local Adapter Address, published when it is changed  
* **`/cellulo_node_xx_xx/autoConnect`** ([std_msgs/Bool])	autoConnect state of the cellulo, published when it is changed

* **`/cellulo_node_xx_xx/macAddress`** ([std_msgs/String])	Mac Address, published when it is changed

* **`/cellulo_node_xx_xx/BatteryState`** ([std_msgs/Int8])	Battery State, published when it is changed
* **`/cellulo_node_xx_xx/Gesture`** ([std_msgs/Int8])	Gesture, published when it is changed

* **`/cellulo_node_xx_xx/lastTimestamp`** ([std_msgs/Int8])	Last time stamp, published when it is changed

* **`/cellulo_node_xx_xx/framerate`** ([std_msgs/Float32])	Frame rate,  published when it is changed
* **`/cellulo_node_xx_xx/cameraImageProgress`** ([std_msgs/Float32])	Camera image progess, published when it is changed
* **`/cellulo_node_xx_xx/Vx`** ([std_msgs/Float64])	Velocity Vx, published when it is changed
* **`/cellulo_node_xx_xx/Vy`** ([std_msgs/Float64])	Velocity Vy, published when it is changed

* **`/cellulo_node_xx_xx/w`** ([std_msgs/Float64])	rotational velocity w, published when it is changed

* **`/cellulo_node_xx_xx/poseVelControlEnable`** ([std_msgs/Bool])	Flag poseVelControlEnable, published when it is changed

* **`/cellulo_node_xx_xx/poseVelControlPeriod`** ([std_msgs/Int32])	poseVelControlPeriod, published when it is changed

* **`/cellulo_node_xx_xx/visualization_marker_traj`** ([visualization_msgs/Marker])	Visualization marker for the trajectory (for rviz)
* **`/cellulo_node_xx_xx/visualization_marker_robot`** ([visualization_msgs/Marker])	Visualization marker for the robot (for rviz)

* **`A TransformBroadcaster`** ([Frame: /xx:xx Parent: /paper_world])	Broadcasts the robots position, each time it is changed (as a slot for the signal poseChanged)



#### Subscribed Topics

* **`/cellulo_node_xx_xx/Set_Visual_Effect`** ([ros_cellulo/cellulo_visual_effect])	Sets the visual effect of the robot.
* **`/cellulo_node_xx_xx/Reset`** ([std_msgs/Empty])	Resets the robot
* **`/cellulo_node_xx_xx/setGoalVelocity`** ([geometry_msgs::Vector3])	Follow a given direction with a given speed (i.e. sets Vx and Vy)
* **`/cellulo_node_xx_xx/Shutdown`** ([std_msgs/Empty])	Shuts down the robot

* **`/cellulo_node_xx_xx/ClearTracking`** ([std_msgs/Empty])	Clears the pose/position/velocity/trajectory goals

* **`/cellulo_node_xx_xx/ClearHapticFeedback`** ([std_msgs/Empty])	Clears the haptic feedback

* **`/cellulo_node_xx_xx/vibrateOnMotion`** ([ros_cellulo/cellulo_vibrateOnMotion])	Sets the vibration on motion with iCoef for a given period

* **`/cellulo_node_xx_xx/setGestureEnabled`** ([std_msgs/Bool])	Sets setGestureEnabled

* **`/cellulo_node_xx_xx/setCasualBackdriveAssistEnabled`** ([std_msgs/Bool])	Sets setCasualBackdriveAssistEnabled

#### Services

* **`get_state`** ([ros_cellulo/CelluloState])

	Returns the state of the robot: - pose (x,y,theta), veclocities (Vx,Vy,w) , Kidnapped state, KeysTouched, KeysLongTouched.


#### Parameters

* **`scale`** (string, default: 1)

	The scale used: scale =1 means the position unit is mm, scale=0.001 means unit is in meters.
	
### type: ros_cellulo_sensor

#### Published Topics
* **`/cellulo_node_xx_xx/sensor`** ([ros_cellulo_swarm/ros_cellulo_sensor]) includes: 1) detected_robots:Number of detected robots, 2) relative_positions: array of relative positions of the detected robots, 3) distances: array of the distances to the detected robots.  

#### Subscribed Topics

* **`/sensor_node_sensor_xx_xx/setThreshold`** ([std_msgs/Floats64])	Sets the detection threshold.

#### Parameters

* **`cellulo_num`** (int, default: 1) indicating which robot to consider from the list. 

* **`threshold`** (string, default: 1) for the detection threshold

## To use QT Creator as IDE :

- 1- open qt Creator   
- 2- File -> Open file or project   
- 3- Choose catkin_ws/src/ros_cellulo/CmakeLists.txt  
- 4- Go to Projects tab, change the build directory to catkin_ws/build/ros_cellulo   
- 5- Make sure that the cmake prefix path is defined as:   
CMAKE_PREFIX_PATH: /opt/Qt/5.11.1/gcc_64;/opt/ros/kinetic/;/home/user/catkin_ws/devel/  
- 6- Now you can run the cmake/build/ and debug in Qt Creator   
