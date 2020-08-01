Jacob Patrick Reed
reed.jacobp@gmail.com
github.com/reedjacobp

This document is a set of notes that I am taking as I build this robot to help those after me.

- Jetson Nano
- CanaKit Wifi USB Adapter
- 16GB SD Card with Ubuntu 18.04
- ROS Melodic
- RobotZone 313RPM Planetary Gear Motors (2 with encoders, 2 without)
- RoboClaw 2x30A V5E
- Slamtec Mapper M1M1/A1M8
- Intel RealSense D435i Depth Camera
- Rubber Wheels x 4 (5 in diameter)
- 14.8V 4000 mAh 35C 4S LiPo Battery

Login Information:
	Username - jetson
	Password - jetson

In order to prevent issues in the future, add yourself to the dialout group:
	$ sudo usermod -a -G dialout jetson

***IMPORTANT***
WHEN USING A BATTERY TO POWER THE JETSON, ENSURE ALL SENSORS/DEVICES ARE NOT PLUGGED INTO THE
USB PORTS WHEN TURNING IT ON. FOR EXAMPLE, IF THE LIDAR/CAMERA IS PLUGGED INTO THE JETSON, THE 
JETSON WILL NOT BE ABLE TO BOOT UP DUE TO LARGE CURRENT DRAW.

**********************************************************************************************
NoMachine:

I first installed NoMachine (Remote Desktop Viewer) in order to help speed up the process
as well as control the robot remotely. I have set the desktop to login automatically but I 
can't figure out how to boot the Jetson Nano without first being connected to a display and
still have a decent resolution.
**********************************************************************************************

**********************************************************************************************
Follow instructions on the ROS Wiki to install ROS Melodic. In my case, this was already
installed. I will first discuss the packages to install and how to use them.

*****************************
*First Package: roboclaw_ros*
*****************************
- The first package to install is roboclaw_ros by sonyccd so that we can get the /odom topic 
  published.
- Find the package here: https://github.com/sonyccd/roboclaw_ros
- Once the roboclaw_ros package is installed, make sure to look at the pull requests tab and
  apply the changes from #15-#18. You will run into issues if you don't. 
- Ensure the motors work correctly by using the BasicMicro Motion Studio for Windows. The
  right side of the robot is M1 and the left side is M2.
- Follow this guide: https://resources.basicmicro.com/auto-tuning-with-motion-studio/
  to autotune the PID values only for velocity and don't forget to write to the device when
  you are all done.
- Once you are back in Ubuntu, open the roboclaw.launch file located in the roboclaw_ros 
  package and edit the values for ticks_per_meter, and base_width.
- For ticks_per_meter, the motors with encoders have 1288.848 countable events to make one
  revolution on the output shaft. Since the wheels have a 5 inch diameter, we can find the 
  circumference and calculate how many ticks the encoders will count for the robot to travel
  1 meter.
- For base_width, just measure the width of the robot from the outside edges of the wheels.
- You should now be able to launch the roboclaw.launch file:
	$ roslaunch roboclaw_node roboclaw.launch
- If you run into an error that says something like "Problem getting roboclaw version" then 
  you probably don't have the correct permissions for /dev/ttyACM0.
- Run:
	$ ls -l /dev/ttyACM* 
  If you see /dev/ttyACM0, make sure it has the correct permissions by running:
	$ sudo chmod a+rw /dev/ttyACM0
  and now you should be all set.

***************************************
*Second Package: teleop_twist_keyboard*
***************************************
- The second package to install is teleop_twist_keyboard so that we can control the movement
  of the robot using the keyboard.
- Run:
	$ sudo apt-get install ros-melodic-teleop-twist-keyboard
	$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
  and now you should be able to control the robot.
- I suggest starting off at slow speeds to see how fast it moves.

************************************************
*Third Package: slamware_ros_sdk & slamware_sdk*
************************************************
- The third package actually consists of two different folders that depend on each other.
  This package supports the use of the Slamtec Mapper M1M1 Lidar.
- Download the package from: http://www.slamtec.com/en/Support#mapper and place the contents 
  of the top "src" folder into the "src" folder of your workspace.
- Since the Jetson Nano is using ROS Melodic and Ubuntu 18.04, the GCC version will be 7 and
  so the package won't build properly.
- To check gcc version, run:
	$ gcc --version
- If you have alternatives, you can change which version you are using:
	$ sudo update-alternatives --config gcc

********************
*NOTE: I have not been able to find a solution for installing this package. I am using the
RPLidar A1M8 for this instead for the time being. 
- In the terminal, move to the catkin_ws/src directory and run the following:
	$ git clone https://github.com/Slamtec/rplidar_ros.git
- Build the workspace and then source devel/setup.bash
- Run the following to test if the RPLidar works:
	$ roslaunch rplidar_ros view_rplidar.launch
  and you should see RViz pop up with red lines from the laser.
- If you get the following error:
	[ERROR] [1585859144.732496083]: Error, cannot bind to the specified serial port 
	/dev/ttyUSB0.

  then that means you need to change the permissions for that.
- To fix that, run the following:
	$ sudo chmod a+rw /dev/ttyUSB0
********************

*******************************
*Fourth Package: realsense_ros*
*******************************
- This fourth package is to utilize the Intel RealSense D435i Depth Camera.
- First, we need to install the Intel RealSense SDK Library, named librealsense. Fortunately,
  there is a github repository that contains instructions to use a pre-made shell script to
  install this specifically for the Jetson Nano.
- Run the following:
	$ git clone https://github.com/JetsonHacksNano/installLibrealsense.git
- MAKE SURE THE CAMERA IS NOT PLUGGED INTO THE JETSON
- Following the instructions from the README, we will be installing librealsense by doing
  the following:
	$ cd installLibrealsense
	$ ./installLibrealsense.sh
- Once installed, test that this worked by running:
	$ realsense-viewer
  and ensure it starts up with no problems and update the firmware if it prompts you to
  do so.

**********************************************************************************************
With all of the device packages installed, we now need to install the packages to create a
URDF (Unified Robot Description Format) of the robot. This is one method to begin mapping, or
you could just make a simple static transform of each device with their location and rotation
with respect to the robot frame. Please refer to the ROS Wiki to find out how to create the 
URDF or do a static transform. Reviewing the wiki and looking at the existing packages and 
launch files for this should be sufficient enough to understand the material.
**********************************************************************************************

******************************************
*Fifth Package: joint_state_publisher-gui*
******************************************
- This package is to create joints in the URDF file in order to begin mapping. 
- If it isn't already installed, run the following:
	$ sudo apt-get install ros-melodic-joint-state-publisher-gui

******************************
*Sixth Package: slam_gmapping*
******************************
- This package is going to create a 2D map for us.
- If it isn't already installed, run the following:
	$ sudo apt-get install ros-melodic-slam-gmapping

*****************************
*Seventh Package: navigation*
*****************************
- The navigation stack will be helping us with the autonomous navigation and it also contains
  the packages required to save and load maps.
- Run the following:
	$ sudo apt-get install ros-melodic-navigation

*******************************************
*Seventh Package: robot_pose_ekf(OPTIONAL)*
*******************************************
- This package takes in odometry, IMU, and/or visual odometry (camera) and applies an
  extended kalman filter (EKF) to the data to make the odometry a little more accurate.
- Run the following:
	$ sudo apt-get install ros-melodic-robot-pose-ekf
-NOTE: This will not work for navigation, but is excellent for mapping. The navigation stack 
      requires a different topic for odometry than this package can output.

**********************************************************************************************
The next section will be going over how to create a 2D map using slam_gmapping. This process
is very simple and only requires the RoboClaw, LIDAR, and that the robot is completely mobile.
**********************************************************************************************
1. Plug in RPLidar and run:
	$ roslaunch rplidar_ros rplidar.launch
2. Plug in RoboClaw and run:
	$ roslaunch roboclaw_node roboclaw.launch
3. Run(OPTIONAL):
	$ roslaunch robot_pose_ekf robot_pose_ekf.launch
4. Run:
	$ roslaunch jetson_description urdf_visualize.launch model:='$(find jetson_description)/urdf/jetson.urdf'
(At this point, what you see in RViz will not look like anything useful. This is because
 the busbot_mapper package provides that last transform you need which is map --> odom_combined)
5. Run:
	$ roslaunch jetson_mapper jetson_mapper_launch.launch
(Make sure to open the busbot_gmapping.rviz config file)
6. Move the robot around the environment to create a map:
	$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
7. To save the map:
	$ rosrun map_server map_saver -f name_of_map

*********************************************************************************
The next section will go over how to localize the robot on the newly created map
if you were to turn it off and back on in a completely different part of the map.
*********************************************************************************
1. Run:
	$ roslaunch jetson_description urdf_visualize.launch model:='$(find jetson_description)/urdf/jetson.urdf'
2. Plug in RPLidar and run:
	$ roslaunch rplidar_ros rplidar.launch
3. Plug in RoboClaw and run:
	$ roslaunch roboclaw_node roboclaw.launch
4. Run:
	$ roslaunch jetson_amcl_launcher jetson_amcl_launch.launch
5. Run:
	$ roslaunch initialize_particles init_particles_caller.launch
6. Run:
	$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py

NOTE: I was unable to get to navigation using the Jetson Nano. There were problems with both power
     and memory. The Jetson Nano was unable to provide enough power for itself, the LIDAR, and also
     the motors. Please take a look at github.com/reedjacobp/BusBot to see how I built the packages 
     for both navigation and RTABmap. Looking at those alongside the tutorials on the ROS Wiki should 
     point you in the right direction. However, the packages are also built here and are ready to go 
     when power and memory are not an issue.

****************************************************************************
With the robot properly localized on a map, the robot is ready to navigate. 
Look at the ROS Wiki if you would like to change some of the parameters for
navigation.
****************************************************************************
1. Run:
	$ roslaunch jetson_description urdf_visualize.launch model:='$(find jetson_description)/urdf/jetson.urdf'
2. Plug in RPLidar and run:
	$ roslaunch rplidar_ros rplidar.launch
3. Plug in RoboClaw and run:
	$ roslaunch roboclaw_node roboclaw.launch
4. Run:
	$ roslaunch jetson_move_base_launcher jetson_move_base_launch_1.launch

*********************************************************************************
If desired, you can create a 3D map along with a 2D map using RTABmap. RTABmap
is great for this kind of mapping and is excellent for localization. It takes
a series of images from a mapping session and is stored in a database. When 
localizing, it will compare what it is currently seeing to the database to place
itself on the map.
*********************************************************************************
1. Run:
	$ roslaunch jetson_description urdf_visualize.launch model:='$(find jetson_description)/urdf/jetson.urdf'
2. Plug in RPLidar and run:
	$ roslaunch rplidar_ros rplidar.launch
3. Plug in RoboClaw and run:
	$ roslaunch roboclaw_node roboclaw.launch
4. Plug in D435i and run:
	$ roslaunch realsense2_camera rs_camera.launch
5a. If you would like to map, run:
	$ roslaunch jetson_rtab_package jetson_rtab_package_launch.launch localization:=false
5b. If you would like to localize, run:
	$ roslaunch jetson_rtab_package jetson_rtab_package_launch.launch localization:=true
