<!-- http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem<br>
http://wiki.ros.org/ROS/Tutorials/CreatingPackage<br>
http://wiki.ros.org/ROS/Tutorials/BuildingPackages<br> -->

<details>
  <summary><h1>ROS Workspace and Packages</h1></summary>
 ​	A workspace is a set of directories (or folders) where you store related pieces of ROS code. The official name for workspaces in ROS is catkin workspaces.

​	Consider a simple drone project like the ones you have done in AeroNav Event. In that event we have provided you a basic PID controller and you have written program to follow a path and count the number of boxes along the way. So, basically there were 3 parts 
  1. Drone Controller
  2. Following the path
  3. Count boxes using camera 

​	All the 3 parts were programmed in a controller.py file which made the code a huge mess. In ROS we can use separate code files of a single part (Ex: drone controller) kept in separate folders called **packages**. Example: there is a famous open source package called PX4 which is used to control drone in ROS. So handling code files in ROS is simple and easier.

  <h3>File Structure of Workspace</h3>
  
   ```bash
    .
└── ros_ws
    └── src
        ├── Package1
        ├── Package2
        └── Package3
  ```
  
  <h3>Creating a Workspace</h3>
  
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/
  ```

</details>

<details>
  <summary><h1>Creating ROS Package</h1></summary>
  
  ```bash
   catkin_create_pkg path_follower rospy roscpp PX4
  ```
  
  The above command is used to create package in ROS
  The command contains 3 parts 
  1. catkin_create_pkg which indicates your creating package
  2. path_follower package name (The folder name in which you keep your code files)
  3. rospy roscpp PX4 : These are dependecies for the package 
  
  For example if you need to create a package for path following you need a controller. You can use the open source PX4 controller. Hence in the code which u write u need to use the PX4 functions to control drone So, PX4 is depedency for your package. 
  
  roscpp is the package which you need to keep as dependency when your coding in C++
  
  rospy is the package which you need to keep as dependency when your coding in python
  
</details>
