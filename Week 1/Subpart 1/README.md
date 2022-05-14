<!-- http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem<br>
http://wiki.ros.org/ROS/Tutorials/CreatingPackage<br>
http://wiki.ros.org/ROS/Tutorials/BuildingPackages<br> -->

<details>
  <summary><h1>ROS Workspace and Packages</h1></summary>
 ​	A workspace is a set of directories (or folders) where you store related pieces of ROS code. The official name for workspaces in ROS is catkin workspaces.

​	Consider a simple drone project like the one you have done in AeroNav Event. In that event, we have provided you with a basic PID controller and you have written a program to follow a path and count the number of boxes along the way. So, basically, there were 3 parts 
  1. Drone Controller
  2. Following the path
  3. Count boxes using the camera 

​	 All the 3 parts were programmed in a controller.py file which made the code a colossal mess. In ROS we can use separate code files of a single part (Ex: drone controller) kept in separate folders called **packages**. For example, a famous open-source package called PX4 is used to control drones in ROS. So handling code files in ROS is simple and easier.

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
   catkin_make
  ```
  We need to activate the workspace
  ```bash
   gedit .bashrc 
  ```
  A window will be opened add the below command at the last line
  
  ```bash
   source devel ~/catkin_ws/devel/setup.bash
  ```

</details>

<details>
  <summary><h1>Creating ROS Package</h1></summary>
                              
  ![6g511m](https://user-images.githubusercontent.com/76437900/168428605-4186fa9a-46af-45df-8397-d6ae606d9e96.jpg)

  ```bash
   cd ~/catkin_ws/src
   catkin_create_pkg path_follower rospy roscpp 
  ```
  
  The above command is used to create a package in ROS
  The command contains 3 parts 
  1. catkin_create_pkg: Which indicates your creating package
  2. path_follower: Package name
  3. rospy roscpp : These are dependencies on the package 
 
  
  For example, if you need to create a package for path following you need a controller. You can use the open-source PX4 controller. Hence in the code which u write u need to use the PX4 functions to control the drone So, PX4 should be a dependency for your package. 
  
  roscpp is the package that you need to keep as a dependency when you code in C++
  
  rospy is the package that you need to keep as a dependency when you code in python

  
   To list dependencies we use below command
  ```bash
    rospack depends1 path_follower 
  ```
   Here depends1 represents the 1st order dependencies of package like rospy roscpp for the above package. Dependency packages of 1st order dependency packages are called indirect dependencies  
  
  <h3>Buildng the Package</h3>
  
  As you know in ROS we can use multiple code files to integrate all code files, to use the functionalities of all code files we need to build the package.
  For building the package we use CMake and Catkin Build.
  In home directory type the below commands to build all packages in the workspace at once

  
  ```bash
   cd catkin_ws
   catkin_make
  ```
  After building the package ,the structure of package will be
  
  ```bash
  .
├── CMakeLists.txt
├── package.xml
└── src
    

  ```
  You can see the new files CMakeLists.txt and package.xml which are generated while building the package 
  
  In CMakeLists.txt
  
  ```cmake
    find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    )
  ```
  The code snippet has the dependency packages which were specified during the creation of the package. There are some lines that show dependency packages in package.xml too
  
  ```xml
    <build_depend>roscpp</build_depend>
    <build_depend>rospy</build_depend>
    <build_export_depend>roscpp</build_export_depend>
    <build_export_depend>rospy</build_export_depend>
    <exec_depend>roscpp</exec_depend>
    <exec_depend>rospy</exec_depend>
  ```
  You should change the above two code snippets to add or delete dependency accordingly.
  
  [Further Reading 1](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
  
  [Further Reading 2](http://wiki.ros.org/ROS/Tutorials/BuildingPackages)
</details>

<details>
  <summary><h1>Navigating ROS File System</h1></summary>
   Generally, we use many packages and code files for a single project hence it's difficult to find the specific package we want. ROS provides commands to find packages in the ROS environment.
  
  ```bash
   rospack find path_follower
  ```
  this outputs the path of path_follower package
  
  ![6g51hj](https://user-images.githubusercontent.com/76437900/168428745-662e6b09-1245-4a0d-ab80-def4470feaef.jpg)

  ```bash
   roscd path_follower
  ```
  this changes the directory to path_follower package
  
  ```bash
   rosls path_follower
  ```
  This lists the files and folders present in path_follower package
  
  [Further Reading](http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem)
  
</details>
