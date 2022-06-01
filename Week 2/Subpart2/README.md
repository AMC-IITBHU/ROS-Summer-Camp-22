<details>
  <summary><h1>Unified Robotics Description Format(URDF)</h1></summary>
  
  When it comes to any project related to robotics or arieal robotics the 1st thing you need to do is to decide the robot on which you want to work. There are many standard robots and arieal vechiles in ROS which are well establised and used by many companies for industrial usage. Many aireal vechiles function in similar way but differ much in their appearance and structure. The structural difference deeply impact on the thrust force provided by the air, tension in material and etc., which makes structural aspects and design an important part of Arieal Robotics. The basic way by which you give your design of robot to Gazebo is through URDF.
  
  URDF assumes a tree structure for a robot. There are mainly 3 things base link, joints and links.
  1. Base Link: Generally robots are very large but many times we tell the location of robot with some x y coordinates. Exactly of what part of robot are these coordinates? The answer is BASE LINK. This the base of robot. We insert other parts of robot after fixing the base link.
  Example consider a 2 wheeled robot as shown below
  
  There is a orange colored box which is the base link of the robot.
  A robot only contains a single base link we cannot have multiple base links.
  
  2. Links: Links are the other parts of robot except the base link. Example: the 2 wheels in the above robot are links.
  
  3. Joint: 2 links are joined together with joints. Its like a pivot which gives necessary force so that they remain attached.
  
  Types of Joints:
  
  revolute - a hinge joint that rotates along the axis and has a limited range specified by the upper and lower limits.
  
  continuous - a continuous hinge joint that rotates around the axis and has no upper and lower limits.
  
  prismatic - a sliding joint that slides along the axis, and has a limited range specified by the upper and lower limits.
  
  fixed - This is not really a joint because it cannot move. All degrees of freedom are locked. 
  
  Joints connects 2 links one of which is parent link other is child link
  
  <h3>Structure and Parameters of URDF</h3>
  
  ** This part is very important, Read it carefully**
  
  1st thing which you need to do is to decide your base link.
  
  ```xml
<?xml version="1.0" ?>
<robot name="AMC">
  <link name="base_link">
    <!-- pose -->
    <pose>0 0 0.1 0 0 0</pose>
    <!-- body -->
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <box size="0.5 0.3 0.07"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <!-- caster front -->
  </link>
 </robot>
 ```
 
</details>
