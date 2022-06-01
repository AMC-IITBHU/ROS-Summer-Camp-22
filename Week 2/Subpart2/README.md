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

    <!-- body -->
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <box size="0.5 0.3 0.07"/>
      </geometry>
    </visual>
    <collision name="collision_chassis">
      <geometry>
        <box size="0.5 0.3 0.07"/>
      </geometry>
    </collision>
    <!-- caster front -->
  </link>
 </robot>
 ```
 Everything should be written under robot tag
 In the above code you can see that there is a link tag and there are different parameters and tags inside it. Base Link and normal Links have same parameters and tags.
 
 1. Visual: It contains the visual aspects of link
 2. origin: it contains 6 float values which specify x,y,z and roll pitch yaw. This defines the frame of link. But the most important question is with respective which frame are these values given?
 For base link it is given with respective to the world frame in simulator as in above example robot is present 0.1m above the ground.
 For other links it is given with respective to the joint in which it is child. Except base links all other links will be as child link to atleast one joint.
 Note: The coordinates specified in pose are the coordinates of center of link with respective to world frame for base link and joint frame for other links
 
 3.geometry: It contains the shape of the link
 4.Box: Here our base link is cuboid
 5. Collision: Visual tag just gives the view of robot. To make robot solid colision tag is used. Generally visual and collision tag have same values.
 
 ```xml

  <joint name="joint_right_wheel" type="continuous">
    <origin rpy="0 0 0" xyz="-0.05 0.15 0"/>
    <child link="link_right_wheel"/>
    <parent link="base_link"/>
    <axis rpy="0 0 0" xyz="0 1 0"/>
  </joint>
 ```
  1. Origin: Here origin is given with respective parent frame.
  Note: Whenever you see frame of link it is same as that of frame of joint in which it is child. Hence frame of link_right_wheel is same as frame of joint_right_wheel.
  Frame of Base link is center of the link.
  2. Axis: As it is continuous joint child link rotates around the axis
  Whole URDF of 2 wheeled robot

  Further Reading 
 
 
 
 
</details>
