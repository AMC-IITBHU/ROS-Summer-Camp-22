<!--http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes<br>
http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics<br>
http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams<br>
http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv<br>
-->
# Glad you have made it this far :tada:

<p align="center">
  <img width=500 src="https://media4.giphy.com/media/hWkg5NRbpwW9yIDV3r/100.webp?cid=ecf05e47su1z5s8b2z4ef0zr0e0mlnlkn103ea5es3jdai7k&rid=100.webp&ct=g">
  </p>
  
#### Now that you have understood how to create packages, its time for you to learn some of their components...Click on the following topics one by one. Click again to collapse them back
<details>
  <summary><h1>1. ROS Nodes</h1></summary>
  
A node is nothing but an executable file inside a ROS package. It makes use of topics and services to communicate with the other nodes that can be inside the same or different package. You can just see a node as a sub-part of your robotics application. Your application will contain many nodes, which will be put into packages. Nodes will then communicate with each other.

Let’s get into more details with an example taken from a real life application!
## A mobile robot controlled by a camera
#### Let’s start with a standard robotics application which involves a mobile robot and a camera. The robot has 3 ROS packages (from low to high level):
- [**Camera package: processes images and give useful info and commands to the robot**](https://github.com/AMC-IITBHU/ROS-Summer-Camp-22/edit/main/Week%201/Subpart%202/README.md#nodes-for-the-camera-package)
- [**Motion planning package: monitors and controls the robot trajectory**](https://github.com/AMC-IITBHU/ROS-Summer-Camp-22/edit/main/Week%201/Subpart%202/README.md#nodes-for-the-motion-planning-package)
- [**Hardware control package: directly controls the hardware (wheels and other actuators)**](https://github.com/AMC-IITBHU/ROS-Summer-Camp-22/edit/main/Week%201/Subpart%202/README.md#nodes-for-the-hardware-control-package)
<br>
<p align="center">
  <img src="https://user-images.githubusercontent.com/77807055/168066267-42738370-7e45-4af4-8a57-3d8c89db3613.jpg">
  </p>
<br>
Those packages are the 3 main parts of your application. As you can see they’re all empty. Let’s now fill in those packages with useful nodes which will be responsible for the execution of the program.

### Nodes for the camera package
The camera package will handle a camera as an independent unit.

So, what should we put inside?

First, we need a driver for the camera, to be able to program it, and get frames from it. Then we also need a program that will take those frames and do some image processing work. We could also add any other program related to the camera we are using.
<br>
<p align="center">
  <img src="https://user-images.githubusercontent.com/77807055/168075161-248b8e63-5f16-4eb2-aa0e-682968d3741f.jpg">
  </p>
<br>
All those programs in blue are nodes. Each node is launched separately. First you will launch the driver, and then the image processing node. The nodes will then communicate using ROS communication functionalities, for example topics, services and actions.
<br>
All right, we have our camera package filled with all the nodes we need.

### Nodes for the motion planning package
In this package you can expect to have a motion planning node, which will compute motion planning for any given robot. We can also add a path correction node, which role is to modify the motion planning due to external factors.
<br>
<p align="center">
  <img src="https://user-images.githubusercontent.com/77807055/168075873-68bd5600-1f63-4784-b218-c993fb4de865.jpg">
  </p>
<br>
Great! We have 2 packages filled with nodes.

What we can do now, is to make 2 nodes inside different packages communicate together.

Let’s link the image processing node to the path correction node. The image processing node will analyze frames coming from the camera and will send an analysis of the environment to the path correction node. This ROS node will then be able to notify the motion planning node.

### Nodes for the hardware control package
And we finish with our third package, which is the hardware control. This package, as an independent unit, will control the hardware of the robot. That can be wheels, a robotic arm joints, or anything else.

In this package we’ll find some drivers to control the motors. The drivers are controlled from the main control loop node. And let’s say that the position data coming from the motor encoders is sent back to the control loop for more precise control. This data is also published by a state publisher node.
<br>
<p align="center">
  <img src="https://user-images.githubusercontent.com/77807055/168076263-d2b984f9-b26b-4001-a7b0-b58432432d4a.jpg">
  </p>
<br>
The motion planning node from the motion planning package will send computed trajectories to the main control loop node, inside the hardware control package.

The hardware state of the robot is published, and both the motion planning and path correction nodes are receiving it. Thus, the motion planning can be dynamically changed thanks to the hardware or camera data.

Note that this architecture is really similar to what you can actually see in real life robots.

### But, what's the point of writing multiple nodes when you can jot down all of the code in a single file? Well, there are some reasons...
* Splitting the code into multiple nodes helps faster debugging and helps in organising them better.
* ROS nodes are not internally linked. They communicate only through topics and services. So, if one node crashes, others are safe.
* There are some client libraries available in ROS such as roscpp and rospy that allow nodes written in different languages (C++/ Python) to communicate. So, your are no more bounded to write all your code in a single programming language.

#### Now that you have a conceptual understanding of nodes, you can visit the following link to learn how to get info about a node and how to run one:
* [Understanding ROS Nodes](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)<br>
#### In the subpart 3, you will learn how to write a node.
</details>

<details>
  <summary><h1>2. ROS Topics</h1></summary>
  
</details>

<details>
  <summary><h1>3. Services</h1></summary>
  
</details>

<details>
  <summary><h1>4. Parameters</h1></summary>
  
</details>

<details>
  <summary><h1>5. Msg</h1></summary>
  
</details>

<details>
  <summary><h1>6. Srv</h1></summary>
  
</details>
