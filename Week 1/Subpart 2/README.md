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
  
A node is nothing but an executable file inside a ROS package. It makes use of topics and services to communicate with the other nodes which are inside the same or different package.

Here is a real life analogy.
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
Let’s now fill in those packages with useful nodes which will be responsible for the execution of the program.

### Nodes for the camera package
The camera package will handle a camera as an independent unit. We need a driver for the camera, to be able to program it, and get frames from it. Then we also need a program that will take those frames and do some image processing work.
<br>
<p align="center">
  <img src="https://user-images.githubusercontent.com/77807055/168075161-248b8e63-5f16-4eb2-aa0e-682968d3741f.jpg">
</p>
<br>
  
All those programs in blue are nodes. Each node is launched separately. First you will launch the driver, and then the image processing node. **The nodes will then communicate using ROS communication functionalities**, for example topics, services and actions.
<br><br>

### Nodes for the motion planning package
In this package you can expect to have a motion planning node, which will compute motion planning for any given robot. We can also add a path correction node, which role is to modify the motion planning due to external factors.
<br>
<p align="center">
  <img src="https://user-images.githubusercontent.com/77807055/168075873-68bd5600-1f63-4784-b218-c993fb4de865.jpg">
  </p>
<br>
Next step is to make the two nodes in different packages communicate each other.

Let’s link the image processing node to the path correction node. The image processing node will analyze frames coming from the camera and will send an analysis of the environment to the path correction node. This ROS node will then be able to notify the motion planning node.

### Nodes for the hardware control package
Hardware control package, as an independent unit, will control the hardware of the robot. That can be wheels, a robotic arm joints, or anything else.

We’ll find some drivers to control the motors. The drivers are controlled from the main control loop node. And let’s say that the position data coming from the motor encoders is sent back to the control loop for more precise control. This data is also published by a state publisher node.
<br>
<p align="center">
  <img src="https://user-images.githubusercontent.com/77807055/168076263-d2b984f9-b26b-4001-a7b0-b58432432d4a.jpg">
  </p>
<br>
The motion planning node from the motion planning package will send computed trajectories to the main control loop node, inside the hardware control package.

The hardware state of the robot is published, and both the motion planning and path correction nodes are receiving it. Thus, the motion planning can be dynamically changed thanks to the hardware or camera data.

### But, what's the point of writing multiple nodes when you can jot down all of the code in a single file? Well, there are some reasons...
* Splitting the code into multiple nodes helps faster debugging and helps in organising them better.
* ROS nodes are not internally linked. They communicate only through topics and services. So, if one node crashes, others are safe.
* There are some client libraries available in ROS such as roscpp and rospy that allow nodes written in different languages (C++/ Python) to communicate. So, your are no more bounded to write all your code in a single programming language.

## Hell of theoretical stuff, isn't it?
<p align="center">
  <img width=500 src="https://media0.giphy.com/media/LTYT5GTIiAMBa/giphy.webp?cid=ecf05e47ema8hvix956qus7ss40s392hbyyetc7myruj6pjb&rid=giphy.webp&ct=g">
  </p>
  
#### Well then, back to technical stuff. Hit that button👇 to learn how to run a node and get info about one:
* [Understanding ROS Nodes](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)<br>
#### In the subpart 3, you will learn how to write a node.
</details>

<details>
  <summary><h1>2. ROS Topics</h1></summary>
  
**A topic is a named bus over which nodes exchange messages**.
  
## Our first publisher
  Let's understand ROS topics through a real world analogy of radio transmitter and receiver. Suppose we have one radio transmitter. It will send some data on a given frequency, say 98.7 frequency, so you know that if you want to receive music from the radio station, you need to connect your device to “98.7”.

  You can see the green box here, 98.7, as a ROS topic, and **the radio transmitter is a publisher** of this topic. So for this case, a data stream is sent over the 98.7 topic.
<br>
  <p align="center">
    <img src="https://user-images.githubusercontent.com/77807055/168092995-7addc220-8c2d-4f50-969b-2b0e403169a0.jpg">
  </p>
  <br>
  
## Time to add some subscribers
  Suppose, your phone receives messages from the 98.7 topic. **Your phone is then a subscriber of the topic**. But for that, your phone must be able to decode the type of message that the radio transmitter is sending, apart from being on the right frequency. If it is sending AM signal, your phone should decode it. That’s why **both the publisher and subscriber must send messages with the same data structure**.

  So we have our radio transmitter and the phone, both using AM signals. They respectively publish and subscribe to the 98.7 topic.
  <br>
  <p align="center">
    <img src="https://user-images.githubusercontent.com/77807055/168093045-2d9daad5-1844-4a1c-a8e5-bbe18faf9b05.jpg">
  </p>
  <br>
  
## Multiple subscribers for one topic
What if you also want to listen to the radio station from your car? You just need to connect your car to the 98.7 radio. Your car should also be able to decode AM signal.
  <br>
  <p align="center">
    <img src="https://user-images.githubusercontent.com/77807055/168093110-3734e565-fcac-438a-9ffe-0cddc1be553c.jpg">
  </p>
  <br>
  
With ROS, **you can have multiple subscribers for the same topic**. A subscriber is not aware of the other subscribers and publisher. It only knows it is receiving data from the 98.7 topic. Thus, we can say that **subscribers are anonymous.**
  
## Multiple publishers for one topic
**You can also have many publishers for the same topic**. Imagine another radio transmitter which is also publishing an AM signal to 98.7. It can be the same radio station, it can also be another radio station. All the subscribers will receive the messages from both publishers.
<br>
  <p align="center">
    <img src="https://user-images.githubusercontent.com/77807055/168093198-852db4ad-6ee4-4fb8-a01d-44fb5abff555.jpg">
  </p>
  <br>
  
A publisher is also not aware of the other publishers and the subscriber of the topic. It only publishes data to the topic, and that’s it. **Publishers on a ROS topic are anonymous.**
  
So, each node which is publishing or subscribing to the topic is totally independent. For example, you could have 3 subscribers on the topic and no publisher. It’s still working, but the subscribers will just receive no data. If you have 2 publishers on the topic, and no subscriber, the data is just sent and no one receives it.

## Multiple publishers/subscribers inside one node
A node can publish and subscribe on many different topics.
  <br>
  <p align="center">
    <img src="https://user-images.githubusercontent.com/77807055/168093295-82bc6726-94c9-414e-8009-afb399b4863d.jpg">
  </p>
  <br>
  
Let’s say that the radio transmitter node number 2 is publishing AM signal on the 98.7 topic, and FM signal on the 101.3 topic. The car can subscribe to the 101.3 topic, and decode FM signal at the same time.

**A node can contain multiple publishers, but also subscribers**. The car, while listening to the radio, can publish its coordinates to a car_location topic.
  <br>
  <p align="center">
    <img src="https://user-images.githubusercontent.com/77807055/168093352-abc57d22-7716-4afd-a707-492ebd4867a6.jpg">
  </p>
  <br>
  
The car node has now one subscriber on the 98.7 topic, and one publisher on the car_location topic. The computer node is subscribing to the car location topic, and for the communication to be successful, both nodes are sending and receiving the same kind of message.

Well, that’s it for the analogy! You should now have a better comprehension of what is a ROS topic and when it is useful.
  
## Get back to reality

Note that for the real world analogy I used numbers with dots as topic name. This is not valid, a topic name must start with a letter, followed by letters, numbers, underscores, tildes, and slashes. For example, you could a topic named “/radio_98_7”.

Technically speaking, the messages are sent over TCP/IP. The ROS libraries that you will use on your code, will provide you with enough abstraction so you don’t have to deal with the TCP/IP layer.
  
### Aren't ROS topics tired of being the middleman😢?
  <br>
  <p align="center">
    <img width=500 src="https://media0.giphy.com/media/l396WS0aAT9hQ3HmU/200w.webp?cid=ecf05e477qxfmxy89rah3o621zmkeuwyg2prpjzbu56e44yr&rid=200w.webp&ct=g">
    <br><i>Topics would love the internet, isn't it? 🙁</i>
  </p>
  <br>

## Points to Note!
- A topic has a message type. All publishers and subscribers on this topic **must use the message type associated with the topic**.
- As you already know, you can write a node in multiple languages, using for example the roscpp library for C++, and rospy library for Python. Well, those libraries also include the Topic functionality. So, you can **create a publisher or subscriber in any ROS supported language you want**, directly inside ROS nodes.
- When a node wants to publish something, it will inform the ROS master. When another node wants to subscribe to a topic, it will ask the ROS master from where it can get the data. You can see the **ROS master as a DNS server for nodes** to find where to communicate.

Again, I want you to head over to the following tutorial to visualise topics using command line:
  - [Understanding Topics](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)
  
</details>

<details>
  <summary><h1>3. Services</h1></summary>
  
A ROS service is a **client/server system**. Let's go with a real world analogy again of a weather service.

## Our first ROS service
The weather service gives us the local weather after we send our location. You, on your computer, are considered as the client, and the weather service online is the server. You will be able to access the server through an HTTP request, with a URL. Think as the HTTP URL as a ROS service.

First of all, your computer will send a request to the server. The request will contain a message, in this case your location. The server will then process the request, and send a response. The response will also contain a message.

The request sent by the client must be a location. And the server must send back a weather.
  <br>
  <p align ="center">
    <img src="https://user-images.githubusercontent.com/77807055/168376907-fcdf0cf8-0594-4016-8b51-eaa8e026ad22.jpg">
  </p>
  <br>

## Multiple clients for one service
Multiple clients can also send a request containing a location to the server, through the HTTP URL. The server will then process the requests and send back a response to each client. **Note that you should not have more than one server for the same service**.
  <br>
  <p align="center">
    <img src="https://user-images.githubusercontent.com/77807055/168376977-2bfb6629-eee5-4251-9907-59f2af7ae77f.jpg">
  </p>
  <br>

Here, we have 3 different computer nodes, and one node for the weather service. The HTTP URL can be seen as a ROS service. The computer nodes contain a service client. This service client will call the ROS service and send a request with a location. On the other side, the weather service contains a ROS service server which will process all requests and send back a response through the ROS service.

Again, **all clients and the server inside nodes are not aware of each other**. They only see up to the ROS service interface.

## Points to Note
Here are some of the main characteristics of a ROS service:

- It is **synchronous**. The client sends a requests, and blocks until it receives a response.
- You should use ROS services **only for computations and quick actions**. For example the client will send some data, and receive another piece of data. Or for example, if you want to enable or disable an actuator, or any immediate action. As the service call is blocking, you don’t want your client to be stuck for too long.
- **A service is defined by a name, and a pair of messages**. One message is the request, one message is the response. You must respect the format of the data on both side of the communication.
- As for nodes and topics, you can directly create service clients and servers inside ROS nodes, using for example the rosccp library for c++ and the rospy library for Python.
  
Topics will be used for unidirectional data streams, and services will be used when you need a client/server architecture.
  <br>
  <p align="center">
    <img width=500 src="https://media2.giphy.com/media/yxt1GCEZ4u9tl5z4br/200w.webp?cid=ecf05e473ix2ifup29mnlitelejzbu7qud8amlf09zwnu6c4&rid=200w.webp&ct=g">
  </p>
  <br>
</details>

<details>
  <summary><h1>4. Parameters</h1></summary>
  
## Why do you need ROS parameters ?
Suppose you want to create some global settings in your application, for example:
- The name of your robot.
- The frequency at which you read some sensors.
- A simulation flag that you can use in all your nodes to inform that the robot is running in real mode or simulation mode.
  
You certainly don’t want to hardcode those settings in all your nodes or get too many useless dependencies between your nodes.

So, you need a sort of global dictionary for shared settings in your application, that can be retrieved at runtime, when you launch your nodes.

## The ROS parameter server
After you launch the ROS master, the parameter server is automatically created inside the ROS master.
  <br>
  <p align="center">
    <img src="https://user-images.githubusercontent.com/77807055/168381409-0d2970f8-008c-43e7-8641-67655c7ef7bb.jpg">
  </p>
  <br>

The parameter server is basically **a dictionary containing global variables (ROS parameters) which are accessible from anywhere in the current ROS environment**.
<br>
  <p align="center">
    <img src="https://user-images.githubusercontent.com/77807055/168381486-30e25e8d-5a6e-470e-9ae5-09f43f6d016d.jpg">
  </p>
  <br>
  
At any time, **a node can read a parameter, modify a parameter, and can create new ones**. Like in the figure, any of the 4 nodes in 3 packages can get access to the ROS parameter server given that the nodes should be on the same environment as the ROS master.

A ROS parameter has a name, and a data type. Among the most common types, you can use:
- Boolean
- Integer number
- Double number
- String
- List of previous data types

See for yourself how services and parameters actually work:
  - [Understanding Services and Parameters](http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams)

### Too much for now, isn't it?
  <p align="center">
    <img src="https://media0.giphy.com/media/hqOOKhJXtk3T7JqX8t/200w.webp?cid=ecf05e47sc05y03z38zyvzrya9f8ulw3qmp98itmrh7u0ic6&rid=200w.webp&ct=g">
  </p>
  <br>
</details>

<details>
  <summary><h1>5. Msg and Srv</h1></summary>
  <p align="center">
    <img src="https://media4.giphy.com/media/xT1R9LUBYOXB4b8E6Y/200.webp?cid=ecf05e475z1guf8z6i739bizam8pk7t2iy3wskys0ccrydej&rid=200.webp&ct=g">
  </p>
  <br>
  
## ROS messages in topics and services
- A topic is defined by 2 things:
  - A name, which is the interface to reach.
  - A message definition, which is the data structure of the information you send.
  
- A service is also defined by 2 things:
  - A name, which is the interface to reach.
  - A service definition that contains one message definition for the request, and one message definition for the response.

Well, **you can see topics and services as the communication layer tools, and messages as the actual content you send.**

## Yay, another real life example
When you send a mail, the transport company will transport your letter. The content of this letter is the analogy of a ROS message.
  <br>
  <p align="center">
    <img src="https://user-images.githubusercontent.com/77807055/168383691-8ad32c75-6b3c-40af-aeb0-d424399b290b.jpg">
  </p>
  <br>
  
When you send a letter and you wait for a response, then the first letter contains a Request message, and the letter that you receive back contains a Response message. The combination of the two message definitions is the service definition.
  <br>
  <p align="center">
    <img src="https://user-images.githubusercontent.com/77807055/168383720-b0467180-9e44-45b1-9ed3-baf31418c224.jpg">
  </p>
  <br>

## How are ROS messages created ?
So, first you create a message definition. When you use the catkin_make command line, the message will be parsed by the build system. And then, a source code will be generated for this message, in any ROS supported language: C++, Python etc.

That’s why you can, for example, directly include the message header in your C++ code. Because the build system generated this header file.
  <br>
  <p align="center">
    <img src="https://user-images.githubusercontent.com/77807055/168383753-a0d04394-2b99-4c70-85a3-192c00d4e7da.jpg">
  </p>
  <br>
 
Head over to the following tutorial to see in depth how messages are actually created👇
- [Creating Msg and Srv](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)
  
#### As I promised, this is the last component. If you have come down this far, then trust me...
  <br>
  <p align="center">
    <img width=300 src="https://media1.giphy.com/media/loA2it5PFS5fBt6Ud3/200w.webp?cid=ecf05e47uuu4e31br0s8ig6carft73xggwova2me5kigmclb&rid=200w.webp&ct=g">
  </p>
  <br>
  
#### Now, straighten your back a li'll bit and take time to grasp these concepts, get familiar with their command line usages, we will meet you next in Subpart 3. Till then...
  <br>
  <p align="center">
    <img width=500 src="https://media2.giphy.com/media/aePS31pKg94KSh8Cl3/200w.webp?cid=ecf05e47sce1s23e2wywcn2i8f7azmbqhpf6qngswvutiioi&rid=200w.webp&ct=g">
  </p>
  <br>
</details>
