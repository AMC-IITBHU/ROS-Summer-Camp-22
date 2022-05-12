http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch (isme launch file banana and roslaunch....ye dono zarur se cover kr lena) <br>
http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29<br>
http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29<br>
http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber<br>
http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29<br>
http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29<br>
http://wiki.ros.org/ROS/Tutorials/ExaminingServiceClient<br>

make sure to cover these

<details>
  <summary><h1>1. Defination of Publishers and Subscribers</h1></summary>
  
Message passing in ROS happens with the Publisher Subscriber Interface provided by ROS library functions. The primary mechanism for ROS nodes to exchange data is sending and receiving messages. Messages are transmitted on a topic, and each topic has a unique name in the ROS network. If a node wants to share information, it uses a publisher to send data to a topic. Or we can say a Publisher is the one puts the messages of some standard Message Type to a particular Topic. A node that wants to receive that information uses a subscriber to that same topic. Or we can say the Subscriber subscribes to the Topic so that it receives the messages whenever any message is published to the Topic. A ROS Node can be a Publisher or a Subscriber. Besides its unique name, each topic also has a message type, which determines the types of messages that are capable of being transmitted under that topic.

This publisher and subscriber communication has the following characteristics:

   - Topics are used for many-to-many communication. Many publishers can send messages to the same topic and many subscribers can receive them.

   - Publishers and subscribers are decoupled through topics and can be created and destroyed in any order. A message can be published to a topic even if there are no active subscribers.
  
   - Note that a publisher can publish to one or more Topic and a Subscriber can subscribe to one or more Topic.

   - Also, publishers and subscribers are not aware of each others’ existence. The idea is to decouple the production of information from its consumption and all the IP addresses of various nodes are tracked by the ROS Master.

The concept of topics, publishers, and subscribers is illustrated in the figure:
  
<br>
<p align="center">
  <img src="https://github.com/AMC-IITBHU/ROS-Summer-Camp-22/blob/main/Week%201/assets/pub_and_sub.png">
</p>
<br>  

  
</details>

