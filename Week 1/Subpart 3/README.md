http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch (isme launch file banana and roslaunch....ye dono zarur se cover kr lena) <br>
http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29<br>
http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29<br>
http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber<br>
http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29<br>
http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29<br>
http://wiki.ros.org/ROS/Tutorials/ExaminingServiceClient<br>

make sure to cover these

<details>
  <summary><h1>1. Definition of Publishers and Subscribers</h1></summary>
  
  I think you already know about publishers and subscribers from subpart 2. Let me brief your memory about it.
  
  Message passing in ROS happens with the Publisher Subscriber Interface provided by ROS library functions. The primary mechanism for ROS nodes to exchange data is sending and receiving messages. Messages are transmitted on a topic, and each topic has a unique name in the ROS network. 
  
  If a node wants to share information, it uses a publisher to send data to a topic. Or we can say a Publisher is the one puts the messages of some standard Message Type to a particular Topic. 
  
  A node that wants to receive that information uses a subscriber to that same topic. Or we can say the Subscriber subscribes to the Topic so that it receives the messages whenever any message is published to the Topic. 
  
  A ROS Node can be a Publisher or a Subscriber. Besides its unique name, each topic also has a message type, which determines the types of messages that are capable of being transmitted under that topic.

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


<details>
  <summary><h1>2. Coding for Publishers and Subscribers</h1></summary>
  
  Publishers and Subscribers can be used by either rostopic pub and rostopic echo respectively or by writting a node in either python or c++. Here I will be explaining you how to write code in python as it is quite easy to understand but if want to learn c++ just tell us in discord and we will guide you for the same
  
  <br>
<p align="center">
  <img src="https://github.com/AMC-IITBHU/ROS-Summer-Camp-22/blob/main/Week%201/assets/pub_and_sub.jpeg">
</p>
<br>  
  
# Publishers
  
  

  Move to the ros package that you created before. Inside the package create a folder named scripts. Inside the scripts folder create a python file with any name you like. Here I am using the name "learn_publishers.py"

  In the python file put the following code 

  ```python
  #!/usr/bin/env python3
  #import the rospy package and the String message type
  import rospy
  from std_msgs.msg import String
  #function to publish messages at the rate of 2 messages per second
  def messagePublisher():
      message_publisher = rospy.Publisher(‘/messageTopic’, String, queue_size=10)
      rospy.init_node(‘messagePubNode’, anonymous=True)
      rate = rospy.Rate(2)
      while not rospy.is_shutdown():
          message = “ROS Tutorial by Aero Modelling Club, IIT BHU Varanasi”
          rospy.loginfo(‘Published: ‘ + message)
          message_publisher.publish(message)
          rate.sleep()
  if __name__ == ‘__main__’:
      try:
          messagePublisher()
      #capture the Interrupt signals
      except rospy.ROSInterruptException:
          pass
  ```
  
  Now open the terminal in the scripts folder and type
  
  ```bash
  chmod a+x learn_publishers.py #name of the python file created
  ```
  
  Now open up a terminal and start roscore
  
  Now open up another terminal and type the following code to run the node for the publsihing the message
  
  ```bash
  cd ~/catkin_ws
  source devel/setup.bash
  rosrun beginner_tutorials learn_publishers.py
  ```
  
  Now a topic is being published with name "/messageTopic". Use the rostopic list command to conform whether the topic is being published. Use the rostopic echo command to see what is being published to the topic. 
 
  Now let us decode the above code line by line
  
  ```python
  #!/usr/bin/env python3
  #import the rospy package and the String message type
  import rospy
  from std_msgs.msg import String
  ```
  
  The first line is just a comment, then why are we explaining this. Well the thing is in the line this comment mentions the path of your python interpreter. In the subsequent lines we have imported the required python packages. First is rospy which is python client library for ROS. Second is std_msgs.msg. The std_msgs.msg import is so that we can reuse the std_msgs/String message type (a simple string container) for publishing. 
  
  Next is the function  messagePublisher() 
  Let us decode it.
  
  ```python
  message_publisher = rospy.Publisher(‘messageTopic’, String, queue_size=10)
  rospy.init_node(‘messagePubNode’, anonymous=True)
  ```
  
  This section of code defines the talker's interface to the rest of ROS. 
  message_publisher = rospy.Publisher("/messageTopic", String, queue_size=10) declares that your node is publishing to the chatter topic using the message type String. String here is actually the class std_msgs.msg.String. The queue_size argument is to limits the amount of queued messages if any subscriber is not receiving them fast enough.
  
  The next line, rospy.init_node(NAME, ...), is very important as it tells rospy the name of your node -- until rospy has this information, it cannot start communicating with the ROS Master. In this case, your node will take on the name talker.
  
  ```python
  rate = rospy.Rate(10) # 10hz
  ```
  
  This line creates a Rate object rate. With the help of its method sleep(), it offers a convenient way for looping at the desired rate. With its argument of 10, we should expect to go through the loop 10 times per second (as long as our processing time does not exceed 1/10th of a second!)
  
  ```python
  while not rospy.is_shutdown():
        message = “ROS Tutorial by Aero Modelling Club, IIT BHU Varanasi”
        rospy.loginfo(‘Published: ‘ + message)
        message_publisher.publish(message)
        rate.sleep()
  ```
  
  This loop is a fairly standard rospy construct: checking the rospy.is_shutdown() flag and then doing work. You have to check is_shutdown() to check if your program should exit (e.g. if there is a Ctrl-C or otherwise). In this case, the "work" is a call to message_publisher.publish(message) that publishes a string to our "messageTopic" topic. The loop calls rate.sleep(), which sleeps just long enough to maintain the desired rate through the loop.
  
  The loop also calls rospy.loginfo , which performs triple-duty: the messages get printed to screen, it gets written to the Node's log file, and it gets written to rosout.
  
</details>  


