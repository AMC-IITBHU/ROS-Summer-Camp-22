
<details>
  <summary><h1>1. How to make a launch file</h1></summary>
  
  One way to execute a program in ROS it to launch one node at a time. But many a times you need to launch multiple nodes. Launching each node one-by-one can get inefficient really quickly.

  Fortunately, ROS has a tool called roslaunch that enables you to launch multiple nodes all at once. Let’s do that now.
  
  <br>
<p align="center">
  <img src="https://github.com/AMC-IITBHU/ROS-Summer-Camp-22/blob/main/Week%201/assets/roslaunch_meme.jpeg">
</p>
<br>  
  
  # Directions
  
  The first thing we need to do is to open a new terminal window and go to the hello_world package (or whatever package you want to launch). Create a folder named launch and inside it create a new launch file. Let the name of file be start.launch
  
  ```bash
  cd catkin_ws/src/hello_world
  mkdir launch
  cd launch
  ```
  
  ## Example 1 - launch turtlesim node and teleop for turtlesim node together.
  
  ```xml
  <?xml version="1.0"?>
  <launch>
    <node name="turtlesim" pkg="turtlesim" type="turtlesim_node" output="screen"/>
    <node name="turtlesim_teleop" pkg="turtlesim" type="turtle_teleop_key" output="screen"/>
  </launch>
  ```
  
  Now let us decode this code. 
  
  The first line is the version of xml that ur editor is using
  
  The file start and end with the launch tag, so that file is identified as a launch tag.
  
  ```xml
  <node name="turtlesim" pkg="turtlesim" type="turtlesim_node" output="screen"/>
  <node name="turtlesim_teleop" pkg="turtlesim" type="turtle_teleop_key" output="screen"/>
  ```
  
  Here we start two nodes. One is to start turtlesim node i.e. to spawn the turtle in a window. Other node is to start the teleop node for turtlesim.
  
  Can you guess the name of these two nodes?
  
  If you wanna know more about roslaunch see this [tutorial for roslaunch](http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch)
  
</details>


<details>
  <summary><h1>2. Coding for Publishers and Subscribers</h1></summary>
  
  Publishers and Subscribers can be used by either rostopic pub and rostopic echo respectively or by writting a node in either python or c++. 
  
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
  
  # Subscribers
  
  Move to the ros package that you created before. Inside the package create a folder named scripts. Inside the scripts folder create a python file with any name you like. Here I am using the name "learn_subscribers.py"

  In the python file put the following code 
  
  ```python
  #!/usr/bin/env python3
  import rospy
  from std_msgs.msg import String
  #Callback function to print the subscribed data on the terminal
  
  def callback_str(subscribedData):
       rospy.loginfo('Subscribed: ' + subscribedData.data)
  
  def messageSubscriber():
      rospy.init_node('messageSubNode', anonymous=False)
      rospy.Subscriber('/messageTopic', String, callback_str)
      rospy.spin()
  
  if __name__ == '__main__':
      try:
          messageSubscriber()
      except rospy.ROSInterruptException:
          pass
  ```
  
  Now open the terminal in the scripts folder and type
  
  ```bash
  chmod a+x learn_subscribers.py #name of the python file created
  ```
  
  Now open up a terminal and start roscore
  
  Now open up another terminal and type the following code to run the node for the publsihing the message
  
  ```bash
  cd ~/catkin_ws
  source devel/setup.bash
  rosrun beginner_tutorials learn_subscribers.py
  ```
  
  Now the above node has subscribed to the topic "/messageTopic". If you are getting a error, the reason would be you are not running the pubslisher node taught above 
  
  In the termianl you will see a message is being printed.
  "Subscribed: ROS Tutorial by Aero Modelling Club, IIT BHU Varanasi"
  
  lets decode the above code line by line. 
  
  ```python
  #!/usr/bin/env python3
  import rospy
  from std_msgs.msg import String
  ```
  
  This is already taught in publishers. First line is the path of python interpreter and the subsequent two lines are imports of required python packages.
  
  Now the function messageSubscriber()
  
  ```python
  rospy.init_node('messageSubNode', anonymous=False)
  rospy.Subscriber('/messageTopic', String, callback_str)
  rospy.spin()
  ```
  
  This declares that your node subscribes to the chatter topic which is of type std_msgs.msgs.String. When new messages are received, a callback function is invoked with the message as the first argument.
  
  The final addition, rospy.spin() simply keeps your node from exiting until the node has been shutdown. Unlike roscpp, rospy.spin() does not affect the subscriber callback functions, as those have their own threads.
  
  
</details>  


<details>
  <summary><h1>3. Coding for Service and Client</h1></summary>
  
  Here we will make a ros service and client for addition of two integers.
  
  # Writing a service node
  
  
  
  Move to the ros package that you created before. Inside the package create a folder named scripts. Inside the scripts folder create a python file with any name you like. Here I am using the name "learn_server.py"

  In the python file put the following code 
  
  ```python
  #!/usr/bin/env python3

  from beginner_tutorials.srv import AddTwoInts,AddTwoIntsResponse
  import rospy

  def handle_add_two_ints(req):
      print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
      return AddTwoIntsResponse(req.a + req.b)

  if __name__ == "__main__":
      rospy.init_node('add_two_ints_server')
      s = rospy.Service('/add_two_ints', AddTwoInts, handle_add_two_ints)
      print("Ready to add two ints.")
      rospy.spin()
  ```
  
  Now open the terminal in the scripts folder and type
  
  ```bash
  chmod a+x learn_service.py #name of the python file created
  ```
  
  Now open up a terminal and start roscore
  
  Now open up another terminal and type the following code to run the node for starting the rosservice
  
  ```bash
  cd ~/catkin_ws
  source devel/setup.bash
  rosrun beginner_tutorials learn_service.py
  ```
  
  Now if you do rosservice list you must see "/add_two_ints"
  
  To call this rosservice 
  
  ```bash
  rosservice call /add_two_ints beginner_tutorials/AddTwoInts "a: 2 b: 2"
  ```
  
  The first three would be already clear to you. The beginner_tutorials.srv import is for using 
  
  
  Now rospy.init_node has already been explained. 
  
  ```python
  s = rospy.Service('/add_two_ints', AddTwoInts, handle_add_two_ints)
  ```
  
  This declares a new service named add_two_ints with the AddTwoInts service type. All requests are passed to handle_add_two_ints function. handle_add_two_ints is called with instances of AddTwoIntsRequest and returns instances of AddTwoIntsResponse.
  
  
  # Writing a client node
  
  
  Move to the ros package that you created before. Inside the package create a folder named scripts. Inside the scripts folder create a python file with any name you like. Here I am using the name "learn_client.py"

  In the python file put the following code 
  
  ```python
  #!/usr/bin/env python
  import sys
  import rospy
  from beginner_tutorials.srv import *

  def add_two_ints_client(x, y):
      rospy.wait_for_service('add_two_ints')
      try:
          add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
          resp1 = add_two_ints(x, y)
          return resp1.sum
      except rospy.ServiceException as e:
          print("Service call failed: %s"%e)

  if __name__ == "__main__":
      print("Give two numbers for input")
      x = int(input())
      y = int(input())
      print("Requesting %s+%s"%(x, y))
      print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))
  ```
  
  Now open the terminal in the scripts folder and type
  
  ```bash
  chmod a+x learn_service.py #name of the python file created
  ```
  
  Now open up a terminal and start roscore
  
  Now open up another terminal and type the following code to run the node for starting the client node
  
  ```bash
  cd ~/catkin_ws
  source devel/setup.bash
  rosrun beginner_tutorials learn_service.py
  ```
  
  The program will take 2 numbers as input and then return their addition. 
  
  If Program is showing "waiting For service ....." then you have not started the server node mentioned above
  
  Now the next part is understanding the code
  
  ```python
  rospy.wait_for_service('add_two_ints')
  ```
  
  rospy.wait_for_service is a convenience method that blocks until the service named add_two_ints is available.
  
  Next we create a handle for calling the service:
  
  ```python
  add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
  ```
  
  We can use this handle just like a normal function and call it:
  
  ```python
        resp1 = add_two_ints(x, y)
        return resp1.sum
  ```
  
  Because we've declared the type of the service to be AddTwoInts, it does the work of generating the AddTwoIntsRequest object for you (you're free to pass in your own instead). The return value is an AddTwoIntsResponse object.
  
  
</details>

