# Week 1
Let us first install ROS in your system and then we will get familiar with some basic tools in ROS.

## ROS Installation:-

You can see the tutorials on official webiste of ROS [installation](http://wiki.ros.org/noetic/Installation/Ubuntu) or you can the tutorials given below:-

1. Setup your sources.list
Using this command your computer will allow downloads from the website of ROS

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

2. Set up your keys

```
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

3. Installation

```
sudo apt update
sudo apt install ros-noetic-desktop-full

```



Now that you have successfully installed ROS, head over to [creating a package](./creating a package.md
