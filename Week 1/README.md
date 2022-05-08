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
```

There are a two versions of ROS - melodic and noetic

If you have Ubuntu 20.04, you need to install ROS noetic
```
sudo apt install ros-noetic-desktop-full
```

if you have Ubuntu 18.04, you need to install ROS melodic
```
sudo apt install ros-melodic-desktop-full
```

4. Now you also need to setup your enviornment
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

5. Install some dependencies 
```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

With the following, you can initialize rosdep.

```
sudo rosdep init
rosdep update
```


Now that you have successfully installed ROS, head over to [creating a package](./creating a package.md
