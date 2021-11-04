# gazebo_openai_tool
bachelor thesis

## Setup

### Installation

Ubuntu 20.04
ROS Noetic

Update Ubuntu and install curl
```bash
sudo apt-get update

sudo apt-get upgrade -y

sudo apt update

sudo apt upgrade -y
```

Install ROS Noetic (source: http://wiki.ros.org/noetic/Installation/Ubuntu)
```bash
sudo apt-get install curl

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt-get update

sudo apt-get upgrade -y

sudo apt update

sudo apt upgrade -y

sudo apt-get install ros-noetic-desktop-full -y

source /opt/ros/noetic/setup.bash

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

source ~/.bashrc

sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y

sudo apt install python3-rosdep -y

sudo rosdep init

rosdep update --include-eol-distros

sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control -y
```

Create catkin workspace (source: http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
```bash
mkdir -p ~/catkin_ws/src

cd ~/catkin_ws/src

catkin_init_workspace

cd ~/catkin_ws

catkin_make

source ~/catkin_ws/devel/setup.bash

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

cd ~/catkin_ws/src

sudo apt-get install python3-pip -y

pip3 install --ignore-installed tensorflow gym wandb
```
Download GIT Repository
```bash

git clone https://github.com/NicoV1994/gazebo_openai_tool.git

cd ~/catkin_ws

catkin_make
```
