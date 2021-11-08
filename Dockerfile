#Download base image ubuntu 20.04
FROM ubuntu:20.04
FROM osrf/ros:noetic-desktop-full

RUN 	apt-get update		&& \
	apt-get upgrade -y	&& \
	apt update		&& \
	apt upgrade -y

RUN 	apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y

RUN 	rosdep update
RUN 	rosdep update --include-eol-distros

RUN 	apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control -y

RUN 	echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN	/bin/bash -c 'source ~/.bashrc'
RUN 	mkdir -p /home/catkin_ws/src
WORKDIR /home/catkin_ws/src
RUN 	/bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_init_workspace /home/catkin_ws/src'

WORKDIR /home/catkin_ws
RUN 	/bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/catkin_ws; catkin_make'
RUN 	echo "source /home/catkin_ws/devel/setup.bash" >> ~/.bashrc

RUN	apt-get install python3-pip -y
RUN	pip3 install --ignore-installed tensorflow gym wandb

WORKDIR /home/catkin_ws/src
RUN 	apt-get install git -y
RUN	git clone https://github.com/NicoV1994/gazebo_openai_tool.git
WORKDIR /home/catkin_ws
RUN 	/bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/catkin_ws; catkin_make'
RUN 	/bin/bash -c 'source /home/catkin_ws/devel/setup.bash'

RUN	apt-get install ros-noetic-joy -y
RUN	apt-get install ros-noetic-teleop-twist-joy -y
RUN	apt-get install ros-noetic-teleop-twist-keyboard -y
RUN	apt-get install ros-noetic-laser-proc -y
RUN	apt-get install ros-noetic-rgbd-launch -y
RUN	apt-get install ros-noetic-rosserial-arduino -y
RUN	apt-get install ros-noetic-rosserial-python -y
RUN	apt-get install ros-noetic-rosserial-client -y
RUN	apt-get install ros-noetic-rosserial-msgs -y
RUN	apt-get install ros-noetic-amcl -y
RUN	apt-get install ros-noetic-map-server -y
RUN	apt-get install ros-noetic-move-base -y
RUN	apt-get install ros-noetic-urdf -y
RUN	apt-get install ros-noetic-xacro -y
RUN	apt-get install ros-noetic-compressed-image-transport -y
RUN	apt-get install ros-noetic-rqt* -y
RUN	apt-get install ros-noetic-rviz -y
RUN	apt-get install ros-noetic-gmapping -y
RUN	apt-get install ros-noetic-navigation -y
RUN	apt-get install ros-noetic-interactive-markers -y
RUN	apt-get install ros-noetic-cv-bridge -y
RUN	apt-get install ros-noetic-vision-opencv -y
RUN	apt-get install python3-opencv -y
RUN	apt-get install libopencv-dev -y
RUN	apt-get install ros-noetic-image-proc -y

RUN	rosdep install --from-paths /home/catkin_ws --ignore-src --rosdistro=noetic -y

RUN	chmod +x /home/catkin_ws/src/gazebo_openai_tool/learning/scripts/.

RUN	apt-get install psmisc -y
RUN	echo "alias killgazebogym='killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient'" >> ~/.bashrc
