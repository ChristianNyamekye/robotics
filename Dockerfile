FROM osrf/ros:humble-desktop

RUN apt-get -y update && apt-get install -y \
    curl

RUN apt-get -y update && apt-get install -y \
    iputils-ping \
    net-tools \
    wget \
    screen \
    git \
    nano \
    vim \
    htop \
    ros-${ROS_DISTRO}-ros-gz
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash"
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo "defshell -bash" >> ~/.screenrc
RUN echo "source /root/catkin_ws/install/setup.bash" >> /root/.bashrc
WORKDIR /root/catkin_ws/src
