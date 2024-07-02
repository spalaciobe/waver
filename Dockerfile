FROM osrf/ros:noetic-desktop-full

RUN mkdir -p /waver_ws/src

RUN apt update && apt install -y \
    python3-catkin-tools \
    python3-rosinstall \
    git \
    nano \
    graphviz \
    iputils-ping \
    net-tools

RUN apt-get update && apt-get install -y \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-catkin \
    ros-noetic-urdf \
    ros-noetic-xacro \
    ros-noetic-tf2-tools

RUN apt-get update && apt-get install -y \
    ros-noetic-navigation \
    ros-noetic-slam-gmapping \
    ros-noetic-map-server \
    ros-noetic-amcl 

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "/waver_ws/devel/setup.bash" >> ~/.bashrc

COPY ./autostart.sh /
RUN chmod +x /autostart.sh
# ENTRYPOINT /autostart.sh
