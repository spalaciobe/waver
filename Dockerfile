ARG IMAGE="osrf/ros:noetic-desktop-full"
ARG OS="linux"

FROM ${IMAGE}

ARG WS
ENV DEBIAN_FRONTEND=noninteractive
ENV RESOLUTION=1920x1080
ENV ROS_DISTRO=noetic
ENV USER=root
ENV WS=${WS}
WORKDIR ${WS}

RUN apt update && apt install -y \
    python3-catkin-tools \
    python3-rosinstall \
    wget \
    curl \
    git \
    nano \
    graphviz \
    iputils-ping \
    net-tools

RUN wget https://raw.githubusercontent.com/roboticamed/docker_ros_vnc/main/install_ros_noetic.sh && \
    chmod +x ${WS}/install_ros_noetic.sh && \
    if [ ${OS} != "linux" ]; then ${WS}/install_ros_noetic.sh ${ROS_DISTRO} ${WS}; fi

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-catkin \
    ros-${ROS_DISTRO}-urdf \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-tf2-tools

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-navigation \
    ros-${ROS_DISTRO}-slam-gmapping \
    ros-${ROS_DISTRO}-map-server \
    ros-${ROS_DISTRO}-amcl \
    ros-${ROS_DISTRO}-teb-local-planner

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source ${WS}/devel/setup.bash" >> ~/.bashrc
RUN echo "alias sros='source /opt/ros/${ROS_DISTRO}/setup.bash ; catkin build ; source ${WS}/devel/setup.bash'" >> ~/.bashrc
