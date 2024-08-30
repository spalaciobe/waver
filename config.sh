#!/bin/bash

ROS_IMAGE="osrf/ros:noetic-desktop-full"
VNC_IMAGE="dorowu/ubuntu-desktop-lxde-vnc:focal"
MAC_IMAGE="dorowu/ubuntu-desktop-lxde-vnc:focal-arm64"
WS="/waver_ws"

DOCKER_IMAGE_NAME="waver-noetic-image"
CONTAINER_NAME="waver-noetic-container"
ROS_NETWORK="host"
