#!/bin/bash

set -e

PROJECT_ROOT="$(cd "$(dirname "$0")"; cd ..; pwd)"
source ${PROJECT_ROOT}/config.sh

cd "$PROJECT_ROOT"

OS_TYPE=$(uname -s | tr '[:upper:]' '[:lower:]')
ARCH_TYPE=$(uname -m)

if [ "$OS_TYPE" == "darwin" ]; then
    if [ "$ARCH_TYPE" == "x86_64" ]; then
        IMAGE=${VNC_IMAGE}
    elif [ "$ARCH_TYPE" == "arm64" ]; then
        IMAGE=${MAC_IMAGE}
    else
        echo "Unsupported Arch type: $ARCH_TYPE"
        exit 1
    fi
elif [ "$OS_TYPE" == "linux" ]; then
    if [ "$ARCH_TYPE" == "x86_64" ]; then
        IMAGE=${ROS_IMAGE}
    elif [ "$ARCH_TYPE" == "aarch64" ]; then
        IMAGE=${ROS_IMAGE}
    elif [ "$ARCH_TYPE" == "arm64" ]; then
        IMAGE=${MAC_IMAGE}
    else
        echo "Unsupported Arch type: $ARCH_TYPE"
        exit 1
    fi
elif [ "$OS_TYPE" == "windows" ]; then
    if [ "$ARCH_TYPE" == "x86_64" ]; then
        IMAGE=${VNC_IMAGE}
    elif [ "$ARCH_TYPE" == "arm64" ]; then
        IMAGE=${MAC_IMAGE}
    else
        echo "Unsupported Arch type: $ARCH_TYPE"
        exit 1
    fi
else
    echo "Unsupported OS type: $OS_TYPE"
    exit 1
fi

docker build --build-arg IMAGE=${IMAGE} --build-arg OS=${OS_TYPE} -t ${DOCKER_IMAGE_NAME} .
