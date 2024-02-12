#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

export HOST_UID=$(id -u)

docker-compose -f $SCRIPT_DIR/docker-compose.yml run \
--volume $SCRIPT_DIR/ros_encoder:/colcon_ws/src/ros_encoder \
ros-encoder bash
