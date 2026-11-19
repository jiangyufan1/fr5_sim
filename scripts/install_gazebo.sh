#!/usr/bin/env bash
set -eo pipefail
source /etc/os-release
[ "$ID" = ubuntu ] && [ "$VERSION_ID" = 22.04 ] || { echo 'Ubuntu 22.04 required'; exit 1; }
sudo -v
sudo apt-get -o Acquire::Retries=2 -o Acquire::https::Timeout=30 -o APT::Update::Error-Mode=any update
sudo apt-get --no-remove install -y ignition-fortress ros-humble-ros-gz ros-humble-gz-ros2-control ros-humble-rqt-image-view
echo 'Gazebo Fortress dependencies installed.'
