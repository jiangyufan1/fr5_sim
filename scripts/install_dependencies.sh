#!/usr/bin/env bash
set -eo pipefail
source /etc/os-release
if [ "$ID" != ubuntu ] || [ "$VERSION_ID" != 22.04 ]; then
  echo 'This installer targets Ubuntu 22.04 / ROS Humble.' >&2; exit 1
fi
sudo -v
sudo apt-get update
sudo apt-get --no-remove install -y curl ca-certificates software-properties-common
sudo add-apt-repository -y universe
if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | grep -q 'install ok installed'; then
  download_dir=$(mktemp -d)
  trap 'rm -rf "$download_dir"' EXIT
  source_version=$(curl -fsSL --retry 2 https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | /usr/bin/python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')
  curl -fL --retry 2 -o "$download_dir/ros2-apt-source.deb" "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${source_version}/ros2-apt-source_${source_version}.jammy_all.deb"
  sudo dpkg -i "$download_dir/ros2-apt-source.deb"
fi
sudo apt-get update
sudo apt-get --no-remove install -y build-essential cmake liblapack-dev python3-colcon-common-extensions python3-yaml ros-humble-desktop ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro ros-humble-tf2-kdl ros-humble-tf2-eigen
printf '\nDependencies installed. Run scripts/build.sh next.\n'
