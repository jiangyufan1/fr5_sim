# Source only from the simulation scripts.
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo 'ROS 2 Humble is missing. Run scripts/install_dependencies.sh first.' >&2
  return 1
fi
# Keep ROS on system Python independently of the ACT Conda environment.
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONHOME PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
unset LD_LIBRARY_PATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=85
