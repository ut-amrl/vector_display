source /opt/ros/noetic/setup.bash
#source /home/rosdev/social_gym/submodules/pedsim_ros/devel/setup.bash

DIR=$(pwd)
echo $DIR
export ROS_PACKAGE_PATH=/home/rosdev/vector_display/src/amrl_msgs:$ROS_PACKAGE_PATH

export ROS_PACKAGE_PATH=/home/rosdev/vector_display:$ROS_PACKAGE_PATH

export ROS_MASTER_URI=http://vector_display_rosmaster:11311
export DISPLAY=unix:0