source /opt/ros/noetic/setup.bash
#source /home/rosdev/social_gym/submodules/pedsim_ros/devel/setup.bash

DIR=$(pwd)
echo $DIR
export ROS_PACKAGE_PATH=/home/rosdev/src/amrl_msgs:$ROS_PACKAGE_PATH

export ROS_PACKAGE_PATH=/home/rosdev/src/:$ROS_PACKAGE_PATH

export ROS_MASTER_URI=http://rosmaster:11311