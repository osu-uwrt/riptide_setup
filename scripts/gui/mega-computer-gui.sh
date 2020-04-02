ssh ros@mega-computer 'bash -ic "roscore"' &

export ROS_IPV6=on
export ROS_MASTER_URI=http://mega-computer:11311
rqt
