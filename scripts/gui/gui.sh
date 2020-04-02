ssh ros@$1 'bash -ic "roscore"' &

export ROS_MASTER_URI=http://$1:11311
rqt
