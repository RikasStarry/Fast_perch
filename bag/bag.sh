TOPIC_NAMES="/gazebo_odom /iris/imu /iris/ground_truth/odometry /iris/ground_truth/imu /manager_node/traj"
#"/iris/ground_truth/imu /gazebo_path /manager_node/traj" /iris/ground_truth/odometry
rosbag record -o record.bag ${TOPIC_NAMES[@]}


