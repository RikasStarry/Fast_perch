TOPIC_NAMES="/gazebo_odom /iris/imu /manager_node/traj"
#"/iris/ground_truth/imu /gazebo_path /manager_node/traj"
#rosbag play target_2024-05-31-10-39-10.bag
rosbag record -o record.bag ${TOPIC_NAMES[@]}


