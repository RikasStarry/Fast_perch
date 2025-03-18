#!/usr/bin/env python

import rospy
import rosbag
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import os

bag_file = "/home/gnij/Fast-Perching-master/bag/collision/pro.bag"  # rosbag 文件路径
topic_name = "/manager_node/traj"  # 要发布的主题名称
new_topic_name = "/manager_node/traj_2"

def publish_path_from_bag(bag_file, topic_name):
    # 检查 bag 文件是否存在
    if not os.path.exists(bag_file):
        rospy.logerr(f"Bag file does not exist: {bag_file}")
        return

    # 初始化 ROS 节点
    rospy.init_node('path_publisher', anonymous=True)
    
    # 创建一个 Publisher 来发布 Path 消息
    path_pub = rospy.Publisher(new_topic_name, Path, queue_size=10)
    
    try:
        # 打开 rosbag 文件
        with rosbag.Bag(bag_file, 'r') as bag:
            topics_info = bag.get_type_and_topic_info()
            rospy.loginfo(f"Topics in bag: {topics_info.topics.keys()}")
            # 检查 bag 文件中是否有目标主题
            if topic_name not in topics_info.topics:
                rospy.logwarn(f"Topic '{topic_name}' not found. Available topics: {list(topics_info.topics.keys())}")
                return

            path_msg = Path()
            path_msg.header.frame_id = "map"
            rospy.loginfo(f"Reading messages from topic: {topic_name}...")

            for topic, msg, t in bag.read_messages(topics=[topic_name]):  # 仅读取目标主题
                rospy.loginfo(f"Processing message type: {type(msg)}")
                if msg._type == "nav_msgs/Path":
                    path_msg.header.stamp = rospy.Time.now()
                    path_msg.poses = msg.poses
                    path_pub.publish(path_msg)
                    rospy.loginfo("Published Path message")
                else:
                    rospy.logwarn(f"Skipped non-Path message: {type(msg)}")

                rospy.sleep(0.1)  # 确保消息有时间发布

            rospy.loginfo("Finished processing bag.")
            return

    except Exception as e:
        rospy.logerr(f"Error: {e}")
        raise

if __name__ == '__main__':
    try:
        publish_path_from_bag(bag_file, topic_name)
    except rospy.ROSInterruptException:
        print("Error occurred during execution.")