#!/usr/bin/env python
import os
import numpy as np
from math import cos, sin
import rospy
import tf
import threading
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path,Odometry
from gazebo_msgs.msg import ModelStates

class TrueStatePublisher:
    def __init__(self):
        rospy.init_node('gazebo_state', log_level=rospy.DEBUG)
        self.odom_pub = rospy.Publisher('/gazebo_odom', Odometry, queue_size=1)
        self.path_pub = rospy.Publisher('/gazebo_path', Path, queue_size=1)
        # 共享数据保护锁
        self.lock = threading.Lock()
        self.latest_data = None

        self.latest_path = Path()
        self.latest_path.header.frame_id = "world"

        # 定时器配置 (100Hz)
        self.timer = rospy.Timer(rospy.Duration(0.005), self.timer_cb)
        self.timer2 = rospy.Timer(rospy.Duration(0.5), self.timer_cb2)
        
        # 订阅Gazebo数据（无频率限制）
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_cb, queue_size=1)

    def model_cb(self, data):
        """原始数据回调，仅存储最新数据"""
        try:
            vehicle_index = data.name.index("iris")
            with self.lock:
                self.latest_data = {
                    'pose': data.pose[vehicle_index],
                    'twist': data.twist[vehicle_index],
                    'recv_time': rospy.Time.now()
                }
        except ValueError:
            rospy.logwarn_throttle(1.0, "等待车辆模型出现...")

    def timer_cb(self, event):
        """定时发布回调 (100Hz严格周期)"""
        with self.lock:
            if self.latest_data is None:
                return
            
            # 复制数据避免处理过程中被修改
            current_data = self.latest_data
            # 保留数据用于下次发布（按需选择）
            # self.latest_data = None  

        try:
            # 计算时间延迟
            processing_delay = (rospy.Time.now() - current_data['recv_time']).to_sec()
            if processing_delay > 0.1:
                rospy.logwarn_throttle(1.0, f"高延迟警告: {processing_delay:.3f}s")

            # 处理数据
            vehicle_pose = current_data['pose']
            vehicle_twist = current_data['twist']
                      
            # 统一使用定时器触发时间作为发布时间戳
            pub_time = event.current_real

            # 发布里程计
            true_odom = Odometry()
            true_odom.header.frame_id = 'world'
            true_odom.header.stamp = pub_time
            true_odom.pose.pose = vehicle_pose
            true_odom.twist.twist = vehicle_twist
            self.odom_pub.publish(true_odom)

            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = pub_time
            pose_stamped.header.frame_id = "world"
            pose_stamped.pose = vehicle_pose
            self.latest_path.header.stamp = pub_time
            self.latest_path.poses.append(pose_stamped)

        except Exception as e:
            rospy.logerr_throttle(1.0, f"数据处理异常: {str(e)}")
        
    def timer_cb2(self, event):
        self.path_pub.publish(self.latest_path)

if __name__ == "__main__":
    try:
        node = TrueStatePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点正常退出")