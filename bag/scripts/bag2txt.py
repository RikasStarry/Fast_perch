#!/usr/bin/env python
import rosbag
import rospy
import tf.transformations as tf
import numpy as np
import sys

def quat_to_euler(quat):
    """将四元数转换为欧拉角 (yaw, pitch, roll)"""
    euler = tf.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w], axes='rzyx')
    return euler[0], euler[1], euler[2]  # yaw, pitch, roll

def process_bag(bag_path):
    # 打开输出文件
    imu1_file = open("/home/gnij/20250317/straight/livox.txt", "a")
    imu2_file = open("/home/gnij/20250317/straight/imu.txt", "a")
    odom_file = open("/home/gnij/20250317/straight/odom.txt", "a")
    
    # 初始化初始时间戳
    first_ts = {
        '/livox/imu': None,
        '/mavros/imu/data_raw': None,
        '/odometry/imu': None
    }
    
    # 遍历bag文件
    with rosbag.Bag(bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            # 检测是否为目标话题
            if topic not in first_ts.keys():
                continue
                
            # 计算相对时间戳
            if first_ts[topic] is None:
                first_ts[topic] = t.to_sec()
            rel_time = t.to_sec() - first_ts[topic]
            
            # 处理IMU类型消息
            if "Imu" in msg._type:
                # 提取角速度和加速度
                wx = msg.angular_velocity.x
                wy = msg.angular_velocity.y
                wz = msg.angular_velocity.z
                ax = msg.linear_acceleration.x
                ay = msg.linear_acceleration.y
                az = msg.linear_acceleration.z
                
                # 写入对应文件
                if topic == '/livox/imu':
                    imu1_file.write(f"{rel_time:.6f} {wx:.6f} {wy:.6f} {wz:.6f} {ax:.6f} {ay:.6f} {az:.6f}\n")
                elif topic == '/mavros/imu/data_raw':
                    imu2_file.write(f"{rel_time:.6f} {wx:.6f} {wy:.6f} {wz:.6f} {ax:.6f} {ay:.6f} {az:.6f}\n")
            
            # 处理Odometry类型消息
            elif "Odometry" in msg._type:
                # 提取位置
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                z = msg.pose.pose.position.z
                
                # 转换四元数为欧拉角
                yaw, pitch, roll = quat_to_euler(msg.pose.pose.orientation)
                
                # 提取线速度和角速度
                vx = msg.twist.twist.linear.x
                vy = msg.twist.twist.linear.y
                vz = msg.twist.twist.linear.z
                wx = msg.twist.twist.angular.x
                wy = msg.twist.twist.angular.y
                wz = msg.twist.twist.angular.z
                
                # 写入文件
                odom_file.write(f"{rel_time:.6f} {x:.6f} {y:.6f} {z:.6f} "
                                f"{yaw:.6f} {pitch:.6f} {roll:.6f} "
                                f"{vx:.6f} {vy:.6f} {vz:.6f} "
                                f"{wx:.6f} {wy:.6f} {wz:.6f}\n")
    
    # 关闭文件
    imu1_file.close()
    imu2_file.close()
    odom_file.close()

if __name__ == "__main__":   
    process_bag("/home/gnij/20250317/straight.bag")
    print("Data extraction completed!")