#!/usr/bin/env python
import rospy
import math
import numpy as np
from geometry_msgs.msg import Wrench, Vector3
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest
from gazebo_msgs.msg import LinkState

class DynamicWindController:
    def __init__(self):
        rospy.init_node('dynamic_wind_controller')
        
        # 连接到Gazebo的ApplyBodyWrench服务
        rospy.wait_for_service('/gazebo/apply_body_wrench')
        self.apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        
        # 风场参数
        self.body_name = "iris/base_link"  # 无人机基座链接名称
        self.reference_frame = "world"           # 参考坐标系
        
        # 风场模式配置
        self.mode = "sine"  # 可选模式：sine/random/ramp/gust
        self.wind_force = Vector3()
        
        # 启动定时器（10Hz更新频率）
        self.timer = rospy.Timer(rospy.Duration(0.1), self.update_wind)

    def generate_wind_force(self, current_time):
        if self.mode == "sine":
            # 正弦波风场（X方向）
            amplitude = 1.0    # 力幅值 (N)
            frequency = 0.5    # 频率 (Hz)
            return Vector3(
                x=amplitude * math.sin(2 * math.pi * frequency * current_time),
                y=0.0,
                z=0.0
            )
        
        elif self.mode == "random":
            # 随机阵风（X/Y方向）
            return Vector3(
                x=np.random.normal(1.0, 0.5),
                y=np.random.normal(0.0, 0.3),
                z=0.0
            )
        
        elif self.mode == "ramp":
            # 渐变风场（Y方向）
            ramp_duration = 20.0  # 渐变总时长 (秒)
            max_force = 3.0       # 最大风力 (N)
            force_y = max_force * (current_time / ramp_duration) if current_time < ramp_duration else max_force
            return Vector3(x=0.0, y=force_y, z=0.0)
        
        elif self.mode == "gust":
            # 突发阵风（组合模式）
            base_force = 1.0
            gust_amplitude = 2.5
            gust_period = 15.0
            force_x = base_force + gust_amplitude if (current_time % gust_period) < 5.0 else base_force
            return Vector3(x=force_x, y=0.0, z=0.0)
        
        return Vector3()

    def update_wind(self, event):
        try:
            current_time = rospy.get_time()
            force = self.generate_wind_force(current_time)
            
            # 构建力请求
            req = ApplyBodyWrenchRequest()
            req.body_name = self.body_name
            req.reference_frame = self.reference_frame
            req.wrench = Wrench(force=force, torque=Vector3())
            req.duration = rospy.Duration(0.1)  # 作用时间
            
            self.apply_wrench(req)
        except rospy.ServiceException as e:
            rospy.logerr("风力施加失败: %s", str(e))

if __name__ == '__main__':
    try:
        controller = DynamicWindController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass