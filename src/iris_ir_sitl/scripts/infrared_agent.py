#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Vector3, Twist

class InfraredAgent:
    def __init__(self, uav_id):
        self.uav_id = uav_id
        self.model_name = "iris_zsj_ir{}".format(uav_id)
        
        # 红外信号参数
        self.infrared_direction = Vector3()
        self.infrared_strength = 0.0
        self.strength_threshold = 0.5
        
        # 控制参数
        self.base_speed = 0.5
        self.max_speed = 1.0
        
        # 订阅红外信号
        rospy.Subscriber('/uav/infrared/receiver/direction_{}'.format(self.model_name), 
                        Vector3, self.infrared_callback)
        
        rospy.loginfo("InfraredAgent {} initialized".format(uav_id))
    
    def infrared_callback(self, msg):
        """接收红外方向信息"""
        self.infrared_direction = msg
        self.infrared_strength = msg.z
    
    def calculate_movement_velocity(self, current_pose=None):
        """
        根据红外信号计算移动速度
        返回: (vx, vy, vz) 速度指令
        """
        # 如果没有红外信号，返回零速度
        if self.infrared_strength < 0.001:
            return 0.0, 0.0, 0.0
        
        # 获取红外方向
        dir_x = self.infrared_direction.x
        dir_y = self.infrared_direction.y
        
        # 如果信号强度超过阈值，反向移动
        if self.infrared_strength > self.strength_threshold:
            dir_x = -dir_x
            dir_y = -dir_y
            rospy.loginfo_throttle(2, "UAV {} avoiding collision (strength: {:.3f})".format(
                self.uav_id, self.infrared_strength))
        else:
            rospy.loginfo_throttle(2, "UAV {} moving toward signal (strength: {:.3f})".format(
                self.uav_id, self.infrared_strength))
        
        # 计算水平速度
        vx = dir_x * self.base_speed
        vy = dir_y * self.base_speed
        
        # 限制最大速度
        speed_xy = np.sqrt(vx**2 + vy**2)
        if speed_xy > self.max_speed:
            scale = self.max_speed / speed_xy
            vx *= scale
            vy *= scale
        
        return vx, vy, 0.0
    
    def get_signal_strength(self):
        """获取当前信号强度"""
        return self.infrared_strength
    
    def get_signal_direction(self):
        """获取当前信号方向"""
        return self.infrared_direction
    
    def stop(self):
        """停止信号处理"""
        pass