#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Vector3, Twist

class PlanningAgent:
    def __init__(self, uav_id):
        self.uav_id = uav_id
        self.model_name = "iris_zsj_ir{}".format(uav_id)
        
        # 红外信号参数
        self.infrared_direction = Vector3()
        self.infrared_strength = 0.0
        self.strength_threshold = 0.5
        
        # 控制参数
        self.base_speed = 0.6
        self.max_speed = 1.0
        
        # 误差控制系数 (0-1) 0=完全禁用误差
        self.speed_error_factor = 0.9  # 速度大小误差系数
        self.direction_error_factor = 0.9 # 方向误差系数
        
        # 订阅红外信号
        rospy.Subscriber('/uav/infrared/receiver/direction_{}'.format(self.model_name), 
                        Vector3, self.infrared_callback)
        
        rospy.loginfo("PlanningAgent {} initialized".format(uav_id))
    
    def set_error_factors(self, speed_factor, direction_factor):
        """
        设置误差控制系数
        Args:
            speed_factor: 速度大小误差系数 (0-1)
            direction_factor: 方向误差系数 (0-1)
        """
        self.speed_error_factor = max(0.0, min(1.0, speed_factor))
        self.direction_error_factor = max(0.0, min(1.0, direction_factor))
        rospy.loginfo("UAV {} error factors set - speed: {:.3f}, direction: {:.3f}".format(
            self.uav_id, self.speed_error_factor, self.direction_error_factor))
    
    def infrared_callback(self, msg):
        """接收红外方向信息"""
        self.infrared_direction = msg
        self.infrared_strength = msg.z
    
    def _add_velocity_error(self, vx, vy, vz):
        """
        为速度添加随机误差
        速度大小可能变小(0-max_speed)，方向可能偏差(-90~90度)
        误差程度由误差控制系数决定
        """
        # 如果没有启用误差，直接返回原速度
        if self.speed_error_factor < 0.001 and self.direction_error_factor < 0.001:
            return vx, vy, vz
        
        # 速度大小误差 - 使用指数分布，误差程度由系数控制
        if self.speed_error_factor > 0.001:
            # 基础误差量，使用指数分布使小误差更可能出现
            base_speed_reduction = np.random.exponential(0.2)
            # 根据误差系数缩放误差程度
            speed_reduction = base_speed_reduction * self.speed_error_factor
            speed_reduction = min(speed_reduction, 1.0)  # 限制在0-1范围内
        else:
            speed_reduction = 0.0
        
        current_speed = np.sqrt(vx**2 + vy**2 + vz**2)
        if current_speed > 0:
            new_speed = current_speed * (1 - speed_reduction)
            speed_scale = new_speed / current_speed
        else:
            speed_scale = 1.0
        
        # 方向误差 - 使用正态分布，误差程度由系数控制
        if self.direction_error_factor > 0.001:
            # 基础角度误差，使用正态分布使小角度偏差更可能出现
            base_angle_error = np.random.normal(0, 15)  # 均值为0，标准差15度
            # 根据误差系数缩放误差程度
            angle_error = base_angle_error * self.direction_error_factor
            angle_error = max(min(angle_error, 90), -90)  # 限制在-90到90度范围内
        else:
            angle_error = 0.0
        
        # 应用方向误差（仅对水平速度）
        if abs(vx) > 0.001 or abs(vy) > 0.001:
            current_angle = np.arctan2(vy, vx)
            new_angle = current_angle + np.radians(angle_error)
            
            # 应用误差后的速度
            vx_error = np.cos(new_angle) * np.sqrt(vx**2 + vy**2) * speed_scale
            vy_error = np.sin(new_angle) * np.sqrt(vx**2 + vy**2) * speed_scale
            vz_error = vz * speed_scale
        else:
            vx_error = vx * speed_scale
            vy_error = vy * speed_scale
            vz_error = vz * speed_scale
        
        rospy.logdebug_throttle(5, "UAV {} velocity error - speed reduction: {:.3f}, angle error: {:.1f}°".format(
            self.uav_id, speed_reduction, angle_error))
        
        return vx_error, vy_error, vz_error
    
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
        
        # 添加速度误差
        vx_error, vy_error, vz_error = self._add_velocity_error(vx, vy, 0.0)
        
        return vx_error, vy_error, vz_error
    
    def get_signal_strength(self):
        """获取当前信号强度"""
        return self.infrared_strength
    
    def get_signal_direction(self):
        """获取当前信号方向"""
        return self.infrared_direction
    
    def get_error_factors(self):
        """获取当前误差控制系数"""
        return self.speed_error_factor, self.direction_error_factor
    
    def stop(self):
        """停止信号处理"""
        pass
        """停止信号处理"""
        pass