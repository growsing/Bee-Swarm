#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Vector3, Twist, PoseStamped

class PlanningAgent:
    def __init__(self, uav_id):
        self.uav_id = uav_id
        self.model_name = "iris_zsj_ir{}".format(uav_id)
        
        # 红外信号参数
        self.infrared_direction = Vector3()
        self.infrared_strength = 0.0
        self.strength_threshold = 0.4  # 避障阈值
        self.speed_reduction_threshold = 0.25  # 速度减小阈值
        
        # 速度控制参数
        self.base_speed = 1.0  # 最大速度（上限）
        self.min_speed = 0.3   # 最小速度（下限）
        self.rotation_speed = 0.8  # 旋转速度（弧度/秒）
        self.rotation_tolerance = 0.1  # 旋转容差（弧度）
        
        # 误差控制系数 (0-1)
        self.direction_error_factor = 0.0  # 方向误差概率
        
        # 无人机当前姿态
        self.current_yaw = 0.0
        self.current_pose = None
        
        # 状态管理
        self.target_yaw = 0.0  # 目标偏航角
        self.is_rotating = False  # 是否正在旋转
        self.last_target_direction = None  # 上次目标方向
        
        # 订阅红外信号
        rospy.Subscriber('/uav/infrared/receiver/direction_{}'.format(self.model_name), 
                        Vector3, self.infrared_callback)
        
        rospy.loginfo("PlanningAgent {} initialized".format(uav_id))
    
    def set_error_factors(self, direction_factor):
        """
        设置误差控制系数
        Args:
            direction_factor: 方向误差概率 (0-1)
        """
        self.direction_error_factor = max(0.0, min(1.0, direction_factor))
        rospy.loginfo("UAV {} error factor set - direction: {:.3f}".format(
            self.uav_id, self.direction_error_factor))
    
    def _calculate_speed_by_strength(self, strength):
        """
        根据红外强度计算速度大小
        红外强度高于阈值后，速度随红外强度增加而非线性减小
        """
        if strength < self.speed_reduction_threshold:
            # 低于阈值，使用最大速度
            return self.base_speed
        elif strength > self.strength_threshold:
            # 高于避障阈值，使用最小速度（接近停止）
            return self.min_speed
        else:
            # 在阈值和避障阈值之间，非线性减小
            # 使用指数衰减函数，使速度变化更平滑
            normalized_strength = (strength - self.speed_reduction_threshold) / (self.strength_threshold - self.speed_reduction_threshold)
            # 指数衰减：从1衰减到0，然后映射到速度范围
            decay_factor = np.exp(-3 * normalized_strength)  # 衰减系数
            speed_range = self.base_speed - self.min_speed
            current_speed = self.min_speed + speed_range * decay_factor
            
            return current_speed
    
    def infrared_callback(self, msg):
        """接收红外方向信息"""
        self.infrared_direction = msg
        self.infrared_strength = msg.z
    
    def _quaternion_to_euler(self, x, y, z, w):
        """
        将四元数转换为欧拉角（偏航角）
        """
        # 计算偏航角 (yaw)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return 0.0, 0.0, yaw  # 只关心偏航角
    
    def _update_attitude(self, pose):
        """从PoseStamped更新无人机姿态"""
        if pose is None:
            return
        
        self.current_pose = pose
        # 从四元数提取偏航角
        orientation = pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = self._quaternion_to_euler(*quaternion)
        self.current_yaw = yaw
    
    def _body_to_world_frame(self, vx_body, vy_body):
        """
        将机体坐标系下的向量转换到世界坐标系
        Args:
            vx_body: 机体坐标系x分量
            vy_body: 机体坐标系y分量
        Returns:
            vx_world, vy_world: 世界坐标系分量
        """
        # 如果无人机没有偏航，直接返回原向量
        if abs(self.current_yaw) < 0.001:
            return vx_body, vy_body
        
        # 坐标系旋转矩阵
        cos_yaw = np.cos(self.current_yaw)
        sin_yaw = np.sin(self.current_yaw)
        
        # 机体坐标系到世界坐标系的转换
        vx_world = vx_body * cos_yaw - vy_body * sin_yaw
        vy_world = vx_body * sin_yaw + vy_body * cos_yaw
        
        return vx_world, vy_world
    
    def _calculate_target_direction(self):
        """
        根据红外信号计算目标移动方向（世界坐标系）
        返回: (target_yaw, is_avoiding)
        """
        # 如果没有红外信号，返回当前方向
        if self.infrared_strength < 0.001:
            return self.current_yaw, False
        
        # 获取红外方向（机体坐标系下的单位向量）
        dir_x_body = self.infrared_direction.x
        dir_y_body = self.infrared_direction.y
        
        # 判断是否处于避障模式
        is_avoiding = self.infrared_strength > self.strength_threshold
        
        # 如果信号强度超过避障阈值，反向移动
        if is_avoiding:
            dir_x_body = -dir_x_body
            dir_y_body = -dir_y_body
        
        # 将机体坐标系的方向转换到世界坐标系
        dir_x_world, dir_y_world = self._body_to_world_frame(dir_x_body, dir_y_body)
        
        # 计算目标偏航角（世界坐标系）
        target_yaw = np.arctan2(dir_y_world, dir_x_world)
        
        # 规范化角度到 [-π, π]
        target_yaw = np.arctan2(np.sin(target_yaw), np.cos(target_yaw))
        
        rospy.logdebug_throttle(5, 
            "UAV {} direction - body: ({:.2f}, {:.2f}), world: ({:.2f}, {:.2f}), target_yaw: {:.1f}°".format(
            self.uav_id, dir_x_body, dir_y_body, dir_x_world, dir_y_world, np.degrees(target_yaw)))
        
        return target_yaw, is_avoiding
    
    def _calculate_rotation_velocity(self, target_yaw):
        """
        计算旋转速度
        Args:
            target_yaw: 目标偏航角
        Returns:
            yaw_rate: 偏航角速度
        """
        # 计算角度差
        angle_diff = target_yaw - self.current_yaw
        
        # 规范化角度差到 [-π, π]
        while angle_diff > np.pi:
            angle_diff -= 2 * np.pi
        while angle_diff < -np.pi:
            angle_diff += 2 * np.pi
        
        # 如果角度差很小，停止旋转
        if abs(angle_diff) < self.rotation_tolerance:
            self.is_rotating = False
            return 0.0
        
        # 设置旋转状态
        self.is_rotating = True
        
        # 计算旋转速度（带符号限制）
        yaw_rate = np.sign(angle_diff) * min(abs(angle_diff), self.rotation_speed)
        
        rospy.logdebug_throttle(2, "UAV {} rotating: current={:.2f}°, target={:.2f}°, diff={:.2f}°, rate={:.2f}rad/s".format(
            self.uav_id, np.degrees(self.current_yaw), np.degrees(target_yaw), 
            np.degrees(angle_diff), yaw_rate))
        
        return yaw_rate
    
    def _calculate_translation_velocity(self, target_yaw, current_speed, is_avoiding):
        """
        计算平移速度
        Args:
            target_yaw: 目标偏航角
            current_speed: 当前速度大小
            is_avoiding: 是否避障模式
        Returns:
            vx, vy: 世界坐标系下的平移速度
        """
        # 如果正在旋转或角度差较大，减小平移速度
        angle_diff = abs(target_yaw - self.current_yaw)
        normalized_angle_diff = min(angle_diff / np.pi, 1.0)  # 归一化到 [0,1]
        
        # 旋转时速度减小，角度差越大速度越小
        rotation_factor = 1.0 - normalized_angle_diff * 0.8  # 最大减小80%
        
        # 如果角度差很大，停止平移
        if angle_diff > np.pi / 3:  # 60度以上
            rotation_factor = 0.6
        
        # 计算前进方向的速度（无人机正前方）
        effective_speed = current_speed * rotation_factor
        
        # 计算世界坐标系下的速度
        vx = np.cos(self.current_yaw) * effective_speed
        vy = np.sin(self.current_yaw) * effective_speed
        
        # 避障模式下不添加方向误差
        if not is_avoiding and self.direction_error_factor > 0.001:
            if np.random.random() < self.direction_error_factor:
                # 发生方向误差，在相机视野范围内随机偏移
                max_error_angle = np.pi / 6  # 最大30度误差（相机视野范围内）
                error_angle = (np.random.random() - 0.5) * 2 * max_error_angle
                vx = np.cos(self.current_yaw + error_angle) * effective_speed
                vy = np.sin(self.current_yaw + error_angle) * effective_speed
                rospy.logdebug_throttle(5, "UAV {} direction error occurred, error angle: {:.1f}°".format(
                    self.uav_id, np.degrees(error_angle)))
        
        return vx, vy
    
    def _add_velocity_error(self, vx, vy, vz, is_avoiding=False):
        """
        为速度添加方向随机误差（现在在平移计算中处理）
        """
        return vx, vy, vz
    
    def calculate_movement_velocity(self, current_pose=None):
        """
        根据红外信号计算移动速度
        返回: (vx, vy, vz, yaw_rate) 速度指令（世界坐标系）和偏航角速度
        """
        # 更新当前姿态
        self._update_attitude(current_pose)
        
        # 如果没有红外信号，返回零速度
        if self.infrared_strength < 0.001:
            self.is_rotating = False
            return 0.0, 0.0, 0.0, 0.0
        
        # 根据红外强度计算当前速度大小
        current_speed = self._calculate_speed_by_strength(self.infrared_strength)
        
        # 计算目标方向
        target_yaw, is_avoiding = self._calculate_target_direction()
        self.target_yaw = target_yaw
        
        # 计算旋转速度
        yaw_rate = self._calculate_rotation_velocity(target_yaw)
        
        # 计算平移速度
        vx, vy = self._calculate_translation_velocity(target_yaw, current_speed, is_avoiding)
        
        vz = 0.0  # 垂直速度由高度控制器处理
        
        # 记录状态信息
        if rospy.get_time() % 5 < 0.1:  # 每5秒输出一次
            status = "AVOIDING" if is_avoiding else "TRACKING"
            rotation_status = "ROTATING" if self.is_rotating else "ALIGNED"
            rospy.loginfo("UAV {} [{}|{}]: current={:.1f}°, target={:.1f}°, speed={:.2f}m/s".format(
                self.uav_id, status, rotation_status, 
                np.degrees(self.current_yaw), np.degrees(target_yaw), 
                np.sqrt(vx**2 + vy**2)))
        
        return vx, vy, vz, yaw_rate
    
    def get_signal_strength(self):
        """获取当前信号强度"""
        return self.infrared_strength
    
    def get_signal_direction(self):
        """获取当前信号方向"""
        return self.infrared_direction
    
    def get_error_factors(self):
        """获取当前误差控制系数"""
        return self.direction_error_factor
    
    def set_speed_parameters(self, base_speed, min_speed, reduction_threshold):
        """
        设置速度相关参数
        Args:
            base_speed: 基础速度（上限）
            min_speed: 最小速度（下限）
            reduction_threshold: 速度减小阈值
        """
        self.base_speed = base_speed
        self.min_speed = min_speed
        self.speed_reduction_threshold = reduction_threshold
        rospy.loginfo("UAV {} speed parameters set - base: {:.3f}, min: {:.3f}, threshold: {:.3f}".format(
            self.uav_id, base_speed, min_speed, reduction_threshold))
    
    def set_rotation_parameters(self, rotation_speed, rotation_tolerance):
        """
        设置旋转相关参数
        Args:
            rotation_speed: 旋转速度（弧度/秒）
            rotation_tolerance: 旋转容差（弧度）
        """
        self.rotation_speed = rotation_speed
        self.rotation_tolerance = rotation_tolerance
        rospy.loginfo("UAV {} rotation parameters set - speed: {:.3f}, tolerance: {:.3f}".format(
            self.uav_id, rotation_speed, rotation_tolerance))
    
    def stop(self):
        """停止信号处理"""
        self.is_rotating = False
        self.target_yaw = self.current_yaw