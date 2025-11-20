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
        self.strength_threshold = 0.5  # 避障阈值
        self.speed_reduction_threshold = 0.3  # 速度减小阈值
        
        # 速度控制参数
        self.base_speed = 1.1  # 最大速度（上限）
        self.min_speed = 0.3   # 最小速度（下限）
        
        # 误差控制系数 (0-1)
        self.direction_error_factor = 0.1  # 方向误差概率
        
        # 无人机当前姿态
        self.current_yaw = 0.0
        self.current_pose = None
        
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
        替代 tf.transformations.euler_from_quaternion
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
    
    def _world_to_body_frame(self, vx_world, vy_world):
        """
        将世界坐标系下的速度转换到机体坐标系
        Args:
            vx_world: 世界坐标系x速度
            vy_world: 世界坐标系y速度
        Returns:
            vx_body, vy_body: 机体坐标系速度
        """
        # 如果无人机没有偏航，直接返回原速度
        if abs(self.current_yaw) < 0.001:
            return vx_world, vy_world
        
        # 坐标系旋转矩阵
        cos_yaw = np.cos(self.current_yaw)
        sin_yaw = np.sin(self.current_yaw)
        
        # 世界坐标系到机体坐标系的转换
        vx_body = vx_world * cos_yaw + vy_world * sin_yaw
        vy_body = -vx_world * sin_yaw + vy_world * cos_yaw
        
        return vx_body, vy_body
    
    def _body_to_world_frame(self, vx_body, vy_body):
        """
        将机体坐标系下的速度转换到世界坐标系
        Args:
            vx_body: 机体坐标系x速度
            vy_body: 机体坐标系y速度
        Returns:
            vx_world, vy_world: 世界坐标系速度
        """
        # 如果无人机没有偏航，直接返回原速度
        if abs(self.current_yaw) < 0.001:
            return vx_body, vy_body
        
        # 坐标系旋转矩阵
        cos_yaw = np.cos(self.current_yaw)
        sin_yaw = np.sin(self.current_yaw)
        
        # 机体坐标系到世界坐标系的转换
        vx_world = vx_body * cos_yaw - vy_body * sin_yaw
        vy_world = vx_body * sin_yaw + vy_body * cos_yaw
        
        return vx_world, vy_world
    
    def _add_velocity_error(self, vx, vy, vz, is_avoiding=False):
        """
        为速度添加方向随机误差
        基于概率模型：以指定概率发生方向误差
        Args:
            is_avoiding: 是否处于避障模式，如果是则不添加方向误差
        """
        # 如果处于避障模式，不添加方向误差
        if is_avoiding:
            return vx, vy, vz
        
        # 计算当前速度大小
        current_speed = np.sqrt(vx**2 + vy**2 + vz**2)
        
        # 方向误差处理（仅对水平速度）
        final_vx, final_vy, final_vz = vx, vy, vz
        
        if abs(vx) > 0.001 or abs(vy) > 0.001:
            if self.direction_error_factor > 0.001 and np.random.random() < self.direction_error_factor:
                # 发生方向误差，在0-360度范围内均匀随机选择新方向
                new_angle = np.random.random() * 2 * np.pi  # 0-2π均匀分布
                final_vx = np.cos(new_angle) * current_speed
                final_vy = np.sin(new_angle) * current_speed
                rospy.logdebug_throttle(5, "UAV {} direction error occurred, new angle: {:.1f}°".format(
                    self.uav_id, np.degrees(new_angle)))
        
        return final_vx, final_vy, final_vz
    
    def calculate_movement_velocity(self, current_pose=None):
        """
        根据红外信号计算移动速度
        返回: (vx, vy, vz) 速度指令（世界坐标系）
        """
        # 更新当前姿态
        self._update_attitude(current_pose)
        
        # 如果没有红外信号，返回零速度
        if self.infrared_strength < 0.001:
            return 0.0, 0.0, 0.0
        
        # 获取红外方向（世界坐标系下的单位向量）
        dir_x_world = self.infrared_direction.x
        dir_y_world = self.infrared_direction.y
        
        # 根据红外强度计算当前速度大小
        current_speed = self._calculate_speed_by_strength(self.infrared_strength)
        
        # 判断是否处于避障模式
        is_avoiding = self.infrared_strength > self.strength_threshold
        
        # 如果信号强度超过避障阈值，反向移动
        if is_avoiding:
            dir_x_world = -dir_x_world
            dir_y_world = -dir_y_world
            rospy.loginfo_throttle(2, "UAV {} avoiding collision (strength: {:.3f}, speed: {:.3f})".format(
                self.uav_id, self.infrared_strength, current_speed))
        else:
            rospy.loginfo_throttle(2, "UAV {} moving toward signal (strength: {:.3f}, speed: {:.3f})".format(
                self.uav_id, self.infrared_strength, current_speed))
        
        # 计算世界坐标系下的速度向量
        vx_world = dir_x_world * current_speed
        vy_world = dir_y_world * current_speed
        vz_world = 0.0
        
        # 将速度转换到机体坐标系进行误差处理
        vx_body, vy_body = self._world_to_body_frame(vx_world, vy_world)
        
        # 在机体坐标系中添加方向误差（避障模式下不添加）
        vx_body_error, vy_body_error, vz_error = self._add_velocity_error(
            vx_body, vy_body, vz_world, is_avoiding)
        
        # 将误差后的速度转换回世界坐标系
        vx_world_error, vy_world_error = self._body_to_world_frame(vx_body_error, vy_body_error)
        
        return vx_world_error, vy_world_error, vz_error
    
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
    
    def stop(self):
        """停止信号处理"""
        pass