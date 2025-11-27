#!/usr/bin/env python
# -*- coding: utf-8 -*-

import csv
import os
import time
from datetime import datetime
import rospy

class DataLogger:
    def __init__(self, uav_id, experiment_timestamp, log_dir="experiment_data"):
        self.uav_id = uav_id
        # 用外部传进来的统一时间戳
        timestamp = experiment_timestamp
        
        # 创建实验目录和raw_data子目录
        experiment_dir = os.path.join(log_dir, f"experiment_{timestamp}")
        raw_data_dir = os.path.join(experiment_dir, "raw_data")
        if not os.path.exists(raw_data_dir):
            os.makedirs(raw_data_dir)

        self.filename = os.path.join(raw_data_dir, f"uav_{uav_id}_experiment_{timestamp}.csv")
        
        # 初始化记录状态
        self.is_logging = False
        self.csv_file = None
        self.csv_writer = None
        
        # 设置CSV文件
        self.setup_csv()
        
        rospy.loginfo(f"DataLogger for UAV {uav_id} initialized. Log file: {self.filename}")
    
    def setup_csv(self):
        """创建CSV文件并写入表头"""
        try:
            self.csv_file = open(self.filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            
            # 写入表头
            header = [
                'timestamp', 'sim_time', 
                'pos_x', 'pos_y', 'pos_z',
                'vel_x', 'vel_y', 'vel_z',
                'yaw', 'target_yaw', 'yaw_rate',
                'infrared_strength', 'infrared_dir_x', 'infrared_dir_y',
                'is_avoiding', 'current_speed',
                'vx_cmd', 'vy_cmd', 'vz_cmd', 'yaw_rate_cmd',
                'altitude_error', 'control_mode'
            ]
            self.csv_writer.writerow(header)
            self.csv_file.flush()
            self.is_logging = True
            
            rospy.loginfo(f"CSV file created for UAV {self.uav_id} with header")
            
        except Exception as e:
            rospy.logerr(f"Failed to create CSV file for UAV {self.uav_id}: {e}")
            self.is_logging = False
    
    def log_data(self, data_dict):
        """
        记录数据到CSV
        Args:
            data_dict: 包含所有要记录数据的字典
        """
        if not self.is_logging or self.csv_writer is None:
            return
        
        try:
            # 提取数据，如果不存在则使用默认值
            row = [
                data_dict.get('timestamp', time.time()),
                data_dict.get('sim_time', rospy.get_time()),
                data_dict.get('pos_x', 0.0),
                data_dict.get('pos_y', 0.0),
                data_dict.get('pos_z', 0.0),
                data_dict.get('vel_x', 0.0),
                data_dict.get('vel_y', 0.0),
                data_dict.get('vel_z', 0.0),
                data_dict.get('yaw', 0.0),
                data_dict.get('target_yaw', 0.0),
                data_dict.get('yaw_rate', 0.0),
                data_dict.get('infrared_strength', 0.0),
                data_dict.get('infrared_dir_x', 0.0),
                data_dict.get('infrared_dir_y', 0.0),
                data_dict.get('is_avoiding', False),
                data_dict.get('current_speed', 0.0),
                data_dict.get('vx_cmd', 0.0),
                data_dict.get('vy_cmd', 0.0),
                data_dict.get('vz_cmd', 0.0),
                data_dict.get('yaw_rate_cmd', 0.0),
                data_dict.get('altitude_error', 0.0),
                data_dict.get('control_mode', 'UNKNOWN')
            ]
            
            self.csv_writer.writerow(row)
            self.csv_file.flush()
            
        except Exception as e:
            rospy.logerr(f"Error writing data for UAV {self.uav_id}: {e}")
    
    def stop_logging(self):
        """停止数据记录并关闭文件"""
        if self.csv_file:
            try:
                self.csv_file.close()
                rospy.loginfo(f"Data logging stopped for UAV {self.uav_id}. File: {self.filename}")
            except Exception as e:
                rospy.logerr(f"Error closing CSV file for UAV {self.uav_id}: {e}")
        
        self.is_logging = False
        self.csv_file = None
        self.csv_writer = None
    
    def get_log_file_path(self):
        """获取日志文件路径"""
        return self.filename
    
    def __del__(self):
        """析构函数，确保文件被关闭"""
        self.stop_logging()