#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import threading
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped

class MultiDroneController:
    def __init__(self, num_drones=10):
        self.num_drones = num_drones
        self.drones = []
        
        # 无人机命名空间列表
        self.namespaces = [f"wmy_iris_camRay{i}" for i in range(num_drones)]
        
        # 初始化每个无人机的控制器
        for ns in self.namespaces:
            self.drones.append(DroneController(ns))
    
    def initialize_drones(self):
        """初始化所有无人机"""
        rospy.loginfo("正在初始化所有无人机...")
        
        # 等待所有无人机连接
        all_connected = False
        while not all_connected and not rospy.is_shutdown():
            all_connected = all([drone.is_connected() for drone in self.drones])
            rospy.sleep(0.1)
        
        rospy.loginfo("所有无人机均已连接!")
    
    def arm_and_takeoff(self, target_altitude=0.5):
        """让所有无人机解锁并起飞到指定高度"""
        rospy.loginfo(f"开始解锁并起飞所有无人机到 {target_altitude} 米高度...")
        
        threads = []
        
        # 为每个无人机创建线程并行执行
        for i, drone in enumerate(self.drones):
            thread = threading.Thread(
                target=self._single_drone_operation,
                args=(drone, i, target_altitude)
            )
            threads.append(thread)
            thread.start()
        
        # 等待所有线程完成
        for thread in threads:
            thread.join()
        
        rospy.loginfo("所有无人机均已起飞并悬停!")
    
    def _single_drone_operation(self, drone, drone_id, target_altitude):
        """单个无人机的起飞操作"""
        try:
            # 设置OFFBOARD模式
            if drone.set_mode("OFFBOARD"):
                rospy.loginfo(f"无人机 {drone_id} OFFBOARD模式设置成功")
            else:
                rospy.logwarn(f"无人机 {drone_id} OFFBOARD模式设置失败")
                return
            
            # 解锁
            if drone.arm():
                rospy.loginfo(f"无人机 {drone_id} 解锁成功")
            else:
                rospy.logwarn(f"无人机 {drone_id} 解锁失败")
                return
            
            # 起飞到目标高度
            drone.takeoff_to_altitude(target_altitude)
            rospy.loginfo(f"无人机 {drone_id} 已到达目标高度并悬停")
            
        except Exception as e:
            rospy.logerr(f"无人机 {drone_id} 操作失败: {str(e)}")
    
    def run(self):
        """主运行函数"""
        rospy.init_node('multi_drone_controller', anonymous=True)
        rospy.loginfo("多无人机控制器节点启动")
        
        # 等待ROS初始化
        rospy.sleep(2)
        
        # 初始化无人机
        self.initialize_drones()
        
        # 起飞到0.5米高度
        self.arm_and_takeoff(0.5)
        
        # 保持运行
        rospy.loginfo("所有无人机正在悬停，按Ctrl+C退出")
        rospy.spin()


class DroneController:
    def __init__(self, namespace):
        self.namespace = namespace
        self.current_state = State()
        self.local_pos_pub = None
        self.state_sub = None
        self.rate = None
        
        self.initialize_ros_components()
    
    def initialize_ros_components(self):
        """初始化ROS发布器和订阅器"""
        # 状态订阅
        self.state_sub = rospy.Subscriber(
            f'/{self.namespace}/mavros/state',
            State,
            self.state_callback
        )
        
        # 本地位置发布
        self.local_pos_pub = rospy.Publisher(
            f'/{self.namespace}/mavros/setpoint_position/local',
            PoseStamped,
            queue_size=10
        )
        
        self.rate = rospy.Rate(20)  # 20Hz
    
    def state_callback(self, data):
        """状态回调函数"""
        self.current_state = data
    
    def is_connected(self):
        """检查是否连接到飞控"""
        return self.current_state.connected
    
    def is_armed(self):
        """检查是否已解锁"""
        return self.current_state.armed
    
    def get_mode(self):
        """获取当前模式"""
        return self.current_state.mode
    
    def set_mode(self, mode):
        """设置飞行模式"""
        rospy.wait_for_service(f'/{self.namespace}/mavros/set_mode')
        try:
            set_mode_service = rospy.ServiceProxy(
                f'/{self.namespace}/mavros/set_mode',
                SetMode
            )
            response = set_mode_service(0, mode)  # 0表示自定义模式
            return response.mode_sent
        except rospy.ServiceException as e:
            rospy.logerr(f"设置模式服务调用失败: {e}")
            return False
    
    def arm(self):
        """解锁无人机"""
        rospy.wait_for_service(f'/{self.namespace}/mavros/cmd/arming')
        try:
            arming_service = rospy.ServiceProxy(
                f'/{self.namespace}/mavros/cmd/arming',
                CommandBool
            )
            response = arming_service(True)
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"解锁服务调用失败: {e}")
            return False
    
    def send_setpoint(self, x, y, z):
        """发送位置设定点"""
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "base_link"
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        
        # 保持水平姿态
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        
        self.local_pos_pub.publish(pose)
    
    def takeoff_to_altitude(self, target_altitude, duration=10):
        """起飞到指定高度"""
        start_time = rospy.Time.now()
        
        # 发送设定点一段时间以确保OFFBOARD模式切换
        for i in range(100):
            if rospy.is_shutdown():
                return
            self.send_setpoint(0, 0, target_altitude)
            self.rate.sleep()
        
        # 保持目标高度
        while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
            self.send_setpoint(0, 0, target_altitude)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        # 创建10架无人机的控制器
        controller = MultiDroneController(num_drones=10)
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被用户中断")
    except Exception as e:
        rospy.logerr(f"节点运行错误: {e}")