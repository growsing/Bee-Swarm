#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class DroneUnit:
    """单个无人机控制单元类"""
    
    def __init__(self, ns):
        """初始化无人机实例
        Args:
            ns: 无人机命名空间，用于区分不同的无人机
        """
        self.ns = ns  # 命名空间
        self.state = State()  # 存储无人机状态
        self.offboard_good = False  # OFFBOARD模式是否成功设置标志
        self.arm_good = False  # 解锁是否成功标志
        self.last_req = rospy.Time.now()  # 上次请求时间，用于控制请求频率
        self.connection_good = False  # 连接状态标志
            
        # 发布器 - 发布位置设定点
        self.pub_sp = rospy.Publisher('/' + self.ns + "/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        
        # 订阅器 - 订阅无人机状态
        self.sub_state = rospy.Subscriber('/' + self.ns + "/mavros/state", State, self.cb_state)
        
        # 服务客户端 - 等待并创建服务代理
        # 解锁服务
        rospy.wait_for_service('/' + ns + '/mavros/cmd/arming')
        self.cli_arm = rospy.ServiceProxy('/' + ns + '/mavros/cmd/arming', CommandBool)
        # 模式设置服务
        rospy.wait_for_service('/' + ns + '/mavros/set_mode')
        self.cli_mode = rospy.ServiceProxy('/' + ns + '/mavros/set_mode', SetMode)
        
        # 位置设定点初始化
        self.sp = PoseStamped()
        self.sp.pose.orientation.w = 1.0  # 默认朝向
        self.sp.pose.position.x = 0.0  # X坐标
        self.sp.pose.position.y = 0.0  # Y坐标
        self.sp.pose.position.z = 2.0  # Z坐标（高度）
    
    def cb_state(self, msg):
        """无人机状态回调函数
        Args:
            msg: 接收到的状态消息
        """
        self.state = msg
        # 更新连接状态
        if msg.connected and not self.connection_good:
            rospy.loginfo("%s connected to MAVROS", self.ns)
            self.connection_good = True
        elif not msg.connected and self.connection_good:
            rospy.logwarn("%s disconnected from MAVROS", self.ns)
            self.connection_good = False

    def try_set_mode(self, mode):
        """尝试设置无人机飞行模式
        Args:
            mode: 目标飞行模式（如"OFFBOARD"）
        Returns:
            bool: 设置是否成功
        """
        try:
            resp = self.cli_mode(custom_mode=mode)
            rospy.loginfo("%s: Set mode to %s - %s", self.ns, mode, "success" if resp.mode_sent else "failed")
            return resp.mode_sent
        except rospy.ServiceException as e:
            rospy.logerr("%s: Service call failed: %s", self.ns, e)
            return False
        
    def try_arm(self, arm):
        """尝试解锁或锁定无人机
        Args:
            arm: True为解锁，False为锁定
        Returns:
            bool: 操作是否成功
        """
        try:
            resp = self.cli_arm(value=arm)
            rospy.loginfo("%s: Arming %s - %s", self.ns, "enable" if arm else "disable", "success" if resp.success else "failed")
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr("%s: Service call failed: %s", self.ns, e)
            return False
            
    def publish_sp(self):
        """发布位置设定点"""
        self.sp.header.stamp = rospy.Time.now()  # 设置时间戳
        self.sp.header.frame_id = "base_link"  # 设置坐标系
        self.pub_sp.publish(self.sp)  # 发布消息
        
    def get_status(self):
        """返回无人机状态字符串"""
        return f"{self.ns}: Connected={self.state.connected}, Armed={self.state.armed}, Mode={self.state.mode}"

class MultiDroneController:
    """多无人机控制器类"""
    
    def __init__(self):
        """初始化多无人机控制器"""
        self.drones = []  # 无人机列表
        # 定义10个无人机的命名空间
        drone_names = ['wmy_iris_camRay0', 'wmy_iris_camRay1', 'wmy_iris_camRay2', 
                      'wmy_iris_camRay3', 'wmy_iris_camRay4', 'wmy_iris_camRay5',
                      'wmy_iris_camRay6', 'wmy_iris_camRay7', 'wmy_iris_camRay8', 
                      'wmy_iris_camRay9']
        
        # 为每个无人机名称创建无人机实例
        for name in drone_names:
            try:
                drone = DroneUnit(name)
                self.drones.append(drone)
                rospy.loginfo("Initialized drone: %s", name)
            except Exception as e:
                rospy.logerr("Failed to initialize drone %s: %s", name, e)

    def run(self):
        """运行多无人机控制主循环"""
        rate = rospy.Rate(20)  # 控制循环频率为20Hz

        # Stage 1: 发送初始位置点（OFFBOARD模式必需的前置步骤）
        rospy.loginfo("Stage 1: Sending initial setpoints...")
        for i in range(100):  # 发送100个点，确保PX4收到足够的位置信息
            if rospy.is_shutdown():  # 检查ROS是否关闭
                return
            for drone in self.drones:
                drone.publish_sp()  # 每个无人机发布位置设定点
            rate.sleep()  # 保持循环频率

        # Stage 2: OFFBOARD模式设置和解锁
        rospy.loginfo("Stage 2: Offboard and arming...")
        
        max_wait_time = 60.0  # 最大等待时间60秒
        start_time = rospy.Time.now()  # 开始时间
        stage2_start = rospy.Time.now()  # 阶段2开始时间
        status_print_time = rospy.Time.now()  # 状态打印时间
        
        while not rospy.is_shutdown():
            current_time = (rospy.Time.now() - start_time).to_sec()
            # 检查是否超时
            if current_time > max_wait_time:
                rospy.logwarn("Timeout waiting for all drones to be ready")
                # 打印每个无人机的状态
                for drone in self.drones:
                    rospy.loginfo(drone.get_status())
                break

            # 持续发布位置信息（OFFBOARD模式必须持续接收位置信息）
            for drone in self.drones:
                drone.publish_sp()

            all_ready = True  # 所有无人机是否就绪标志
            current_loop_time = rospy.Time.now()  # 当前循环时间
            
            # 检查每个无人机的状态
            for drone in self.drones:
                # 检查连接状态
                if not drone.state.connected:
                    rospy.logwarn_throttle(10, "%s waiting for connection...", drone.ns)  # 每10秒打印一次警告
                    all_ready = False
                    continue

                # 设置OFFBOARD模式（先设置模式后解锁）
                if not drone.offboard_good:
                    # 检查是否需要发送模式设置请求（每5秒一次）
                    if drone.state.mode != "OFFBOARD" and (current_loop_time - drone.last_req) > rospy.Duration(5.0):
                        if drone.try_set_mode("OFFBOARD"):
                            drone.offboard_good = True
                            rospy.loginfo("%s successfully entered OFFBOARD mode!", drone.ns)
                        drone.last_req = current_loop_time  # 更新最后请求时间
                    elif drone.state.mode == "OFFBOARD":  # 如果已经在OFFBOARD模式
                        drone.offboard_good = True
                        rospy.loginfo("%s is already in OFFBOARD mode", drone.ns)

                # 如果还没进入OFFBOARD模式，继续等待
                if not drone.offboard_good:
                    all_ready = False
                    continue
                
                # 尝试解锁（在进入OFFBOARD模式后）
                if not drone.arm_good:
                    if not drone.state.armed and (current_loop_time - drone.last_req) > rospy.Duration(5.0):
                        if drone.try_arm(True):  # 发送解锁请求
                            drone.arm_good = True
                            rospy.loginfo("%s successfully armed!", drone.ns)
                        drone.last_req = current_loop_time
                    elif drone.state.armed:  # 如果已经解锁
                        drone.arm_good = True
                        rospy.loginfo("%s is already armed", drone.ns)
                
                # 如果还没解锁，继续等待
                if not drone.arm_good:
                    all_ready = False

            # 定期打印状态（每5秒）
            if (rospy.Time.now() - status_print_time).to_sec() > 5.0:
                rospy.loginfo("Current status summary:")
                ready_count = 0  # 就绪无人机计数
                for drone in self.drones:
                    status = "READY" if (drone.arm_good and drone.offboard_good) else "NOT READY"
                    if drone.arm_good and drone.offboard_good:
                        ready_count += 1
                    rospy.loginfo("  %s: %s - %s", drone.ns, status, drone.get_status())
                rospy.loginfo("Ready: %d/%d drones", ready_count, len(self.drones))
                status_print_time = rospy.Time.now()  # 更新状态打印时间
            
            # 如果所有无人机都就绪，退出循环
            if all_ready:
                rospy.loginfo("All drones are ready for takeoff!")
                break
                
            rate.sleep()  # 保持循环频率

        # 如果有无人机未就绪，记录信息
        if not all_ready:
            rospy.logerr("Not all drones are ready, attempting to continue with available drones...")
            # 分类就绪和未就绪的无人机
            ready_drones = []
            not_ready_drones = []
            for drone in self.drones:
                if drone.arm_good and drone.offboard_good:
                    ready_drones.append(drone.ns)
                else:
                    not_ready_drones.append(drone.ns)
            
            rospy.loginfo("Ready drones: %s", ready_drones)
            rospy.loginfo("Not ready drones: %s", not_ready_drones)

        # Stage 3: 起飞阶段
        rospy.loginfo("Stage 3: Taking off...")
        target_height = 2.0  # 目标高度2米
        takeoff_duration = 5.0  # 起飞持续时间5秒
        takeoff_start_time = rospy.Time.now()  # 起飞开始时间

        while not rospy.is_shutdown():
            current_time = (rospy.Time.now() - takeoff_start_time).to_sec()  # 当前起飞时间

            # 检查是否完成起飞
            if current_time > takeoff_duration:
                break

            # 计算当前高度（线性插值）
            progress = current_time / takeoff_duration  # 起飞进度（0-1）
            current_height = target_height * progress  # 当前目标高度

            # 为每个无人机设置高度并发布
            for drone in self.drones:
                drone.sp.pose.position.z = current_height
                drone.publish_sp()

            rate.sleep()
        
        rospy.loginfo("All drones have reached the desired altitude!")

        # 保持高度阶段
        while not rospy.is_shutdown():
            for drone in self.drones:
                drone.sp.pose.position.z = target_height  # 保持目标高度
                drone.publish_sp()
            rate.sleep()

        rospy.loginfo("Mission accomplished!")

if __name__ == '__main__':
    """主程序入口"""
    try:
        # 初始化ROS节点
        rospy.init_node('multi_drone_controller', anonymous=True)
        rospy.loginfo("Multi Drone Controller is starting...")

        # 创建多无人机控制器并运行
        controller = MultiDroneController()
        controller.run()

    except rospy.ROSInterruptException:
        rospy.loginfo("Multi drone controller shutdown.")
    except Exception as e:
        rospy.logerr("Error in multi-drone controller: %s", str(e))