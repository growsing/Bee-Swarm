#!/usr/bin/env python3

# 自动生成无人机集群launch文件，自定义行数列数间距
# 2025/11/12

import os
import argparse
from pathlib import Path

def generate_swarm_launch(rows, cols, spacing, start_z=0.5, output_path=None):
    """
    生成无人机群launch文件
    
    Args:
        rows: 行数
        cols: 列数
        spacing: 间距（米）
        start_z: 起始高度（米）
        output_path: 输出文件路径
    """
    
    launch_template = '''<?xml version="1.0"?>
<launch>
    <arg name="px4_home" default="/home/zsj/PX4_Firmware"/>
    <arg name="paused" default="false"/>
    <arg name="use_sim_time" default="true"/>
    <arg name="gui" default="true"/>
    <arg name="headless" default="false"/>
    <arg name="debug" default="false"/>
    <arg name="verbose" default="true"/>

    <!-- 设置环境变量 -->
    <env name="GAZEBO_PLUGIN_PATH" value="$(arg px4_home)/build/px4_sitl_default/build_gazebo-classic"/>
    <env name="GAZEBO_PLUGIN_PATH" value="$(find iris_ir_sitl)/../../devel/lib:$(optenv GAZEBO_PLUGIN_PATH)"/>
    <env name="GAZEBO_MODEL_PATH" value="$(arg px4_home)/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"/>

    <!-- 启动Gazebo -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="world_name" value="$(arg px4_home)/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world"/>
        <arg name="debug" value="$(arg debug)"/>
        <arg name="gui" value="$(arg gui)"/>
        <arg name="paused" value="$(arg paused)"/>
        <arg name="use_sim_time" value="$(arg use_sim_time)"/>
        <arg name="headless" value="$(arg headless)"/>
        <arg name="verbose" value="$(arg verbose)"/>
    </include>

    <!-- 等待Gazebo完全启动 -->
    <arg name="node_start_delay" default="10.0" />
{drone_groups}
</launch>
'''
    
    drone_template = '''
    <!-- 第{index}架无人机 -->
    <group ns="iris_zsj_ir{id}">
        <arg name="ID" value="{id}"/>
        <arg name="fcu_url" default="udp://:{udp_offboard_port_remote}@localhost:{udp_offboard_port_local}"/>
        <include file="$(find px4)/launch/single_vehicle_spawn.launch">
            <arg name="x" value="{x:.2f}"/>
            <arg name="y" value="{y:.2f}"/>
            <arg name="z" value="{z:.2f}"/>
            <arg name="vehicle" value="iris_zsj_ir"/>
            <arg name="mavlink_udp_port" value="{udp_offboard_port_local}"/>
            <arg name="mavlink_tcp_port" value="{tcp_port}"/>
            <arg name="ID" value="$(arg ID)"/>
        </include>

        <include file="$(find mavros)/launch/px4.launch">
            <arg name="fcu_url" value="$(arg fcu_url)"/>
            <arg name="gcs_url" value=""/>
            <arg name="tgt_system" value="$(eval 1 + arg('ID'))"/>
            <arg name="tgt_component" value="1"/>
        </include>
    </group>'''
    
    drone_groups = []
    drone_count = 0
    
    # 网格队形
    start_x = -((cols - 1) * spacing) / 2
    start_y = -((rows - 1) * spacing) / 2
    
    for row in range(rows):
        for col in range(cols):
            x = start_x + col * spacing
            y = start_y + row * spacing
            z = start_z
            
            # 根据你修改后的PX4脚本分配端口
            # PX4脚本现在使用：
            #   udp_offboard_port_local = 14580 + px4_instance
            #   udp_offboard_port_remote = 10540 + px4_instance
            
            udp_offboard_port_local = 14580 + drone_count  # MAVROS监听端口
            udp_offboard_port_remote = 10540 + drone_count  # PX4发送端口
            
            # TCP端口
            tcp_port = 4560 + drone_count
            
            drone_group = drone_template.format(
                index=drone_count + 1,
                id=drone_count,
                udp_offboard_port_local=udp_offboard_port_local,
                udp_offboard_port_remote=udp_offboard_port_remote,
                tcp_port=tcp_port,
                x=x,
                y=y,
                z=z
            )
            
            drone_groups.append(drone_group)
            drone_count += 1
    
    # 合并所有无人机组
    drone_groups_str = ''.join(drone_groups)
    
    # 生成完整的launch文件内容
    launch_content = launch_template.format(drone_groups=drone_groups_str)
    
    # 写入文件
    with open(output_path, 'w') as f:
        f.write(launch_content)
    
    print(f"成功生成 {rows}x{cols} 的无人机网格队形launch文件")
    print(f"无人机数量: {drone_count}")
    print(f"端口分配:")
    print(f"  - MAVROS监听端口 (mavlink_udp_port): 14580-{14580+drone_count-1}")
    print(f"  - PX4发送端口 (fcu_url远程端口): 10540-{10540+drone_count-1}")
    print(f"  - TCP端口: 4560-{4560+drone_count-1}")
    print(f"文件保存至: {output_path}")

def main():
    parser = argparse.ArgumentParser(description='生成无人机群launch文件')
    parser.add_argument('--rows', type=int, default=3, help='行数 (默认: 3)')
    parser.add_argument('--cols', type=int, default=3, help='列数 (默认: 3)')
    parser.add_argument('--spacing', type=float, default=2.0, help='间距 (米) (默认: 2.0)')
    parser.add_argument('--height', type=float, default=0.5, help='起始高度 (米) (默认: 0.5)')
    parser.add_argument('--output', type=str, 
                       default='/home/zsj/catkin_ws/src/iris_ir_sitl/launch/swarm_formation.launch',
                       help='输出文件路径')
    
    args = parser.parse_args()
    
    # 参数验证
    if args.rows <= 0 or args.cols <= 0:
        print("错误：行数和列数必须大于0")
        return
    
    total_drones = args.rows * args.cols
    
    # 确保输出目录存在
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    
    # 生成launch文件
    generate_swarm_launch(
        rows=args.rows,
        cols=args.cols,
        spacing=args.spacing,
        start_z=args.height,
        output_path=args.output
    )

if __name__ == "__main__":
    main()