# 无人机集群仿真项目

基于ROS、PX4和Gazebo的无人机集群仿真系统。

## 功能特性

- 自动生成任意规模的无人机集群
- 支持网格队形部署
- 基于红外信号的编队控制
- 一键启动仿真环境

## 快速开始

```bash

# 生成并启动4x4无人机集群  
cd /home/zsj/catkin_ws/src/iris_ir_sitl/shell
./run_formation.sh --rows 4 --cols 4 --spacing 2

## 配置

1、model 要放到 px4的模型路径
2、修改px4的single_vehicle_spawn.launch
3、py脚本要赋予执行权限
4、修改文件ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink，重新编译
5、先编译