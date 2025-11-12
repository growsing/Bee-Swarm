#!/usr/bin/env bash
# run_formation.sh  —— 一键替代原 top.launch

# 仿真启动

set -e

# 默认参数
ROWS=3
COLS=3
SPACING=2.0
HEIGHT=0.5

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -r|--rows)
            ROWS="$2"
            shift 2
            ;;
        -c|--cols)
            COLS="$2"
            shift 2
            ;;
        -s|--spacing)
            SPACING="$2"
            shift 2
            ;;
        -z|--height)
            HEIGHT="$2"
            shift 2
            ;;
        -h|--help)
            echo "用法: $0 [选项]"
            echo "选项:"
            echo "  -r, --rows ROWS       行数 (默认: 3)"
            echo "  -c, --cols COLS       列数 (默认: 3)"
            echo "  -s, --spacing SPACING 间距(米) (默认: 2.0)"
            echo "  -z, --height HEIGHT   高度(米) (默认: 0.5)"
            echo "  -h, --help            显示帮助信息"
            exit 0
            ;;
        *)
            echo "未知参数: $1"
            exit 1
            ;;
    esac
done

# 0. 基本检查
if [ -z "$ROS_DISTRO" ]; then
    echo "请先 source ROS 环境（source /opt/ros/noetic/setup.bash）"
    exit 1
fi

# 检查Python脚本是否存在
if [ ! -f "/home/zsj/catkin_ws/src/iris_ir_sitl/scripts/swarm_launch_generate.py" ]; then
    echo "错误：swarm_launch_generate.py 脚本不存在！"
    exit 1
fi

# 0.5 生成launch文件（带参数）
echo ">>> 生成无人机群launch文件..."
echo ">>> 配置: ${ROWS}行 x ${COLS}列, 间距: ${SPACING}米, 高度: ${HEIGHT}米"
python3 /home/zsj/catkin_ws/src/iris_ir_sitl/scripts/swarm_launch_generate.py \
    --rows $ROWS \
    --cols $COLS \
    --spacing $SPACING \
    --height $HEIGHT

if [ $? -ne 0 ]; then
    echo "错误：launch文件生成失败！"
    exit 1
fi
echo ">>> launch文件生成成功"

# 1. 启动仿真（复用当前终端）
echo ">>> 启动多无人机仿真 ..."
roslaunch iris_ir_sitl swarm_formation.launch &
SIM_PID=$!

# 2. 等待仿真完全起来（根据无人机数量动态调整等待时间）
TOTAL_DRONES=$((ROWS * COLS))
if [ $TOTAL_DRONES -le 5 ]; then
    WAIT_TIME=20
elif [ $TOTAL_DRONES -le 10 ]; then
    WAIT_TIME=30
elif [ $TOTAL_DRONES -le 20 ]; then
    WAIT_TIME=45
else
    WAIT_TIME=60
fi

echo ">>> 等待 $WAIT_TIME 秒让Gazebo和 $TOTAL_DRONES 架无人机启动..."
for i in $(seq 1 $WAIT_TIME); do
    echo -ne ">>> 倒计时: $(($WAIT_TIME - $i)) 秒\r"
    sleep 1
done
echo -e "\n>>> 启动完成"

# 3. 检查无人机是否正常启动
echo ">>> 检查无人机状态..."
if rostopic list | grep -q "/iris_zsj_ir0/mavros/state"; then
    echo ">>> 第一架无人机启动成功"
    
    # 统计启动的无人机数量
    DRONE_COUNT=$(rostopic list | grep -c "/iris_zsj_ir[0-9]*/mavros/state" || true)
    echo ">>> 检测到 $DRONE_COUNT 架无人机已启动"
else
    echo ">>> 警告：无人机可能未完全启动，继续执行..."
fi

# 4. 弹新终端跑编队控制器
echo ">>> 打开新终端并启动 formation_controller …"
gnome-terminal -- \
  bash -c "echo 'Starting formation controller for ${ROWS}x${COLS} formation…'; \
           echo 'Spacing: ${SPACING}m, Height: ${HEIGHT}m'; \
           cd /home/zsj/catkin_ws/src/iris_ir_sitl/scripts; \
           python3 uav_formation_controller.py; \
           exec bash"   # 节点结束后不自动关窗，方便看 log

# 5. 可选：挂住脚本，直到用户 Ctrl-C 一键全杀
echo ">>> ==========================================="
echo ">>> 仿真已后台运行，formation controller 已在新终端启动。"
echo ">>> 队形配置: ${ROWS}行 x ${COLS}列"
echo ">>> 按 Ctrl-C 可全部退出。"
echo ">>> ==========================================="

# 设置退出时的清理函数
cleanup() {
    echo -e "\n>>> 正在关闭所有进程..."
    kill $SIM_PID 2>/dev/null || true
    pkill -f "roslaunch iris_ir_sitl" 2>/dev/null || true
    echo ">>> 已退出"
    exit 0
}

trap cleanup SIGINT

wait $SIM_PID