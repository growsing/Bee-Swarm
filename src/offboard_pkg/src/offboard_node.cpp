/**
 * 将正方形编队(边长1米)作为一个整体，绕着半径4.75米的圆形轨迹飞行，同时编队自身也会旋转
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

/*
头文件，包括常见的geometry_msgs和mavros通信需要的mavros_msgs，添加上就行
*/
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

/*
current_state表示的是无人机的状态，在主函数中订阅了对应的话题，这个状态就会不断更新，表示无人机当前的状态。
state_cb就是对应的回调函数，会不断的执行，更新状态。现在先不用管为什么需要这个状态，后面的代码会解释。
*/
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    // 初始化ROS节点，节点名称为"offb_node"
    ros::init(argc, argv, "offb_node");

    // 初始化第0架无人机(uav0)的通信接口
    ros::NodeHandle nh0;  // 创建第0架无人机的节点句柄
    // 订阅uav0的飞行状态信息
    ros::Subscriber state_sub0 = nh0.subscribe<mavros_msgs::State>
            ("uav0/mavros/state", 10, state_cb);  
    // 发布uav0的位置控制指令
    /* 
    发布uav0的位置控制指令.发布一个geometry_msgs::PoseStamped的消息，需要知道的是，这个消息是控制无人机的一种方式，
    将指定坐标包裹进这个消息，然后发布出去，无人机就能自动飞行到指定的坐标地点
    */ 
    ros::Publisher local_pos_pub0 = nh0.advertise<geometry_msgs::PoseStamped>
            ("/uav0/mavros/setpoint_position/local", 10);  
    // 创建uav0的解锁服务客户端
    /*
    无人机有一个锁，如果不解锁，无人机虽然接受了命令但是不会动被锁住了，只有解锁了才能对无人机进行控制，
    下面这个服务调用就是用来请求解锁无人机。上面的current_state就包含了无人机是否解锁的信息，若没解锁就需要解锁，否则就不用，其用途在这就体现出来
    */
    ros::ServiceClient arming_client0 = nh0.serviceClient<mavros_msgs::CommandBool>
            ("/uav0/mavros/cmd/arming"); 
    // 创建uav0的飞行模式设置服务客户端
    /*
    无人机飞行有很多种模式，如果需要用代码操控无人机，我们就需要切换到OFFBOARD模式。
    上面的current_state也包含了无人机当前的飞行模式，若不是OFFBOARD就需要切换到该模式。下面的这个服务调用就是用来请求切换无人机飞行模式。
    */
    ros::ServiceClient set_mode_client0 = nh0.serviceClient<mavros_msgs::SetMode>
            ("/uav0/mavros/set_mode");  

    // 初始化第1架无人机(uav1)的通信接口
    ros::NodeHandle nh1;  // 创建第1架无人机的节点句柄
    ros::Subscriber state_sub1 = nh1.subscribe<mavros_msgs::State>
            ("uav1/mavros/state", 10, state_cb);  // 订阅uav1的飞行状态信息
    ros::Publisher local_pos_pub1 = nh1.advertise<geometry_msgs::PoseStamped>
            ("/uav1/mavros/setpoint_position/local", 10);  // 发布uav1的位置控制指令
    ros::ServiceClient arming_client1 = nh1.serviceClient<mavros_msgs::CommandBool>
            ("/uav1/mavros/cmd/arming");  // 创建uav1的解锁服务客户端
    ros::ServiceClient set_mode_client1 = nh1.serviceClient<mavros_msgs::SetMode>
            ("/uav1/mavros/set_mode");  // 创建uav1的飞行模式设置服务客户端

    // 初始化第2架无人机(uav2)的通信接口
    ros::NodeHandle nh2;  // 创建第2架无人机的节点句柄
    ros::Subscriber state_sub2 = nh2.subscribe<mavros_msgs::State>
            ("uav2/mavros/state", 10, state_cb);  // 订阅uav2的飞行状态信息
    ros::Publisher local_pos_pub2 = nh2.advertise<geometry_msgs::PoseStamped>
            ("/uav2/mavros/setpoint_position/local", 10);  // 发布uav2的位置控制指令
    ros::ServiceClient arming_client2 = nh2.serviceClient<mavros_msgs::CommandBool>
            ("/uav2/mavros/cmd/arming");  // 创建uav2的解锁服务客户端
    ros::ServiceClient set_mode_client2 = nh2.serviceClient<mavros_msgs::SetMode>
            ("/uav2/mavros/set_mode");  // 创建uav2的飞行模式设置服务客户端

    // 初始化第3架无人机(uav3)的通信接口
    ros::NodeHandle nh3;  // 创建第3架无人机的节点句柄
    ros::Subscriber state_sub3 = nh3.subscribe<mavros_msgs::State>
            ("uav3/mavros/state", 10, state_cb);  // 订阅uav3的飞行状态信息
    ros::Publisher local_pos_pub3 = nh3.advertise<geometry_msgs::PoseStamped>
            ("/uav3/mavros/setpoint_position/local", 10);  // 发布uav3的位置控制指令
    ros::ServiceClient arming_client3 = nh3.serviceClient<mavros_msgs::CommandBool>
            ("/uav3/mavros/cmd/arming");  // 创建uav3的解锁服务客户端
    ros::ServiceClient set_mode_client3 = nh3.serviceClient<mavros_msgs::SetMode>
            ("/uav3/mavros/set_mode");  // 创建uav3的飞行模式设置服务客户端

    // 设置控制指令发布频率为20Hz，必须大于2Hz才能保持offboard模式
    ros::Rate rate(20.0);

    // 等待飞控连接，确保所有无人机都已连接到飞控
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    
    // 初始化各无人机的目标位置变量
    // x_num, y_num表示对应无人机的期望输入位置
    // 所有无人机保持相同的高度z

    float x0 = 0.0, y0 = 0.0;  // uav0的初始目标位置
    float x1 = 0.0, y1 = 0.0;  // uav1的初始目标位置
    float x2 = 0.0, y2 = 0.0;  // uav2的初始目标位置
    float x3 = 0.0, y3 = 0.0;  // uav3的初始目标位置
    float z = 2.0;  // 所有无人机的飞行高度(2米)
    float w = 0.0;  // 编队旋转角度变量，初始为0
    const float pi = 3.1415926;  // 圆周率常量
    geometry_msgs::PoseStamped pose0;  // uav0的位置消息
    geometry_msgs::PoseStamped pose1;  // uav1的位置消息
    geometry_msgs::PoseStamped pose2;  // uav2的位置消息
    geometry_msgs::PoseStamped pose3;  // uav3的位置消息
    
    // 设置各无人机的初始位置(高度为2米)
    pose0.pose.position.x = x0;
    pose0.pose.position.y = y0;
    pose0.pose.position.z = z;
    pose1.pose.position.x = x1;
    pose1.pose.position.y = y1;
    pose1.pose.position.z = z;
    pose2.pose.position.x = x2;
    pose2.pose.position.y = y2;
    pose2.pose.position.z = z;
    pose3.pose.position.x = x3;
    pose3.pose.position.y = y3;
    pose3.pose.position.z = z;

    // 由于launch配置文件的原因，无人机有初始位置偏移
    float x0_offset = 0;  // uav0的X轴偏移量
    float y0_offset = 0;  // uav0的y轴偏移量
    float x1_offset = 1;  // uav1的X轴偏移量
    float y1_offset = 0;  // uav1的y轴偏移量
    float x2_offset = 0;   // uav2的X轴偏移量
    float y2_offset = 1;  // uav2的y轴偏移量
    float x3_offset = 1;   // uav3的X轴偏移量
    float y3_offset = 1;  // uav3的y轴偏移量

    // 定义各无人机在编队坐标系中的相对位置(形成正方形编队)
    float x0_f = -0.5, y0_f = 0.5;   // uav0在编队中的相对坐标(左上)
    float x1_f = 0.5, y1_f = 0.5;    // uav1在编队中的相对坐标(右上)
    float x2_f = -0.5, y2_f = -0.5;  // uav2在编队中的相对坐标(左下)
    float x3_f = 0.5, y3_f = -0.5;   // uav3在编队中的相对坐标(右下)
    float xx, yy;  // 编队中心坐标变量

    // 在正式起飞前，先发送100个位置控制点
    // 这是PX4固件的要求，确保offboard模式能够正常启动
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub0.publish(pose0);  // 发布uav0的位置指令
        local_pos_pub1.publish(pose1);  // 发布uav1的位置指令
        local_pos_pub2.publish(pose2);  // 发布uav2的位置指令
        local_pos_pub3.publish(pose3);  // 发布uav3的位置指令
        ros::spinOnce();
        rate.sleep();
    }

    // 请求的切换模式的消息，设置为OFFBOARD
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";  // 设置为offboard控制模式

    // 请求解锁的消息，arm表示解锁，设置为true，disarm是上锁
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;  // 请求解锁(arm)所有无人机
    // 记录上一次请求的时间
    ros::Time last_request = ros::Time::now();  

    // 主控制循环
    while(ros::ok()){
        // 如果当前不是offboard模式，且距离上次请求超过5秒，则尝试切换到offboard模式
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            // 为所有4架无人机同时设置offboard模式
            if( set_mode_client0.call(offb_set_mode) &&
                set_mode_client1.call(offb_set_mode) &&
                set_mode_client2.call(offb_set_mode) &&
                set_mode_client3.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled : 4");  // 成功为4架无人机启用offboard模式
            }
            last_request = ros::Time::now();  // 更新请求时间
        } else {
            // 如果无人机未解锁，且距离上次请求超过5秒，则尝试解锁
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                // 为所有4架无人机同时发送解锁命令
                if( arming_client0.call(arm_cmd) &&
                    arming_client1.call(arm_cmd) &&
                    arming_client2.call(arm_cmd) &&
                    arming_client3.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed : 4");  // 成功解锁4架无人机
                }
                last_request = ros::Time::now();  // 更新请求时间
            }
        }

        // 发布各无人机的位置控制指令
        local_pos_pub0.publish(pose0);  // 发布uav0的位置
        local_pos_pub1.publish(pose1);  // 发布uav1的位置
        local_pos_pub2.publish(pose2);  // 发布uav2的位置
        local_pos_pub3.publish(pose3);  // 发布uav3的位置

        // 更新编队旋转角度(每30秒完成一圈旋转)
        w = w + 2*pi/(30/(1/20.0));  // 根据20Hz的控制频率计算角度增量
        if(w > 2*pi){  // 如果角度超过360度，则减去360度
            w = w - 2*pi;
        }
        
        // 计算编队中心的圆形轨迹坐标(半径4.75米)
        xx = 4.75*cos(w);  // 编队中心的X坐标
        yy = 4.75*sin(w);  // 编队中心的Y坐标
        
        // 计算uav0的最终位置(考虑编队旋转、中心位置和偏移量)
        x0 = x0_f*cos(w) - y0_f*sin(w) + xx - x0_offset;  // 旋转后的X坐标 + 中心X坐标 - 偏移量
        y0 = y0_f*cos(w) + x0_f*sin(w) + yy - y0_offset;  // 旋转后的Y坐标 + 中心Y坐标
        pose0.pose.position.x = x0;  // 更新uav0的目标X坐标
        pose0.pose.position.y = y0;  // 更新uav0的目标Y坐标

        // 计算uav1的最终位置(考虑编队旋转、中心位置和偏移量)
        x1 = x1_f*cos(w) - y1_f*sin(w) + xx - x1_offset;  // 旋转后的X坐标 + 中心X坐标 - 偏移量
        y1 = y1_f*cos(w) + x1_f*sin(w) + yy - y1_offset;  // 旋转后的Y坐标 + 中心Y坐标
        pose1.pose.position.x = x1;  // 更新uav1的目标X坐标
        pose1.pose.position.y = y1;  // 更新uav1的目标Y坐标

        // 计算uav2的最终位置(考虑编队旋转、中心位置和偏移量)
        x2 = x2_f*cos(w) - y2_f*sin(w) + xx - x2_offset;  // 旋转后的X坐标 + 中心X坐标 - 偏移量
        y2 = y2_f*cos(w) + x2_f*sin(w) + yy - y2_offset;  // 旋转后的Y坐标 + 中心Y坐标
        pose2.pose.position.x = x2;  // 更新uav2的目标X坐标
        pose2.pose.position.y = y2;  // 更新uav2的目标Y坐标

        // 计算uav3的最终位置(考虑编队旋转、中心位置和偏移量)
        x3 = x3_f*cos(w) - y3_f*sin(w) + xx - x3_offset;  // 旋转后的X坐标 + 中心X坐标 - 偏移量
        y3 = y3_f*cos(w) + x3_f*sin(w) + yy - y3_offset;  // 旋转后的Y坐标 + 中心Y坐标
        pose3.pose.position.x = x3;  // 更新uav3的目标X坐标
        pose3.pose.position.y = y3;  // 更新uav3的目标Y坐标

        ros::spinOnce();  // 处理ROS回调函数
        rate.sleep();     // 按照20Hz频率休眠
    }

    return 0;  // 程序正常退出
}