#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <map>
#include <cmath>
#include <mutex>

namespace gazebo
{
// 全局标志，表示ROS是否已初始化
static bool g_ros_initialized_receiver = false;
static std::mutex g_ros_mutex_receiver;

class InfraredReceiverPlugin : public ModelPlugin
{
public:
    InfraredReceiverPlugin() : 
        model(nullptr), 
        updateCount(0),
        discoveryCount(0) 
    {}

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        // 存储模型指针
        this->model = _model;
        if (!this->model) {
            gzerr << "Received NULL model pointer!" << std::endl;
            return;
        }

        this->modelName = this->model->GetName();
        gzmsg << "🎯 === InfraredReceiverPlugin Loading for: " << this->modelName << " ===" << std::endl;

        // 安全初始化ROS
        if (!this->initializeROS()) {
            gzerr << "❌ Failed to initialize ROS for receiver " << this->modelName << std::endl;
            return;
        }

        // 连接更新事件
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&InfraredReceiverPlugin::OnUpdate, this, std::placeholders::_1));

        // 创建定时器来发现其他无人机
        this->discoveryTimer = this->rosNode->createTimer(
            ros::Duration(3.0), &InfraredReceiverPlugin::DiscoverOtherUavs, this);

        gzmsg << "🎉 === InfraredReceiverPlugin SUCCESS for: " << this->modelName << " ===" << std::endl;
    }

private:
    bool initializeROS()
    {
        std::lock_guard<std::mutex> lock(g_ros_mutex_receiver);
        
        // 如果ROS已经初始化，直接创建节点
        if (g_ros_initialized_receiver) {
            return this->createROSNode();
        }
        
        // 第一次初始化ROS
        std::string node_name = "infrared_receiver_master";
        gzmsg << "Initializing ROS for receiver with node: " << node_name << std::endl;
        
        int argc = 0;
        char **argv = NULL;
        
        // 使用NoSigintHandler避免信号处理冲突
        ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
        
        if (!ros::master::check()) {
            gzerr << "ROS master is not running!" << std::endl;
            return false;
        }
        
        g_ros_initialized_receiver = true;
        gzmsg << "✅ ROS initialized successfully for receivers" << std::endl;
        
        return this->createROSNode();
    }
    
    bool createROSNode()
    {
        try {
            // 创建节点句柄
            this->rosNode.reset(new ros::NodeHandle());
            
            // 创建发布器 - 方向信息
            std::string direction_topic = "/uav/infrared/receiver/direction_" + this->modelName;
            this->directionPub = this->rosNode->advertise<geometry_msgs::Vector3>(direction_topic, 10);
            
            gzmsg << "✅ Receiver " << this->modelName << " publishing direction to: " << direction_topic << std::endl;
            
            return true;
            
        } catch (const std::exception& e) {
            gzerr << "❌ Failed to create ROS node for receiver: " << e.what() << std::endl;
            return false;
        }
    }

    void DiscoverOtherUavs(const ros::TimerEvent& event)
    {
        if (!this->rosNode) return;

        discoveryCount++;
        
        // 每3次发现才输出一次日志，避免太频繁
        if (discoveryCount % 3 == 1) {
            gzmsg << "🔍 Receiver " << this->modelName << " discovering other UAVs (attempt " << discoveryCount << ")..." << std::endl;
        }

        // 获取所有已发布的红外发射器话题
        ros::master::V_TopicInfo topics;
        ros::master::getTopics(topics);
        
        int new_subscriptions = 0;
        for (const auto& topic : topics) {
            std::string topic_name = topic.name;
            
            // 查找红外发射器话题
            if (topic_name.find("/uav/infrared/emitter/") != std::string::npos) {
                // 提取无人机名称
                size_t pos = topic_name.find_last_of("/");
                std::string uav_name = topic_name.substr(pos + 1);
                
                // 如果不是自己且尚未订阅
                if (uav_name != this->modelName && 
                    this->otherUavSubscriptions.find(uav_name) == this->otherUavSubscriptions.end()) {
                    
                    // 订阅这个无人机的话题
                    try {
                        ros::Subscriber sub = this->rosNode->subscribe<geometry_msgs::Vector3>(
                            topic_name, 10,
                            boost::bind(&InfraredReceiverPlugin::OnOtherUavSignal, this, _1, uav_name));
                        
                        this->otherUavSubscriptions[uav_name] = sub;
                        new_subscriptions++;
                        
                        if (discoveryCount % 3 == 1) {
                            gzmsg << "   ✅ Subscribed to: " << uav_name << std::endl;
                        }
                    } catch (const std::exception& e) {
                        gzerr << "   ❌ Failed to subscribe to " << uav_name << ": " << e.what() << std::endl;
                    }
                }
            }
        }
        
        if (new_subscriptions > 0 && discoveryCount % 3 == 1) {
            gzmsg << "   📡 Total subscriptions: " << this->otherUavSubscriptions.size() << std::endl;
        }
    }
    
    void OnOtherUavSignal(const geometry_msgs::Vector3::ConstPtr& msg, const std::string& emitter_name)
    {
        // 存储其他无人机的位置和速度信息
        std::lock_guard<std::mutex> lock(this->dataMutex);
        
        // 检查是否已经有这个发射器的历史数据
        if (this->otherUavData.find(emitter_name) != this->otherUavData.end()) {
            // 计算速度（位置变化率）
            auto& data = this->otherUavData[emitter_name];
            double dt = 0.5; // 假设更新频率为2Hz
            
            // 计算速度
            ignition::math::Vector3d new_pos(msg->x, msg->y, msg->z);
            ignition::math::Vector3d velocity = (new_pos - data.position) / dt;
            
            // 更新位置和速度
            data.position = new_pos;
            data.velocity = velocity;
            data.last_update_time = this->model->GetWorld()->SimTime();
        } else {
            // 第一次收到这个发射器的数据
            UavData new_data;
            new_data.position = ignition::math::Vector3d(msg->x, msg->y, msg->z);
            new_data.velocity = ignition::math::Vector3d(0, 0, 0); // 初始速度为0
            new_data.last_update_time = this->model->GetWorld()->SimTime();
            this->otherUavData[emitter_name] = new_data;
        }
    }
    
    // 计算四个接收器阵列的位置（相对于无人机中心）
    std::vector<ignition::math::Vector3d> getReceiverPositions(const ignition::math::Pose3d& dronePose)
    {
        // 假设无人机为X型布局，四个接收器位于机臂末端
        // 机臂长度假设为0.3米
        double arm_length = 0.3;
        
        std::vector<ignition::math::Vector3d> positions;
        
        // 四个方向：前右、前左、后右、后左
        positions.push_back(ignition::math::Vector3d(arm_length, -arm_length, 0));  // 前右
        positions.push_back(ignition::math::Vector3d(arm_length, arm_length, 0));   // 前左  
        positions.push_back(ignition::math::Vector3d(-arm_length, -arm_length, 0)); // 后右
        positions.push_back(ignition::math::Vector3d(-arm_length, arm_length, 0));  // 后左
        
        // 应用无人机的旋转到接收器位置
        for (auto& pos : positions) {
            pos = dronePose.Rot().RotateVector(pos) + dronePose.Pos();
        }
        
        return positions;
    }
    
    // 计算每个接收器的朝向（从无人机中心指向接收器位置）
    std::vector<ignition::math::Vector3d> getReceiverOrientations(const ignition::math::Pose3d& dronePose,
                                                                 const std::vector<ignition::math::Vector3d>& receiverPositions)
    {
        std::vector<ignition::math::Vector3d> orientations;
        ignition::math::Vector3d droneCenter = dronePose.Pos();
        
        for (const auto& receiverPos : receiverPositions) {
            // 计算从无人机中心指向接收器的方向向量
            ignition::math::Vector3d orientation = receiverPos - droneCenter;
            orientation.Normalize();
            orientations.push_back(orientation);
        }
        
        return orientations;
    }
    
    // 计算运动方向增强因子
    double calculateMotionDirectionFactor(const ignition::math::Vector3d& emitterPos,
                                         const ignition::math::Vector3d& emitterVelocity,
                                         const ignition::math::Vector3d& receiverPos)
    {
        // 如果速度很小，不应用方向增强
        if (emitterVelocity.Length() < 0.1) {
            return 1.0;
        }
        
        // 计算从发射器指向接收器的方向
        ignition::math::Vector3d toReceiver = receiverPos - emitterPos;
        toReceiver.Normalize();
        
        // 归一化发射器速度方向
        ignition::math::Vector3d emitterDirection = emitterVelocity;
        emitterDirection.Normalize();
        
        // 计算两个方向之间的夹角余弦值
        double cosAngle = toReceiver.Dot(emitterDirection);
        
        // 计算夹角（弧度）
        double angle = std::acos(std::max(-1.0, std::min(1.0, cosAngle)));
        
        // 45度 = π/4 弧度
        double half_angle_range = M_PI / 4.0; // 45度，总共90度范围
        
        // 如果在运动方向的前方90度范围内，信号强度加倍
        if (angle <= half_angle_range) {
            return 3.0; // 前方90度范围内，信号强度增强
        } else {
            return 1.0; // 其他270度范围，正常信号强度
        }
    }
    
    // 计算信号强度（考虑距离、方向和运动方向）
    double calculateSignalStrength(const ignition::math::Vector3d& receiverPos, 
                                  const ignition::math::Vector3d& emitterPos,
                                  const ignition::math::Vector3d& receiverOrientation,
                                  const ignition::math::Vector3d& emitterVelocity)
    {
        // 计算距离
        double distance = receiverPos.Distance(emitterPos);
        
        // 计算方向向量（从接收器指向发射器）
        ignition::math::Vector3d direction = emitterPos - receiverPos;
        direction.Normalize();
        
        // 计算方向夹角余弦值
        double cosAngle = direction.Dot(receiverOrientation);
        
        // 信号强度与距离平方成反比，与方向余弦值成正比
        // 使用平滑函数处理方向影响
        double directionFactor = std::max(0.0, cosAngle); // 只考虑前方180度
        
        // 添加方向灵敏度 - 当信号源在接收器正前方时信号最强
        double sensitivity = 1.0 + 2.0 * directionFactor; // 前方信号增强
        
        // 计算运动方向增强因子
        double motionFactor = calculateMotionDirectionFactor(emitterPos, emitterVelocity, receiverPos);
        
        double strength = motionFactor * sensitivity * directionFactor / (1.0 + distance * distance);
        
        return strength;
    }
    
    void OnUpdate(const common::UpdateInfo &_info)
    {
        // 安全检查
        if (!this->model || !this->rosNode || !this->directionPub) {
            return;
        }

        // 初始化时间
        if (this->lastUpdateTime == common::Time::Zero) {
            this->lastUpdateTime = _info.simTime;
        }

        // 控制更新频率 (2Hz)
        double updateRate = 2.0;
        if ((_info.simTime - this->lastUpdateTime).Double() < 1.0 / updateRate) {
            return;
        }
        this->lastUpdateTime = _info.simTime;

        // 获取无人机位置和方向
        ignition::math::Pose3d dronePose = this->model->WorldPose();
        
        // 获取四个接收器的位置
        std::vector<ignition::math::Vector3d> receiverPositions = getReceiverPositions(dronePose);
        
        // 获取四个接收器的朝向（从无人机中心指向接收器）
        std::vector<ignition::math::Vector3d> receiverOrientations = getReceiverOrientations(dronePose, receiverPositions);

        std::lock_guard<std::mutex> lock(this->dataMutex);
        
        // 为四个接收器初始化信号强度
        std::vector<double> receiverStrengths(4, 0.0);
        
        // 计算每个接收器从所有发射器接收到的总信号强度
        for (const auto& pair : this->otherUavData) {
            const UavData& emitter_data = pair.second;
            ignition::math::Vector3d emitterPos = emitter_data.position;
            ignition::math::Vector3d emitterVelocity = emitter_data.velocity;
            
            // 为每个接收器计算信号强度
            for (int i = 0; i < 4; i++) {
                double strength = calculateSignalStrength(
                    receiverPositions[i], 
                    emitterPos, 
                    receiverOrientations[i],
                    emitterVelocity);
                receiverStrengths[i] += strength;
            }
        }
        
        // 找出信号最强的接收器
        int strongestReceiver = -1;
        double maxStrength = 0.0;
        for (int i = 0; i < 4; i++) {
            if (receiverStrengths[i] > maxStrength) {
                maxStrength = receiverStrengths[i];
                strongestReceiver = i;
            }
        }
        
        // 发布方向信息
        geometry_msgs::Vector3 directionMsg;
        
        if (strongestReceiver >= 0 && maxStrength > 0.001) {
            // 根据最强接收器的位置确定方向
            // 方向编码: x=前后方向(-1后,1前), y=左右方向(-1右,1左), z=信号强度
            switch (strongestReceiver) {
                case 0: // 前右
                    directionMsg.x = 1; directionMsg.y = -1; break;
                case 1: // 前左
                    directionMsg.x = 1; directionMsg.y = 1; break;
                case 2: // 后右
                    directionMsg.x = -1; directionMsg.y = -1; break;
                case 3: // 后左
                    directionMsg.x = -1; directionMsg.y = 1; break;
            }
            directionMsg.z = maxStrength;
            
            // 发布方向信息
            this->directionPub.publish(directionMsg);
            
            // 减少日志频率
            if (this->updateCount % 20 == 0) {
                std::string directionStr;
                switch (strongestReceiver) {
                    case 0: directionStr = "前右"; break;
                    case 1: directionStr = "前左"; break;
                    case 2: directionStr = "后右"; break;
                    case 3: directionStr = "后左"; break;
                }
                
                gzmsg << "🧭 Receiver " << this->modelName << ": strongest signal from " 
                      << directionStr << " (strength: " << std::fixed << std::setprecision(4) 
                      << maxStrength << ")" << std::endl;
                
                // 调试输出：显示所有接收器的信号强度
                gzmsg << "     Signal strengths: FR=" << std::setprecision(3) << receiverStrengths[0]
                      << ", FL=" << receiverStrengths[1]
                      << ", RR=" << receiverStrengths[2] 
                      << ", RL=" << receiverStrengths[3] << std::endl;
            }
        } else {
            // 没有检测到有效信号
            directionMsg.x = 0;
            directionMsg.y = 0;
            directionMsg.z = 0;
            this->directionPub.publish(directionMsg);
            
            // 减少日志频率
            if (this->updateCount % 40 == 0) {
                gzmsg << "🔇 Receiver " << this->modelName << ": no signals detected" << std::endl;
            }
        }
        
        this->updateCount++;
    }

    // 无人机数据结构
    struct UavData {
        ignition::math::Vector3d position;
        ignition::math::Vector3d velocity;
        common::Time last_update_time;
    };

    physics::ModelPtr model;
    std::string modelName;
    event::ConnectionPtr updateConnection;
    std::unique_ptr<ros::NodeHandle> rosNode;
    
    // 发布器 - 只发布方向信息
    ros::Publisher directionPub;
    
    // 订阅器管理
    std::map<std::string, ros::Subscriber> otherUavSubscriptions;
    std::map<std::string, UavData> otherUavData; // 改为存储完整数据
    std::mutex dataMutex;
    
    // 定时器
    ros::Timer discoveryTimer;
    
    common::Time lastUpdateTime;
    int updateCount;
    int discoveryCount;
};

GZ_REGISTER_MODEL_PLUGIN(InfraredReceiverPlugin)
}