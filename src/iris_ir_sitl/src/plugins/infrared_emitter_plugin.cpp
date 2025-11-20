#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <mutex>

namespace gazebo
{
// 全局标志，表示ROS是否已初始化
static bool g_ros_initialized = false;
static std::mutex g_ros_mutex;

class InfraredEmitterPlugin : public ModelPlugin
{
public:
    InfraredEmitterPlugin() : model(nullptr), updateCount(0) {}

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        // 存储模型指针
        this->model = _model;
        if (!this->model) {
            gzerr << "Received NULL model pointer!" << std::endl;
            return;
        }

        this->modelName = this->model->GetName();
        gzmsg << "🚀 === InfraredEmitterPlugin Loading for: " << this->modelName << " ===" << std::endl;

        // 安全初始化ROS
        if (!this->initializeROS()) {
            gzerr << "❌ Failed to initialize ROS for " << this->modelName << std::endl;
            return;
        }

        // 连接更新事件
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&InfraredEmitterPlugin::OnUpdate, this, std::placeholders::_1));

        gzmsg << "🎉 === InfraredEmitterPlugin SUCCESS for: " << this->modelName << " ===" << std::endl;
    }

private:
    bool initializeROS()  //ROS节点初始化
    {
        std::lock_guard<std::mutex> lock(g_ros_mutex);
        
        // 如果ROS已经初始化，直接创建节点
        if (g_ros_initialized) {
            return this->createROSNode();
        }
        
        // 第一次初始化ROS
        std::string node_name = "infrared_emitter_master";
        gzmsg << "Initializing ROS for the first time with node: " << node_name << std::endl;
        
        int argc = 0;
        char **argv = NULL;
        
        // 使用NoSigintHandler避免信号处理冲突
        ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
        
        if (!ros::master::check()) {
            gzerr << "ROS master is not running!" << std::endl;
            return false;
        }
        
        g_ros_initialized = true;
        gzmsg << "✅ ROS initialized successfully" << std::endl;
        
        return this->createROSNode();
    }
    
    bool createROSNode()  // 创建节点句柄  创建publisher
    {
        try {
            // 创建节点句柄
            this->rosNode.reset(new ros::NodeHandle());
            
            // 创建发布器
            std::string topic_name = "/uav/infrared/emitter/" + this->modelName;
            this->signalPub = this->rosNode->advertise<geometry_msgs::Vector3>(topic_name, 10);
            
            gzmsg << "✅ Created publisher for " << this->modelName << " on topic: " << topic_name << std::endl;
            
            // 发布测试消息
            this->publishTestMessage();
            
            return true;
            
        } catch (const std::exception& e) {
            gzerr << "❌ Failed to create ROS node: " << e.what() << std::endl;
            return false;
        }
    }

    void publishTestMessage()  //新建测试消息
    {
        geometry_msgs::Vector3 test_msg;
        if (this->modelName.find("iris_ir_1") != std::string::npos) {
            test_msg.x = 111.0; test_msg.y = 111.0; test_msg.z = 111.0;
        } else if (this->modelName.find("iris_ir_2") != std::string::npos) {
            test_msg.x = 222.0; test_msg.y = 222.0; test_msg.z = 222.0;
        } else if (this->modelName.find("iris_ir_3") != std::string::npos) {
            test_msg.x = 333.0; test_msg.y = 333.0; test_msg.z = 333.0;
        } else {
            test_msg.x = 999.0; test_msg.y = 999.0; test_msg.z = 999.0;
        }
        
        this->signalPub.publish(test_msg);
        gzmsg << "🧪 " << this->modelName << " TEST: (" 
              << test_msg.x << ", " << test_msg.y << ", " << test_msg.z << ")" << std::endl;
    }

    void OnUpdate(const common::UpdateInfo &_info)  //发布消息
    {
        // 安全检查
        if (!this->model || !this->rosNode || !this->signalPub) {
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

        // 获取无人机位置
        ignition::math::Pose3d pose = this->model->WorldPose();
        ignition::math::Vector3d pos = pose.Pos();

        // 计算速度（基于位置变化）
        ignition::math::Vector3d velocity(0, 0, 0);
        if (this->lastPosition != ignition::math::Vector3d::Zero) {
            double dt = (_info.simTime - this->lastUpdateTime).Double();
            if (dt > 0) {
                velocity = (pos - this->lastPosition) / dt;
            }
        }
        this->lastPosition = pos;

        // 发布位置信息（现在包含位置信息，速度信息在接收器中计算）
        geometry_msgs::Vector3 signalMsg;
        signalMsg.x = pos.X();
        signalMsg.y = pos.Y();
        signalMsg.z = pos.Z();

        this->signalPub.publish(signalMsg);

        // 减少日志频率避免过载
        if (this->updateCount % 20 == 0) {
            gzmsg << "📤 [" << this->modelName << "] at (" 
                  << std::fixed << std::setprecision(1) 
                  << pos.X() << ", " << pos.Y() << ", " << pos.Z() << ")"
                  << " velocity: (" << std::setprecision(2)
                  << velocity.X() << ", " << velocity.Y() << ", " << velocity.Z() << ")" << std::endl;
        }
        this->updateCount++;
    }

    physics::ModelPtr model;
    std::string modelName;
    event::ConnectionPtr updateConnection;
    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::Publisher signalPub;
    
    common::Time lastUpdateTime;
    ignition::math::Vector3d lastPosition;
    int updateCount;
};

GZ_REGISTER_MODEL_PLUGIN(InfraredEmitterPlugin)
}