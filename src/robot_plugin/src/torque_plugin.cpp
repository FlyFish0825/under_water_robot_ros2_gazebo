#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/components/ExternalWorldWrenchCmd.hh>
#include <ignition/gazebo/components/JointAxis.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/msgs/Utility.hh>
#include <ignition/transport/Node.hh>
#include <mutex>

namespace under_robot {

class TorquePlugin : public ignition::gazebo::System,
                     public ignition::gazebo::ISystemConfigure,
                     public ignition::gazebo::ISystemPreUpdate 
{
public:
    void Configure(const ignition::gazebo::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   ignition::gazebo::EntityComponentManager &_ecm,
                   ignition::gazebo::EventManager &/*_eventMgr*/) override 
    {
        auto model = ignition::gazebo::Model(_entity);

        // 1. 读取受力 Link 名称（通用性改进）
        std::string linkName = _sdf->Get<std::string>("link_name", "base_link").first;
        this->linkEntity = model.LinkByName(_ecm, linkName);

        if (this->linkEntity == ignition::gazebo::kNullEntity) {
            ignerr << "无法找到指定的 Link: [" << linkName << "]" << std::endl;
            return;
        }

        // 2. 读取推进器 Joint 名称和系数
        if (!_sdf->HasElement("joint_name")) {
            ignerr << "插件缺少 <joint_name> 参数!" << std::endl;
            return;
        }
        std::string jointName = _sdf->Get<std::string>("joint_name");
        this->k_torque = _sdf->Get<double>("k_torque", 0.01).first;

        // 3. 获取 Joint 的旋转轴方向
        auto jointEntity = model.JointByName(_ecm, jointName);
        if (jointEntity != ignition::gazebo::kNullEntity) {
            auto axisComp = _ecm.Component<ignition::gazebo::components::JointAxis>(jointEntity);
            if (axisComp) {
                // 获取相对于模型坐标系的旋转轴单位向量
                this->unitRotationAxis = axisComp->Data().Xyz();
                ignmsg << "插件加载成功: [" << jointName << "] -> [" << linkName 
                       << "]，旋转轴: " << this->unitRotationAxis << std::endl;
            }
        }

        // 4. 订阅独立的推力话题
        std::string topic = "/model/" + model.Name(_ecm) + "/joint/" + jointName + "/cmd_thrust";
        if (!this->node.Subscribe(topic, &TorquePlugin::OnThrustMsg, this)) {
            ignerr << "订阅失败: " << topic << std::endl;
        }
    }

    void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                   ignition::gazebo::EntityComponentManager &_ecm) override 
    {
        if (_info.paused || this->linkEntity == ignition::gazebo::kNullEntity) return;

        double currentThrust = 0.0;
        {
            std::lock_guard<std::mutex> lock(this->mutex);
            currentThrust = this->latestThrust;
        }

        // 计算反作用力矩向量 (反向 = -1.0)
        // 力矩方向始终沿着 Joint 的物理旋转轴
        ignition::math::Vector3d torqueVec = this->unitRotationAxis * (currentThrust * this->k_torque * -1.0);

        using WrenchCmd = ignition::gazebo::components::ExternalWorldWrenchCmd;
        auto wrenchComp = _ecm.Component<WrenchCmd>(this->linkEntity);

        if (!wrenchComp) {
            ignition::msgs::Wrench msg;
            ignition::msgs::Set(msg.mutable_torque(), torqueVec);
            _ecm.CreateComponent(this->linkEntity, WrenchCmd(msg));
        } else {
            // 多个插件实例会在这里安全地累加力矩
            ignition::math::Vector3d currentTorque = ignition::msgs::Convert(wrenchComp->Data().torque());
            ignition::msgs::Set(wrenchComp->Data().mutable_torque(), currentTorque + torqueVec);
        }
    }

private:
    void OnThrustMsg(const ignition::msgs::Double &_msg) {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->latestThrust = _msg.data();
    }

    ignition::gazebo::Entity linkEntity{ignition::gazebo::kNullEntity};
    ignition::transport::Node node;
    ignition::math::Vector3d unitRotationAxis{0, 0, 1};
    double latestThrust{0.0};
    double k_torque{0.01};
    std::mutex mutex;
};

} // namespace under_robot

IGNITION_ADD_PLUGIN(under_robot::TorquePlugin, ignition::gazebo::System,
                    under_robot::TorquePlugin::ISystemConfigure,
                    under_robot::TorquePlugin::ISystemPreUpdate)