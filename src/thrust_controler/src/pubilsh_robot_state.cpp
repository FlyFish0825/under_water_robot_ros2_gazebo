#include <iostream>
#include <string>

// Ignition 头文件
#include <ignition/transport/Node.hh>
#include <ignition/msgs/pose_v.pb.h>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Matrix4.hh>

// ROS 2 头文件
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp> // 【新增】标准的里程计消息类型，包含位姿和速度

class StateBridgeNode : public rclcpp::Node
{
public:
    StateBridgeNode() : Node("state_bridge_node"), last_time_(-1.0)
    {
        // 1. 初始化 ROS 2 发布者
        // 发布到 "/robot/odom" 话题，队列长度设为 10
        std::string ROS2_topic = "/robot/odom";
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(ROS2_topic, 10);

        // 2. 初始化 Ignition 订阅者
        std::string topic = "/model/UnderWater_Robot/pose";
        RCLCPP_INFO(this->get_logger(), "正在尝试订阅 Ignition 话题: %s ...", topic.c_str());

        // 【语法注意】在类中注册 Ignition 回调函数，需要传入对象指针 `this`
        if (ign_node_.Subscribe(topic, &StateBridgeNode::OnPoseReceived, this))
        {
            RCLCPP_INFO(this->get_logger(), "✅ Ignition 订阅成功！开始向 ROS 2 转发数据...");
            RCLCPP_INFO(this->get_logger(), "Ignition 话题: %s", topic.c_str());
            RCLCPP_INFO(this->get_logger(), "ROS2 话题: %s", topic.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Ignition 订阅失败！");
        }
    }

private:
    // 【修改】把原来的普通函数变成了类的成员函数
    void OnPoseReceived(const ignition::msgs::Pose_V &_msg)
    {
        for (int i = 0; i < _msg.pose_size(); ++i)
        {
            const auto &pose_msg = _msg.pose(i);

            std::string frame_id = "";
            for (int j = 0; j < pose_msg.header().data_size(); ++j)
            {
                if (pose_msg.header().data(j).key() == "frame_id" && pose_msg.header().data(j).value_size() > 0)
                {
                    frame_id = pose_msg.header().data(j).value(0);
                    break;
                }
            }

            if (frame_id == "world")
            {
                ignition::math::Pose3d current_world_pose = ignition::msgs::Convert(pose_msg);
                double current_time = pose_msg.header().stamp().sec() + pose_msg.header().stamp().nsec() * 1e-9;

                // 如果不是第一次接收数据，计算速度并发布 ROS 2 消息
                if (last_time_ > 0.0)
                {
                    double dt = current_time - last_time_;

                    if (dt > 0.0)
                    {
                        // 计算世界系速度
                        ignition::math::Vector3d V_world = (current_world_pose.Pos() - last_world_pose_.Pos()) / dt;
                        // 1. 分别计算 Roll(X), Pitch(Y), Yaw(Z) 的角度差
                        double roll_diff = current_world_pose.Rot().Euler().X() - last_world_pose_.Rot().Euler().X();
                        double pitch_diff = current_world_pose.Rot().Euler().Y() - last_world_pose_.Rot().Euler().Y();
                        double yaw_diff = current_world_pose.Rot().Euler().Z() - last_world_pose_.Rot().Euler().Z();

                        // 2. 加入你的代码：处理 Z 轴（Yaw角）的跳变
                        if (yaw_diff > M_PI)
                            yaw_diff -= 2.0 * M_PI;
                        if (yaw_diff < -M_PI)
                            yaw_diff += 2.0 * M_PI;

                        // 3. 将处理后的差值重新组合成 Vector3d 向量
                        ignition::math::Vector3d Euler_diff(roll_diff, pitch_diff, yaw_diff);
                        ignition::math::Vector3d Omega_world = Euler_diff / dt;

                        // 转换到机体系
                        ignition::math::Quaterniond R_world_to_body = current_world_pose.Rot().Inverse();
                        ignition::math::Vector3d V_body = R_world_to_body * V_world;
                        ignition::math::Vector3d Omega_body = R_world_to_body * Omega_world;

                        // ==========================================
                        // C. 【新增】填充并发布 ROS 2 Odometry 消息
                        // ==========================================
                        nav_msgs::msg::Odometry odom_msg;

                        // 1. 填充时间戳与坐标系信息
                        odom_msg.header.stamp = this->get_clock()->now();
                        odom_msg.header.frame_id = "world";    // 参考坐标系 (位姿所在的坐标系)
                        odom_msg.child_frame_id = "base_link"; // 子坐标系 (速度所在的坐标系，即机体)

                        // 2. 填充位置 (Position)
                        odom_msg.pose.pose.position.x = current_world_pose.Pos().X();
                        odom_msg.pose.pose.position.y = current_world_pose.Pos().Y();
                        odom_msg.pose.pose.position.z = current_world_pose.Pos().Z();

                        // 3. 填充姿态 (Orientation，四元数)
                        odom_msg.pose.pose.orientation.w = current_world_pose.Rot().W();
                        odom_msg.pose.pose.orientation.x = current_world_pose.Rot().X();
                        odom_msg.pose.pose.orientation.y = current_world_pose.Rot().Y();
                        odom_msg.pose.pose.orientation.z = current_world_pose.Rot().Z();

                        // 4. 填充线速度 (Linear Velocity，机体系)
                        odom_msg.twist.twist.linear.x = V_body.X();
                        odom_msg.twist.twist.linear.y = V_body.Y();
                        odom_msg.twist.twist.linear.z = V_body.Z();

                        // 5. 填充角速度 (Angular Velocity，机体系)
                        odom_msg.twist.twist.angular.x = Omega_body.X();
                        odom_msg.twist.twist.angular.y = Omega_body.Y();
                        odom_msg.twist.twist.angular.z = Omega_body.Z();

                        // 6. 发布消息！
                        odom_pub_->publish(odom_msg);
                    }
                }

                // 更新历史记录 (用类的成员变量替代了 C 语言的 static 变量)
                last_world_pose_ = current_world_pose;
                last_time_ = current_time;

                break;
            }
        }
    }

    // 类的私有成员变量
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    ignition::transport::Node ign_node_;

    ignition::math::Pose3d last_world_pose_;
    double last_time_;
};

int main(int argc, char **argv)
{
    // 1. 初始化 ROS 2
    rclcpp::init(argc, argv);

    // 2. 创建节点实例 (智能指针)
    auto node = std::make_shared<StateBridgeNode>();

    // 3. 运行 ROS 2 事件循环 (Ignition 内部自带线程处理它的回调，因此不冲突)
    rclcpp::spin(node);

    // 4. 退出清理
    rclcpp::shutdown();
    return 0;
}