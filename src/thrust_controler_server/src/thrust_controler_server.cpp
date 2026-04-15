#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include "ThrustAllocationSolver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

// 1. 引入你自定义的服务头文件 (注意命名规范：大写驼峰转小写下划线)
#include "rov_interfaces/srv/set_force.hpp"

using namespace std::chrono_literals;

class ThrusterAllocatorSystem
{
  // ...(此类与你原来一模一样，保持不变)
private:
  ThrustAllocationSolver allocator;
  bool is_initialized = false;
  double max_thrust_per_motor = 500.0;

public:
  ThrusterAllocatorSystem()
  {
    allocator.addThruster(Eigen::Vector3d(0.17108, 0.18031, 0.0), Eigen::Vector3d(0.0, 0.0, 1.0), max_thrust_per_motor);
    allocator.addThruster(Eigen::Vector3d(0.17108, -0.18031, 0.0), Eigen::Vector3d(0.0, 0.0, 1.0), max_thrust_per_motor);
    allocator.addThruster(Eigen::Vector3d(-0.18955, -0.18031, 0.0), Eigen::Vector3d(0.0, 0.0, 1.0), max_thrust_per_motor);
    allocator.addThruster(Eigen::Vector3d(-0.18955, 0.18031, 0.0), Eigen::Vector3d(0.0, 0.0, 1.0), max_thrust_per_motor);

    allocator.addThruster(Eigen::Vector3d(0.25239, -0.26163, 0.0), Eigen::Vector3d(-0.70711, -0.70711, 0.0), max_thrust_per_motor);
    allocator.addThruster(Eigen::Vector3d(0.25239, 0.26163, 0.0), Eigen::Vector3d(-0.70711, 0.70711, 0.0), max_thrust_per_motor);
    allocator.addThruster(Eigen::Vector3d(-0.27087, -0.26163, 0.0), Eigen::Vector3d(0.70711, -0.70711, 0.0), max_thrust_per_motor);
    allocator.addThruster(Eigen::Vector3d(-0.27087, 0.26163, 0.0), Eigen::Vector3d(0.70711, 0.70711, 0.0), max_thrust_per_motor);

    is_initialized = allocator.initialize();
    if (!is_initialized)
    {
      std::cerr << "推力分配求解器初始化失败！" << std::endl;
    }
  }

  Eigen::VectorXd computeMotorThrusts(const Eigen::VectorXd &T_desired)
  {
    if (!is_initialized)
      return Eigen::VectorXd::Zero(8);

    Eigen::VectorXd k_solution;
    if (allocator.solve(T_desired, k_solution))
    {
      return k_solution * max_thrust_per_motor;
    }
    else
    {
      std::cerr << "QP 求解失败，输出零推力防暴走。" << std::endl;
      return Eigen::VectorXd::Zero(8);
    }
  }
};

class ThrustControl : public rclcpp::Node
{
public:
  ThrustControl()
      : Node("thrust_controler_server")
  {
    // 1. 初始化发布者
    publishers_list_.reserve(8);
    for (int i = 1; i <= 8; ++i)
    {
      std::string topic = "/model/UnderWater_Robot/joint/fan" + std::to_string(i) + "/cmd_thrust";
      auto pub = this->create_publisher<std_msgs::msg::Float64>(topic, 10);
      publishers_list_.push_back(pub);
    }

    Force_desired_ = Eigen::VectorXd::Zero(6);
    thrust_values_ = Eigen::VectorXd::Zero(8);

    // 2. 初始分配器与定时器
    allocator_system_ = ThrusterAllocatorSystem();
    timer_ = this->create_wall_timer(50ms, std::bind(&ThrustControl::time_callback, this));

    // 3. 【新增】：创建 ROS 2 服务端 (Service Server)
    // 名称为 "set_desired_force"，绑定到 handle_set_force_service 函数
    using std::placeholders::_1;
    using std::placeholders::_2;
    service_ = this->create_service<rov_interfaces::srv::SetForce>(
        "set_desired_force",
        std::bind(&ThrustControl::handle_set_force_service, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "推力控制节点已启动，等待服务调用...");
  }

  bool Set_Desired_Force(const Eigen::VectorXd &desired_force)
  {
    if (desired_force.size() != 6)
    {
      RCLCPP_ERROR(this->get_logger(), "期望的合力/合力矩必须是 6 维向量！");
      return false;
    }
    Force_desired_ = desired_force;
    thrust_values_ = allocator_system_.computeMotorThrusts(Force_desired_);
    return true;
  }

private:
  // 【新增】：服务的回调函数
  // 当终端或其它节点发送服务请求时，会触发这个函数
  void handle_set_force_service(
      const std::shared_ptr<rov_interfaces::srv::SetForce::Request> request,
      std::shared_ptr<rov_interfaces::srv::SetForce::Response> response)
  {
    // 将 srv 请求中的 float64[6] 数组转换给 Eigen::VectorXd
    Eigen::VectorXd new_force(6);
    for (int i = 0; i < 6; ++i)
    {
      new_force(i) = request->force[i];
    }

    // 调用现有的函数设置推力，并将结果返回给服务请求者
    response->success = this->Set_Desired_Force(new_force);

    RCLCPP_INFO(this->get_logger(),
                "接收到新的推力请求: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                new_force(0), new_force(1), new_force(2),
                new_force(3), new_force(4), new_force(5));
    time_count_ = 0; // 重置计数器，立即响应新的推力请求
  }

  void time_callback()
  {
    if (thrust_values_.size() != 8 || time_count_ >10 ){
      time_count_ = 0;
      thrust_values_ = Eigen::VectorXd::Zero(8);
    
    }
    for (size_t i = 0; i < 8; ++i)
    {
      std_msgs::msg::Float64 msg;
      msg.data = static_cast<double>(thrust_values_[i]);
      publishers_list_[i]->publish(msg);
    }
    time_count_ += 1;

  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> publishers_list_;

  // 【新增】：保存服务对象的智能指针
  rclcpp::Service<rov_interfaces::srv::SetForce>::SharedPtr service_;

  ThrusterAllocatorSystem allocator_system_;
  Eigen::VectorXd Force_desired_ = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd thrust_values_ = Eigen::VectorXd::Zero(8);
  int time_count_ = 0;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ThrustControl>();

  // 【删除】：我们去掉了这里的硬编码配置，现在默认是 0
  // node->Set_Desired_Force((Eigen::VectorXd(6) << 0.0, 10.0, 0.0, 0.0, 0.0, 0.0).finished());

  std::cout << "准备进入 ROS 2 循环..." << std::endl;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}