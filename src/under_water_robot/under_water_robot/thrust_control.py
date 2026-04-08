import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64,Float64MultiArray
from rov_interfaces.srv import SetForce
from nav_msgs.msg import Odometry






class ForceClient(Node):
    def __init__(self):
        # x y z 欧拉角 线速度 角速度 共12维状态量
        self.state = [0]*12
        super().__init__('thrust_control')
        # 参数：指定推进器编号与推力值8*1
        self.declare_parameter("force_array", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.client = self.create_client(SetForce, 'set_desired_force')
        # 阻塞等待服务上线
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('正在等待 C++ 服务端 "set_desired_force" 上线...')
        # 【修改 4】：服务一旦上线，立即调用函数发送期望力
        self.send_force_request()
        # 直接在这个现有的类里创建一个订阅者
        # 并将回调函数指向下面写好的 self.get_state
        self.subscription = self.create_subscription(
            Odometry,
            '/robot/odom',
            self.get_state,  # C语言视角：这里相当于传了一个函数指针
            10)


    def send_force_request(self):
        # 1. 获取参数列表中配置的力学数组
        force_values = self.get_parameter("force_array").value
        
        if len(force_values) != 6:
            self.get_logger().error(f"期待 6 个 force 数值，但收到 {len(force_values)} 个！")
            return
            
        # 2. 构造服务请求格式
        req = SetForce.Request()
        req.force = force_values  # 将 6 维数组填入 request 中
        
        # 3. 异步发送给 C++ 节点
        self.get_logger().info(f"发送力/力矩请求: {force_values}")
        self.future = self.client.call_async(req)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        # 4. 接收 C++ 节点处理完毕后返回的信息
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('C++ 节点成功接收并设置了新的推力！')
            else:
                self.get_logger().warn('C++ 节点处理失败。')
        except Exception as e:
            self.get_logger().error(f'服务调用异常: {e}')


    def get_state(self, msg):
        # 1. 提取位置 (Position)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        # 1. 提取姿态 (Orientation)
        orientation_x = msg.pose.pose.orientation.x
        orientation_y = msg.pose.pose.orientation.y
        orientation_z = msg.pose.pose.orientation.z
        orientation_w = msg.pose.pose.orientation.w
        # 2. 提取线速度 (Linear Velocity)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        # 3. 提取角速度 (Angular Velocity)
        p = msg.twist.twist.angular.x
        q = msg.twist.twist.angular.y
        r = msg.twist.twist.angular.z

        # 5. 可选：如果 C++ 节点有返回当前状态的服务，可以在这里调用获取并打印
        pass  # 这里暂时不实现状态获取功能
def main():
    rclpy.init()
    node = ForceClient()
    try:
        # 只在有回调时触发（发送完请求就可以等待结束了）
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()








