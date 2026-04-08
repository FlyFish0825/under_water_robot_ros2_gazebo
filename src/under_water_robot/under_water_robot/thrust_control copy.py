import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64,Float64MultiArray
from rov_interfaces.srv import SetForce
import math
#演示使用

class ForceClient(Node):
    def __init__(self):
        super().__init__('thrust_control')
        # 参数：指定推进器编号与推力值8*1
        
        
        #self.declare_parameter("force_array", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])



        # #设置推力参数，【0 ， 0 ，0 ，10 ，0 ，0 】代表M_x=10N，其他力矩和力为0，机体绕x轴旋转一定角度
        # self.declare_parameter("force_array", [0.0, 0.0, 0.0, 10.0, 0.0, 0.0])

        # #设置推力参数，【0 ， 0 ，0 ，0 ，10 ，0 】代表M_y=10N，其他力矩和力为0，机体绕y轴旋转一定角度
        # self.declare_parameter("force_array", [0.0, 0.0, 0.0, 0.0, 10.0, 0.0])

         #设置推力参数，【0 ， 0 ，0 ，0 ，0 ，10 】代表M_z=10N，其他力矩和力为0，机体绕z轴匀速旋转
        self.declare_parameter("force_array", [0.0, 0.0, 0.0, 0.0, 0.0, 10.0])
        self._t = 0.0
        self.create_timer(0.05, self.update_z_torque)

        self.client = self.create_client(SetForce, 'set_desired_force')
        # 阻塞等待服务上线
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('正在等待 C++ 服务端 "set_desired_force" 上线...')
        # 【修改 4】：服务一旦上线，立即调用函数发送期望力
        self.send_force_request()


    def update_z_torque(self):
        self._t +=0.05  # 每次调用增加时间步长
        z_torque = 100.0 * math.sin(2.0 * math.pi * 1.0 * self._t)
        force_values = [0.0, 0.0, 0.0, 0.0, 0.0, z_torque]
        self.set_parameters([Parameter("force_array", Parameter.Type.DOUBLE_ARRAY, force_values)])
        self.send_force_request()

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
                pass
            else:
                self.get_logger().warn('C++ 节点处理失败。')
        except Exception as e:
            self.get_logger().error(f'服务调用异常: {e}')
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








