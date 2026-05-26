import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64,Float64MultiArray
from rov_interfaces.srv import SetForce, SetState
from nav_msgs.msg import Odometry
import math
import time

# 辅助函数：四元数转欧拉角 (弧度)
def euler_from_quaternion(x, y, z, w):
    """
    将四元数转换为欧拉角 (roll, pitch, yaw)
    """
    # roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    # pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    # yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # 单位：弧度





class ForceClient(Node):
    def __init__(self):
        super().__init__('thrust_control')
        # x y z 欧拉角 线速度uvw 角速度pqr — 共12维状态量
        self.state = [0.0] * 12

        # PID 状态初始化
        self._integral_error = [0.0] * 6
        self._prev_error = [0.0] * 6       # 仅用于位置环的微分(备用)
        self._pid_last_time = None

        # 参数：期望推力（6维）和目标状态（6维: x,y,z,roll,pitch,yaw）
        self.declare_parameter("force_array", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("target_state", [0.0, 0.0, 2.5, 0.0, 0.0, 0.0])
        self.SetForce_client = self.create_client(SetForce, 'set_desired_force')
        # 阻塞等待服务上线
        while not self.SetForce_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('正在等待 C++ 服务端 "set_desired_force" 上线...')
        self.force_values = [0.0] * 6
        # 订阅里程计话题获取状态
        self.subscription = self.create_subscription(
            Odometry,
            '/robot/pose',
            self.get_state,
            10)

        self.time_ = self.create_timer(0.05, self.time_callback)  # 50ms 控制周期



    def time_callback(self):
        req = SetForce.Request()
        req.force = self.force_values  # 将 6 维数组填入 request 中
        target_state_6d = self.get_parameter("target_state").value   # 长度为6的列表
        target_state=list(target_state_6d )
        PID_output = self.PID_control(target_state)  # 这里暂时写死一个目标状态，后续你可以改成动态的
        req.force = PID_output  # 使用 PID 控制器的输出作为新的推力值
        self.force_values = req.force


        self.future = self.SetForce_client.call_async(req)
        self.future.add_done_callback(self.response_callback)
        
    def send_force_request(self):
        # 1. 获取参数列表中配置的力学数组
        self.force_values = self.get_parameter("force_array").value
        
        if len(self.force_values) != 6:
            self.get_logger().error(f"期待 6 个 force 数值，但收到 {len(self.force_values)} 个！")
            return
            
        # 2. 构造服务请求格式
        req = SetForce.Request()
        req.force = self.force_values  # 将 6 维数组填入 request 中
        
        # 3. 异步发送给 C++ 节点
        self.get_logger().info(f"发送力/力矩请求: {self.force_values}")
        self.future = self.SetForce_client.call_async(req)
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

        roll, pitch, yaw = euler_from_quaternion(orientation_x, orientation_y, orientation_z, orientation_w)
        self.state = [x, y, z, roll, pitch, yaw, vx, vy, vz, p, q, r]
        # 5. 可选：如果 C++ 节点有返回当前状态的服务，可以在这里调用获取并打印
        pass  # 这里暂时不实现状态获取功能
            
    def PID_control(self, target_state):
        """
        位置/姿态 PID 控制器（基于位置误差 + 速度阻尼）。
        - 比例 P：位置误差 × Kp（世界坐标系 → 转机体坐标系）
        - 积分 I：误差累积（带限幅），消除静差
        - 阻尼 D：用实测机体速度 × Kd，替代误差微分，避免目标突变尖峰
        """
        # ---------- 1. 时间间隔 ----------
        now = self.get_clock().now().nanoseconds / 1e9
        if self._pid_last_time is None:
            self._pid_last_time = now
            return [0.0] * 6

        dt = now - self._pid_last_time
        if dt <= 0:
            return [0.0] * 6
        self._pid_last_time = now

        # ---------- 2. 位置/姿态误差（世界坐标系） ----------
        error = [0.0] * 6
        for i in range(6):
            error[i] = target_state[i] - self.state[i]

        # 姿态误差归一化到 [-pi, pi]
        error[3] = math.atan2(math.sin(error[3]), math.cos(error[3]))
        error[4] = math.atan2(math.sin(error[4]), math.cos(error[4]))
        error[5] = math.atan2(math.sin(error[5]), math.cos(error[5]))

        # ---------- 3. PID 参数 ----------
        # 顺序: [Fx, Fy, Fz, Mx, My, Mz]
        Kp = [100.0, 100.0, 100.0,  15.0, 15.0, 12.0]   # 比例增益
        Ki = [1.0,   1.0,   1.05,   0.3,  0.3,  0.2]    # 积分增益
        Kd = [20.0,  20.0,  20.0,   2.0,  2.0,  2.0]    # 速度阻尼增益（作用于实测机体速度）

        # ---------- 4. 更新积分项（带限幅抗饱和） ----------
        for i in range(6):
            self._integral_error[i] += error[i] * dt
            limit = 50.0 if i < 3 else 20.0
            self._integral_error[i] = max(-limit, min(limit, self._integral_error[i]))

        # ---------- 5. 计算 P+I 输出（世界坐标系） ----------
        output_world = [0.0] * 6
        for i in range(6):
            output_world[i] = Kp[i] * error[i] + Ki[i] * self._integral_error[i]

        # ---------- 6. 转换 P+I 到机体坐标系 ----------
        force_body, torque_body = self.world_to_body_force(
            output_world[:3], output_world[3:])

        # ---------- 7. 叠加速度阻尼（机体坐标系，直接使用实测速度） ----------
        # self.state[6:9] = [u, v, w]  机体线速度
        # self.state[9:12] = [p, q, r] 机体角速度
        for i in range(3):
            force_body[i] -= Kd[i] * self.state[6 + i]
            torque_body[i] -= Kd[i + 3] * self.state[9 + i]

        # ---------- 8. 输出限幅 ----------
        force_limit = 500.0
        torque_limit = 100.0
        for i in range(3):
            force_body[i] = max(-force_limit, min(force_limit, force_body[i]))
            torque_body[i] = max(-torque_limit, min(torque_limit, torque_body[i]))

        output_force = force_body + torque_body
        print(f"PID 输出 (机体):  Fx:{force_body[0]:.1f}  Fy:{force_body[1]:.1f}  Fz:{force_body[2]:.1f}  "
              f"Mx:{torque_body[0]:.1f}  My:{torque_body[1]:.1f}  Mz:{torque_body[2]:.1f}")
        return output_force



    def world_to_body_force(self, force_world, torque_world):
        """
        将世界坐标系下的力和力矩转换到机体坐标系（基于当前姿态）。
        参数:
            force_world: [Fx, Fy, Fz] 世界坐标系下的力
            torque_world: [Mx, My, Mz] 世界坐标系下的力矩
        返回:
            force_body: [Fx_body, Fy_body, Fz_body]
            torque_body: [Mx_body, My_body, Mz_body]
        """
        # 获取当前欧拉角 (roll, pitch, yaw) 弧度
        roll, pitch, yaw = self.state[3], self.state[4], self.state[5]

        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        # 旋转矩阵：从世界坐标系到机体坐标系 (3x3)
        # 基于 ZYX 欧拉角顺序 (yaw -> pitch -> roll)
        R = [
            [cy*cp, sy*cp, -sp],
            [cy*sp*sr - sy*cr, sy*sp*sr + cy*cr, cp*sr],
            [cy*sp*cr + sy*sr, sy*sp*cr - cy*sr, cp*cr]
        ]

        # 变换力
        force_body = [
            R[0][0]*force_world[0] + R[0][1]*force_world[1] + R[0][2]*force_world[2],
            R[1][0]*force_world[0] + R[1][1]*force_world[1] + R[1][2]*force_world[2],
            R[2][0]*force_world[0] + R[2][1]*force_world[1] + R[2][2]*force_world[2]
        ]

        # 变换力矩（与力使用相同的旋转矩阵）
        torque_body = [
            R[0][0]*torque_world[0] + R[0][1]*torque_world[1] + R[0][2]*torque_world[2],
            R[1][0]*torque_world[0] + R[1][1]*torque_world[1] + R[1][2]*torque_world[2],
            R[2][0]*torque_world[0] + R[2][1]*torque_world[1] + R[2][2]*torque_world[2]
        ]

        return force_body, torque_body
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








