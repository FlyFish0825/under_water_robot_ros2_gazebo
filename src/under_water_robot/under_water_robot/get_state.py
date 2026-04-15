import rclpy
from rclpy.node import Node
# 必须从 nav_msgs 导入，std_msgs 里没有这个
from nav_msgs.msg import Odometry
import math


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


class State_Getter(Node):
    def __init__(self):
        super().__init__('state_getter')
        # 订阅 C++ 节点发布的 "/robot/odom"
        self.subscription = self.create_subscription(
            Odometry,
            '/robot/pose',  # 注意这里要和 C++ 节点发布的 topic 名称一致
            self.odom_callback,
            10)
        self.get_logger().info('Python state 订阅节点已启动...')

    def odom_callback(self, msg):
        # 1. 提取位置 (Position)
        # 对应你命令行输出的 pose -> pose -> position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        orientation_x = msg.pose.pose.orientation.x
        orientation_y = msg.pose.pose.orientation.y
        orientation_z = msg.pose.pose.orientation.z
        orientation_w = msg.pose.pose.orientation.w

        roll, pitch, yaw = euler_from_quaternion(orientation_x, orientation_y, orientation_z, orientation_w)

        # 4. 转换为角度 (Degree) 方便观察
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)

        # 2. 提取线速度 (Linear Velocity)
        # 对应你命令行输出的 twist -> twist -> linear
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z

        p = msg.twist.twist.angular.x
        q = msg.twist.twist.angular.y
        r = msg.twist.twist.angular.z
        # 打印结果，:.3f 表示保留三位小数
        self.get_logger().info(
            f'\n位置: [{x:.3f}, {y:.3f}, {z:.3f}]\n'
            f'欧拉角(度): [Roll:{roll_deg:.2f}, Pitch:{pitch_deg:.2f}, Yaw:{yaw_deg:.2f}]'
            f'欧拉角(弧度): [Roll:{roll:.3f}, Pitch:{pitch:.3f}, Yaw:{yaw:.3f}]'
            f'速度: [{vx:.3f}, {vy:.3f}, {vz:.3f}]\n'
            f'角速度: [{p:.3f}, {q:.3f}, {r:.3f}]'
        )

def main(args=None):
    rclpy.init(args=args)
    node = State_Getter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()