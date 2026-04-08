import rclpy
from rclpy.node import Node
# 必须从 nav_msgs 导入，std_msgs 里没有这个
from nav_msgs.msg import Odometry

class State_Getter(Node):
    def __init__(self):
        super().__init__('state_getter')
        # 订阅 C++ 节点发布的 "/robot/odom"
        self.subscription = self.create_subscription(
            Odometry,
            '/robot/odom',
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
            f'姿态 (四元数): [{orientation_x:.3f}, {orientation_y:.3f}, {orientation_z:.3f}, {orientation_w:.3f}]\n'
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