import rclpy
from rclpy.node import Node
from rov_interfaces.srv import SetForce
from nav_msgs.msg import Odometry
import math
def euler_from_quaternion(x, y, z, w):
	"""
	将四元数转换为欧拉角 (roll, pitch, yaw)
	"""
	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + y * y)
	roll_x = math.atan2(t0, t1)

	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	pitch_y = math.asin(t2)

	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (y * y + z * z)
	yaw_z = math.atan2(t3, t4)

	return roll_x, pitch_y, yaw_z


class DepthEsoControl(Node):
	"""
	控制/观测模板：
	- 控制输出仅作用于 x/y 和姿态 (roll/pitch/yaw)
	- 深度 (z) 轴输出固定为 0，留作 ESO 输入/观测验证
	"""

	def __init__(self):
		super().__init__('depth_eso_control')

		# x y z 欧拉角 线速度uvw 角速度pqr — 共12维状态量
		self.state = [0.0] * 12

		# 预留：深度输入与观测量（供 ESO 使用）
		self.depth_input = 0.0
		self.depth_observation = 0.0
		self.depth_velocity = 0.0

		# PID 状态初始化
		self._integral_error = [0.0] * 6
		self._pid_last_time = None

		# 目标状态（6维: x,y,z,roll,pitch,yaw），z目标保留但不参与输出
		self.declare_parameter("target_state", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

		self.client = self.create_client(SetForce, 'set_desired_force')
		while not self.client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('正在等待 C++ 服务端 "set_desired_force" 上线...')

		self.subscription = self.create_subscription(
			Odometry,
			'/robot/pose',
			self.get_state,
			10)

		self.timer = self.create_timer(0.05, self.time_callback)

	def time_callback(self):
		target_state_6d = self.get_parameter("target_state").value
		target_state = list(target_state_6d)
		output_force = self.pid_control(target_state)

		req = SetForce.Request()
		req.force = output_force
		self.client.call_async(req)

	def get_state(self, msg: Odometry):
		x = msg.pose.pose.position.x
		y = msg.pose.pose.position.y
		z = msg.pose.pose.position.z

		orientation_x = msg.pose.pose.orientation.x
		orientation_y = msg.pose.pose.orientation.y
		orientation_z = msg.pose.pose.orientation.z
		orientation_w = msg.pose.pose.orientation.w

		vx = msg.twist.twist.linear.x
		vy = msg.twist.twist.linear.y
		vz = msg.twist.twist.linear.z

		p = msg.twist.twist.angular.x
		q = msg.twist.twist.angular.y
		r = msg.twist.twist.angular.z

		roll, pitch, yaw = euler_from_quaternion(
			orientation_x, orientation_y, orientation_z, orientation_w)

		self.state = [x, y, z, roll, pitch, yaw, vx, vy, vz, p, q, r]

		# 预留：深度观测量
		self.depth_observation = z
		self.depth_velocity = vz

	def pid_control(self, target_state):
		now = self.get_clock().now().nanoseconds / 1e9
		if self._pid_last_time is None:
			self._pid_last_time = now
			return [0.0] * 6

		dt = now - self._pid_last_time
		if dt <= 0.0:
			return [0.0] * 6
		self._pid_last_time = now

		# 位置/姿态误差（世界坐标系）
		error = [0.0] * 6
		for i in range(6):
			error[i] = target_state[i] - self.state[i]

		# 姿态误差归一化到 [-pi, pi]
		error[3] = math.atan2(math.sin(error[3]), math.cos(error[3]))
		error[4] = math.atan2(math.sin(error[4]), math.cos(error[4]))
		error[5] = math.atan2(math.sin(error[5]), math.cos(error[5]))

		# PID 参数（z轴全部置0，确保不施加深度力）
		Kp = [100.0, 100.0, 0.0, 15.0, 15.0, 12.0]
		Ki = [1.0, 1.0, 0.0, 0.3, 0.3, 0.2]
		Kd = [20.0, 20.0, 0.0, 2.0, 2.0, 2.0]

		# 更新积分项（z轴不积分）
		for i in range(6):
			if i == 2:
				self._integral_error[i] = 0.0
				continue
			self._integral_error[i] += error[i] * dt
			limit = 50.0 if i < 3 else 20.0
			self._integral_error[i] = max(-limit, min(limit, self._integral_error[i]))

		# 计算 P+I 输出（世界坐标系）
		output_world = [0.0] * 6
		for i in range(6):
			output_world[i] = Kp[i] * error[i] + Ki[i] * self._integral_error[i]

		# 转换到机体坐标系
		force_body, torque_body = self.world_to_body_force(
			output_world[:3], output_world[3:])

		# 叠加速度阻尼（机体坐标系）
		for i in range(3):
			force_body[i] -= Kd[i] * self.state[6 + i]
			torque_body[i] -= Kd[i + 3] * self.state[9 + i]

		# 强制 z 方向输出为 0
		force_body[2] = 0.0

		return force_body + torque_body

	def world_to_body_force(self, force_world, torque_world):
		roll, pitch, yaw = self.state[3], self.state[4], self.state[5]

		cr = math.cos(roll)
		sr = math.sin(roll)
		cp = math.cos(pitch)
		sp = math.sin(pitch)
		cy = math.cos(yaw)
		sy = math.sin(yaw)

		R = [
			[cy * cp, sy * cp, -sp],
			[cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, cp * sr],
			[cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr]
		]

		force_body = [
			R[0][0] * force_world[0] + R[0][1] * force_world[1] + R[0][2] * force_world[2],
			R[1][0] * force_world[0] + R[1][1] * force_world[1] + R[1][2] * force_world[2],
			R[2][0] * force_world[0] + R[2][1] * force_world[1] + R[2][2] * force_world[2]
		]

		torque_body = [
			R[0][0] * torque_world[0] + R[0][1] * torque_world[1] + R[0][2] * torque_world[2],
			R[1][0] * torque_world[0] + R[1][1] * torque_world[1] + R[1][2] * torque_world[2],
			R[2][0] * torque_world[0] + R[2][1] * torque_world[1] + R[2][2] * torque_world[2]
		]

		return force_body, torque_body


def main():
	rclpy.init()
	node = DepthEsoControl()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()






