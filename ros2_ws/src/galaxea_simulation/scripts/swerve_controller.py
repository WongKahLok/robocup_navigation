#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class SwerveController(Node):

    MODULES = [
        ('steer_motor_joint1', 'wheel_motor_joint1', 0.21516,  0.28),
        ('steer_motor_joint2', 'wheel_motor_joint2', 0.21516, -0.28),
        ('steer_motor_joint3', 'wheel_motor_joint3', -0.28084, 0.0),
    ]

    def __init__(self):
        super().__init__('swerve_controller')

        self.declare_parameter('wheel_radius', 0.025)
        self.declare_parameter('deadband', 0.001)
        self.declare_parameter('rate', 50.0)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.deadband = self.get_parameter('deadband').value
        rate = self.get_parameter('rate').value

        self.cmd = Twist()
        self.current_steer = [0.0] * len(self.MODULES)

        self.steer_pubs = []
        self.wheel_pubs = []
        for steer_jnt, wheel_jnt, _, _ in self.MODULES:
            self.steer_pubs.append(
                self.create_publisher(
                    Float64,
                    f'/model/r1/joint/{steer_jnt}/0/cmd_pos', 10))
            self.wheel_pubs.append(
                self.create_publisher(
                    Float64,
                    f'/model/r1/joint/{wheel_jnt}/cmd_vel', 10))

        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(
            JointState, '/joint_states', self._joint_states_cb, 10)
        self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info(
            f'Swerve controller ready (wheel_r={self.wheel_radius})')

    def _cmd_vel_cb(self, msg):
        self.cmd = msg

    def _joint_states_cb(self, msg):
        for i, (steer_jnt, _, _, _) in enumerate(self.MODULES):
            if steer_jnt in msg.name:
                self.current_steer[i] = msg.position[
                    msg.name.index(steer_jnt)]

    @staticmethod
    def _normalize(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def _control_loop(self):
        vx = self.cmd.linear.x
        vy = self.cmd.linear.y
        wz = self.cmd.angular.z

        for i, (_, _, lx, ly) in enumerate(self.MODULES):
            mx = vx - wz * ly
            my = vy + wz * lx
            speed = math.hypot(mx, my)

            if speed < self.deadband:
                angle = self.current_steer[i]
                wheel_vel = 0.0
            else:
                angle = math.atan2(my, mx)
                wheel_vel = speed / self.wheel_radius

                delta = self._normalize(angle - self.current_steer[i])
                if abs(delta) > math.pi / 2.0:
                    angle = self._normalize(angle + math.pi)
                    wheel_vel = -wheel_vel

                angle = max(-math.pi / 2.0, min(math.pi / 2.0, angle))

            s = Float64()
            s.data = angle
            self.steer_pubs[i].publish(s)

            w = Float64()
            w.data = wheel_vel
            self.wheel_pubs[i].publish(w)


def main(args=None):
    rclpy.init(args=args)
    node = SwerveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
