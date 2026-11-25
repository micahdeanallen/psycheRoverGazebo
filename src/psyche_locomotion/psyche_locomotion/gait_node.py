"""Gait node: /cmd_vel in, 18 joint commands out.

This is the link between the nav stack and the legs:
    nav stack -> /cmd_vel (geometry_msgs/Twist) -> gait_node
              -> /leg_position_controller/commands (18 angles) -> ros2_control -> servos

    ros2 run psyche_locomotion gait_node --ros-args -p use_sim_time:=true

Uses linear.x (forward), linear.y (sideways) and angular.z (turn). If no
/cmd_vel arrives for `cmd_timeout` seconds, the rover steps to a stop.
On startup it eases from wherever the legs are into the standing pose.
Don't run stand_node at the same time: both publish to the same controller.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from .gait import TripodGait
from .kinematics import JOINT_ORDER


class GaitNode(Node):

    def __init__(self):
        super().__init__('gait_node')
        p = self.declare_parameter
        p('rate', 50.0)
        p('period', 1.0)          # s per full gait cycle
        p('step_height', 0.03)    # m
        p('max_vx', 0.10)         # m/s
        p('max_vy', 0.05)         # m/s
        p('max_wz', 0.40)         # rad/s
        p('cmd_timeout', 0.5)     # s
        p('startup_time', 2.0)    # s to ease into the standing pose
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.gait = TripodGait(period=g('period'), step_height=g('step_height'),
                               max_vx=g('max_vx'), max_vy=g('max_vy'), max_wz=g('max_wz'))
        self.cmd_timeout = g('cmd_timeout')
        self.startup_time = g('startup_time')

        self.cmd = (0.0, 0.0, 0.0)
        self.last_cmd_time = None
        self.last_tick = None
        self.start_angles = None
        self.start_time = None
        self.warned_unreachable = False

        self.pub = self.create_publisher(Float64MultiArray, '/leg_position_controller/commands', 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)
        self.create_timer(1.0 / g('rate'), self.on_timer)
        self.get_logger().info('Waiting for /joint_states ...')

    def on_cmd_vel(self, msg):
        self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def on_joint_states(self, msg):
        if self.start_angles is not None:
            return
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in JOINT_ORDER):
            self.start_angles = [pos[j] for j in JOINT_ORDER]
            self.start_time = self.get_clock().now()
            self.get_logger().info('Easing into standing pose, then listening on /cmd_vel')

    def on_timer(self):
        if self.start_angles is None:
            return
        now = self.get_clock().now()
        dt = 0.0 if self.last_tick is None else (now - self.last_tick).nanoseconds * 1e-9
        self.last_tick = now
        dt = max(0.0, min(dt, 0.1))

        since_start = (now - self.start_time).nanoseconds * 1e-9
        if since_start < self.startup_time:
            # Blend from the current pose into the gait's standing pose.
            s = since_start / self.startup_time
            target = self.gait.joint_angles()
            angles = [a + s * (b - a) for a, b in zip(self.start_angles, target)]
        else:
            cmd = self.cmd
            if self.last_cmd_time is None or \
                    (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout:
                cmd = (0.0, 0.0, 0.0)
            angles = self.gait.update(dt, *cmd)
            if self.gait.unreachable and not self.warned_unreachable:
                self.get_logger().warn('A foot target was out of reach and got clamped. '
                                       'Lower max speeds or step_height.')
                self.warned_unreachable = True

        msg = Float64MultiArray()
        msg.data = list(angles)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = GaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
