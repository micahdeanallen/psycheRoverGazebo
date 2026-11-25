"""Smoothly move all six legs to one posture, then exit.

First piece of the locomotion package: it shows the command path the gait
node will use (read /joint_states, publish 18 joint angles) and gives a quick
way to test postures and servo torque in sim without writing code.

    # standing pose (all zeros)
    ros2 run psyche_locomotion stand_node --ros-args -p use_sim_time:=true

    # crouch: femur raised 0.4 rad, foot tucked 0.3 rad
    ros2 run psyche_locomotion stand_node --ros-args -p use_sim_time:=true \
        -p femur:=0.4 -p tibia:=-0.3

Angles are radians, same for all legs. Sign conventions are in
psyche_description/urdf/params.xacro.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

LEGS = ['lf', 'lm', 'lr', 'rf', 'rm', 'rr']
JOINT_TYPES = ['hip', 'femur', 'tibia']
# Must match psyche_description/config/controllers.yaml
JOINT_ORDER = [f'{leg}_{jt}_joint' for leg in LEGS for jt in JOINT_TYPES]


class StandNode(Node):

    def __init__(self):
        super().__init__('stand_node')
        self.declare_parameter('hip', 0.0)
        self.declare_parameter('femur', 0.0)
        self.declare_parameter('tibia', 0.0)
        self.declare_parameter('duration', 2.0)  # seconds to reach the pose
        self.declare_parameter('rate', 50.0)     # Hz

        per_leg = [self.get_parameter(jt).value for jt in JOINT_TYPES]
        self.target = per_leg * len(LEGS)
        self.duration = max(0.1, float(self.get_parameter('duration').value))

        self.start = None
        self.t0 = None
        self.done = False

        self.pub = self.create_publisher(
            Float64MultiArray, '/leg_position_controller/commands', 10)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self.on_timer)
        self.get_logger().info('Waiting for /joint_states ...')

    def on_joint_states(self, msg):
        if self.start is not None:
            return
        pos = dict(zip(msg.name, msg.position))
        if not all(j in pos for j in JOINT_ORDER):
            return
        self.start = [pos[j] for j in JOINT_ORDER]
        self.t0 = self.get_clock().now()
        self.get_logger().info(f'Moving to posture over {self.duration:.1f} s')

    def on_timer(self):
        if self.start is None or self.done:
            return
        elapsed = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        s = min(1.0, elapsed / self.duration)
        s = 0.5 - 0.5 * math.cos(math.pi * s)  # ease in and out
        cmd = Float64MultiArray()
        cmd.data = [a + s * (b - a) for a, b in zip(self.start, self.target)]
        self.pub.publish(cmd)
        if s >= 1.0:
            self.done = True
            self.get_logger().info('Posture reached. The controller holds it.')


def main():
    rclpy.init()
    node = StandNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
