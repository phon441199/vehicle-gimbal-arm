import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .joint_config import JOINT_NAMES, JOINT_LIMITS, NUM_JOINTS


class ArmControllerNode(Node):
    def __init__(self, node_name: str = 'arm_controller'):
        super().__init__(node_name)

        # micro-ROS(Teensy)로 명령 전송
        self._cmd_pub = self.create_publisher(
            JointState,
            '/joint_commands',
            10
        )

        # micro-ROS(Teensy)로부터 현재 상태 수신
        self._state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )

        # 현재 수신된 조인트 상태 저장
        self._current_positions = [0.0] * NUM_JOINTS
        self._current_velocities = [0.0] * NUM_JOINTS
        self._current_efforts = [0.0] * NUM_JOINTS

        self.get_logger().info(
            f'ArmControllerNode 시작 — 조인트: {JOINT_NAMES}'
        )

    def _joint_state_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in JOINT_NAMES:
                idx = JOINT_NAMES.index(name)
                if i < len(msg.position):
                    self._current_positions[idx] = msg.position[i]
                if i < len(msg.velocity):
                    self._current_velocities[idx] = msg.velocity[i]
                if i < len(msg.effort):
                    self._current_efforts[idx] = msg.effort[i]

    def send_joint_command(self, positions: list[float]):
        """
        positions: 각 조인트의 목표 각도 (라디안), JOINT_NAMES 순서
        """
        if len(positions) != NUM_JOINTS:
            self.get_logger().error(
                f'위치 배열 크기 불일치: {len(positions)} != {NUM_JOINTS}'
            )
            return

        clamped = []
        for i, (pos, name) in enumerate(zip(positions, JOINT_NAMES)):
            lo, hi = JOINT_LIMITS[name]
            clamped.append(max(lo, min(hi, pos)))

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = clamped
        msg.velocity = [0.0] * NUM_JOINTS
        msg.effort = [0.0] * NUM_JOINTS

        self._cmd_pub.publish(msg)

    @property
    def current_positions(self) -> list[float]:
        return list(self._current_positions)


def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
