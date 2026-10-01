import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

from grupo03_kuka_kr6_kinematics.kinematics import ik_position


class IKNode(Node):
    def __init__(self):
        super().__init__("ik_node")

        # Inicialización: configuración 3 utilizada en las pruebas.
        self.q = [
            0.7856774160777671,
            -0.7887666488537972,
            0.795870138909414,
            0.0, 0.0, 0.0,
        ]
        self.have_solution = True

        self.publisher = self.create_publisher(
            JointState, "/joint_states", 10
        )
        self.subscription = self.create_subscription(
            Point, "/target", self.target_callback, 10
        )

        # Mantener publicada la última solución a 10 Hz.
        self.timer = self.create_timer(0.1, self.publish_solution)

        self.get_logger().info(
            "IK listo. Esperando /target en metros respecto a base_link."
        )

    def target_callback(self, msg):
        target = [msg.x, msg.y, msg.z]

        try:
            q, success, iterations, error = ik_position(target, self.q)
        except ValueError as exc:
            self.get_logger().warning(str(exc))
            return

        if not success:
            self.get_logger().warning(
                f"Sin convergencia para {target}: "
                f"error={error:.6f} m, iteraciones={iterations}. "
                "Se conserva la última solución válida."
            )
            return

        self.q = q.tolist()
        self.have_solution = True
        self.publish_solution()

        self.get_logger().info(
            f"Objetivo={target}; error={error:.8f} m; "
            f"iteraciones={iterations}; "
            f"q={[round(value, 6) for value in self.q]}"
        )

    def publish_solution(self):
        if not self.have_solution:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f"joint_{i}" for i in range(1, 7)]
        msg.position = self.q
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IKNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
