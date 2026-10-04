"""Cinemática directa DH y publicación de /fk_pose."""

import numpy as np


def dh(theta, d, a, alpha):
    """Transformación homogénea DH estándar."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0, sa, ca, d],
            [0, 0, 0, 1],
        ],
        dtype=float,
    )


def get_fk_matrices(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (6,) or not np.all(np.isfinite(q)):
        raise ValueError('Se requieren seis ángulos finitos en radianes.')

    T_base = np.diag([1.0, -1.0, -1.0, 1.0])
    A1 = dh(q[0], -0.400, 0.025, np.pi / 2)
    A2 = dh(q[1], 0.0, 0.455, 0.0)
    A3 = dh(q[2] - np.pi / 2, 0.0, 0.035, np.pi / 2)
    A4 = dh(q[3], -0.420, 0.0, -np.pi / 2)
    A5 = dh(q[4], 0.0, 0.0, np.pi / 2)
    A6 = dh(q[5], -0.080, 0.0, 0.0)
    T_tool = np.diag([-1.0, 1.0, -1.0, 1.0])
    return [T_base, A1, A2, A3, A4, A5, A6, T_tool]


def fk(q):
    m = get_fk_matrices(q)
    return m[0] @ m[1] @ m[2] @ m[3] @ m[4] @ m[5] @ m[6] @ m[7]


def quaternion(R):
    """Convierte una matriz de rotación a cuaternión [x, y, z, w]."""
    q = np.zeros(4)
    tr = np.trace(R)

    if tr > 0:
        s = 2 * np.sqrt(tr + 1)
        q[3] = s / 4
        q[0] = (R[2, 1] - R[1, 2]) / s
        q[1] = (R[0, 2] - R[2, 0]) / s
        q[2] = (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2 * np.sqrt(1 + R[i, i] - R[j, j] - R[k, k])
        q[i] = s / 4
        q[j] = (R[j, i] + R[i, j]) / s
        q[k] = (R[k, i] + R[i, k]) / s
        q[3] = (R[k, j] - R[j, k]) / s

    return q / np.linalg.norm(q)


def main(args=None):
    """Iniciar el nodo ROS; las funciones matemáticas solo requieren NumPy."""
    from geometry_msgs.msg import PoseStamped
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState

    class FKNode(Node):
        def __init__(self):
            super().__init__('fk_node')

            self.joint_names = [f'joint_{i}' for i in range(1, 7)]
            self.publisher = self.create_publisher(PoseStamped, '/fk_pose', 10)
            self.subscription = self.create_subscription(
                JointState,
                '/joint_states',
                self.joint_callback,
                qos_profile_sensor_data,
            )

            self.get_logger().info('FK activo: /joint_states -> /fk_pose (base_link a tool0)')

        def joint_callback(self, msg):
            # Asociar nombres y posiciones evita depender del orden recibido.
            if len(msg.name) != len(msg.position):
                self.get_logger().warning(
                    'JointState con nombres y posiciones de distinto tamaño.'
                )
                return

            positions = dict(zip(msg.name, msg.position))

            if any(name not in positions for name in self.joint_names):
                self.get_logger().warning('Faltan articulaciones del robot en /joint_states.')
                return

            q = [positions[name] for name in self.joint_names]

            try:
                T = fk(q)
            except ValueError as error:
                self.get_logger().warning(str(error))
                return

            orientation = quaternion(T[:3, :3])

            pose = PoseStamped()
            pose.header.stamp = msg.header.stamp
            pose.header.frame_id = 'base_link'

            pose.pose.position.x = float(T[0, 3])
            pose.pose.position.y = float(T[1, 3])
            pose.pose.position.z = float(T[2, 3])

            pose.pose.orientation.x = float(orientation[0])
            pose.pose.orientation.y = float(orientation[1])
            pose.pose.orientation.z = float(orientation[2])
            pose.pose.orientation.w = float(orientation[3])

            self.publisher.publish(pose)

    rclpy.init(args=args)
    node = FKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
