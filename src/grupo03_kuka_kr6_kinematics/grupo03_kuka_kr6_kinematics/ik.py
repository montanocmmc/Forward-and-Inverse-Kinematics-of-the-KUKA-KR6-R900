"""Jacobiano geométrico, IK de posición DLS y nodo /target."""

from grupo03_kuka_kr6_kinematics.fk import fk, get_fk_matrices
import numpy as np


Q_MIN = np.deg2rad([-170, -190, -120, -185, -120, -350])
Q_MAX = np.deg2rad([170, 45, 156, 185, 120, 350])


def jacobian_geometrico(q):
    m = get_fk_matrices(q)
    T = m[0]
    origins = []
    axes = []

    for i in range(1, 7):
        origins.append(T[:3, 3].copy())
        axes.append(T[:3, 2].copy())
        T = T @ m[i]

    T = T @ m[7]
    p_n = T[:3, 3]

    J = np.zeros((6, 6))
    for i in range(6):
        z_i = axes[i]
        p_i = origins[i]
        J[:3, i] = np.cross(z_i, p_n - p_i)
        J[3:, i] = z_i

    return J


def ik_position(target, q0):
    """Retorna: ángulos, éxito, iteraciones y error de posición [m]."""
    target = np.asarray(target, dtype=float)
    q = np.asarray(q0, dtype=float)

    if target.shape != (3,) or not np.all(np.isfinite(target)):
        raise ValueError('El objetivo debe contener tres valores finitos.')

    if q.shape != (6,) or not np.all(np.isfinite(q)):
        raise ValueError('La inicialización debe contener seis ángulos finitos.')

    q = np.clip(q, Q_MIN, Q_MAX)

    tol = 1e-4  # 0.1 mm de error de posición.
    damping = 0.02  # Amortiguamiento.
    max_step = 0.2  # Máxima norma del incremento articular [rad].
    max_iter = 300

    for iteration in range(max_iter + 1):
        e = target - fk(q)[:3, 3]
        error = float(np.linalg.norm(e))

        if error <= tol:
            return q, True, iteration, error

        if iteration == max_iter:
            break

        J = jacobian_geometrico(q)[:3, :]

        dq = J.T @ np.linalg.solve(
            J @ J.T + damping**2 * np.eye(3),
            e,
        )

        # Limitar el tamaño del paso.
        norm = np.linalg.norm(dq)
        if norm > max_step:
            dq *= max_step / norm

        # Reducir el paso si hace falta para disminuir el error.
        for scale in (1.0, 0.5, 0.25, 0.125, 0.0625, 0.03125):
            candidate = np.clip(q + scale * dq, Q_MIN, Q_MAX)
            new_error = np.linalg.norm(target - fk(candidate)[:3, 3])

            if new_error < error:
                q = candidate
                break
        else:
            return q, False, iteration, error

    return q, False, max_iter, error


def main(args=None):
    """Iniciar el nodo ROS; las funciones matemáticas solo requieren NumPy."""
    from geometry_msgs.msg import Point
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    class IKNode(Node):
        def __init__(self):
            super().__init__('ik_node')

            # Inicialización: configuración 3 utilizada en las pruebas.
            self.q = [
                0.7856774160777671,
                -0.7887666488537972,
                0.795870138909414,
                0.0,
                0.0,
                0.0,
            ]
            self.have_solution = True

            self.publisher = self.create_publisher(JointState, '/joint_states', 10)
            self.subscription = self.create_subscription(
                Point, '/target', self.target_callback, 10
            )

            # Mantener publicada la última solución a 10 Hz.
            self.timer = self.create_timer(0.1, self.publish_solution)

            self.get_logger().info('IK listo. Esperando /target en metros respecto a base_link.')

        def target_callback(self, msg):
            target = [msg.x, msg.y, msg.z]
            seed = list(self.q)

            try:
                q, success, iterations, error = ik_position(target, self.q)
            except ValueError as exc:
                self.get_logger().warning(str(exc))
                return

            if not success:
                self.get_logger().warning(
                    f'Sin convergencia para {target}: '
                    f'error={error:.6f} m, iteraciones={iterations}. '
                    'Se conserva la última solución válida.'
                )
                return

            self.q = q.tolist()
            self.have_solution = True
            self.publish_solution()

            self.get_logger().info(
                f'Objetivo={target}; q0={seed}; '
                f'posición={fk(q)[:3, 3].tolist()}; error={error:.8f} m; '
                f'iteraciones={iterations}; '
                f'q={[round(value, 6) for value in self.q]}'
            )

        def publish_solution(self):
            if not self.have_solution:
                return

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [f'joint_{i}' for i in range(1, 7)]
            msg.position = self.q
            self.publisher.publish(msg)

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


if __name__ == '__main__':
    main()
