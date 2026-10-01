import numpy as np


def dh(theta, d, a, alpha):
    """Transformación homogénea DH estándar."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d],
        [0,   0,        0,       1],
    ], dtype=float)


def fk(q):
    """Pose de tool0 respecto a base_link; q en radianes."""
    q = np.asarray(q, dtype=float)

    if q.shape != (6,) or not np.all(np.isfinite(q)):
        raise ValueError("Se requieren seis ángulos finitos en radianes.")

    theta = q + np.array([0, 0, -np.pi / 2, 0, 0, 0])
    d = [-0.400, 0, 0, -0.420, 0, -0.080]
    a = [0.025, 0.455, 0.035, 0, 0, 0]
    alpha = [np.pi / 2, 0, np.pi / 2, -np.pi / 2, np.pi / 2, 0]

    # Transformación fija de base_link al frame DH 0: Rx(pi).
    T = np.diag([1.0, -1.0, -1.0, 1.0])

    for i in range(6):
        T = T @ dh(theta[i], d[i], a[i], alpha[i])

    # Transformación fija del frame DH 6 a tool0: Ry(pi).
    T = T @ np.diag([-1.0, 1.0, -1.0, 1.0])

    return T

def jacobian_pos(q):
    """Jacobiano de posición de tool0, expresado en base_link."""
    q = np.asarray(q, dtype=float)

    if q.shape != (6,) or not np.all(np.isfinite(q)):
        raise ValueError("Se requieren seis ángulos finitos en radianes.")

    theta = q + np.array([0, 0, -np.pi / 2, 0, 0, 0])
    d = [-0.400, 0, 0, -0.420, 0, -0.080]
    a = [0.025, 0.455, 0.035, 0, 0, 0]
    alpha = [np.pi / 2, 0, np.pi / 2, -np.pi / 2, np.pi / 2, 0]

    T = np.diag([1.0, -1.0, -1.0, 1.0])
    origins = []
    axes = []

    for i in range(6):
        # Guardar el frame ANTERIOR a la transformación i.
        origins.append(T[:3, 3].copy())
        axes.append(T[:3, 2].copy())
        T = T @ dh(theta[i], d[i], a[i], alpha[i])

    # La transformación final a tool0 solo rota: no cambia su posición.
    p = T[:3, 3]

    J = np.zeros((3, 6))
    for i in range(6):
        J[:, i] = np.cross(axes[i], p - origins[i])

    return J

Q_MIN = np.deg2rad([-170, -190, -120, -185, -120, -350])
Q_MAX = np.deg2rad([170, 45, 156, 185, 120, 350])


def ik_position(target, q0):
    """Retorna: ángulos, éxito, iteraciones y error de posición [m]."""
    target = np.asarray(target, dtype=float)
    q = np.asarray(q0, dtype=float)

    if target.shape != (3,) or not np.all(np.isfinite(target)):
        raise ValueError("El objetivo debe contener tres valores finitos.")

    if q.shape != (6,) or not np.all(np.isfinite(q)):
        raise ValueError("La inicialización debe contener seis ángulos finitos.")

    q = np.clip(q, Q_MIN, Q_MAX)

    tol = 1e-4          # 0.1 mm de error de posición.
    damping = 0.02     # Amortiguamiento.
    max_step = 0.2     # Máxima norma del incremento articular [rad].
    max_iter = 300

    for iteration in range(max_iter + 1):
        e = target - fk(q)[:3, 3]
        error = float(np.linalg.norm(e))

        if error <= tol:
            return q, True, iteration, error

        if iteration == max_iter:
            break

        J = jacobian_pos(q)

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

if __name__ == "__main__":
    # Ángulos reales registrados en /joint_states.
    configuraciones = [
        [0, -4.363323129963348e-05, -0.00039793506945473567,
         0, 0, 0],

        [0, -1.564774940875516, -0.00039793506945473567,
         0, 0, 0],

        [0.7856774160777671, -0.7887666488537972,
         0.795870138909414, 0, 0, 0],
    ]

    np.set_printoptions(precision=6, suppress=True)

    for numero, q in enumerate(configuraciones, start=1):
        T = fk(q)
        print(f"\nConfiguración {numero}")
        print("Posición [m]:", T[:3, 3])
        print("Matriz base_link → tool0:")
        print(T)
