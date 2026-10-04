from grupo03_kuka_kr6_kinematics.fk import fk
from grupo03_kuka_kr6_kinematics.ik import (
    ik_position,
    jacobian_geometrico,
    Q_MAX,
    Q_MIN,
)
import numpy as np
import pytest


# Configuraciones reales registradas en /joint_states.
CONFIGURACIONES = [
    [0, -4.363323129963348e-05, -0.00039793506945473567, 0, 0, 0],
    [0, -1.564774940875516, -0.00039793506945473567, 0, 0, 0],
    [
        0.7856774160777671,
        -0.7887666488537972,
        0.795870138909414,
        0,
        0,
        0,
    ],
]

# Posiciones mostradas por TF, redondeadas a tres decimales.
POSICIONES_TF = [
    [0.980, 0.000, 0.435],
    [-0.004, 0.000, 1.355],
    [0.598, -0.598, 0.754],
]


@pytest.mark.parametrize('q, esperado', list(zip(CONFIGURACIONES, POSICIONES_TF)))
def test_fk_contra_tf(q, esperado):
    """Comparar FK con las mediciones registradas de TF."""
    posicion = fk(q)[:3, 3]

    # TF se registró a resolución de 0.001 m:
    # admitir hasta la mitad de esa unidad por redondeo.
    np.testing.assert_allclose(posicion, esperado, atol=5e-4, rtol=0)


@pytest.mark.parametrize(
    'q',
    CONFIGURACIONES + [[0.2, -0.8, 0.9, 0.4, -0.5, 0.6]],
)
def test_jacobiano_diferencias_finitas(q):
    """Comprobar también una postura con la muñeca girada."""
    q = np.asarray(q, dtype=float)
    h = 1e-6
    numerico = np.zeros((3, 6))

    for i in range(6):
        delta = np.zeros(6)
        delta[i] = h
        numerico[:, i] = (fk(q + delta)[:3, 3] - fk(q - delta)[:3, 3]) / (2 * h)

    np.testing.assert_allclose(jacobian_geometrico(q)[:3, :], numerico, atol=1e-7, rtol=0)

    # Velocidad angular espacial: [omega]x = R_dot @ R.T.
    R = fk(q)[:3, :3]
    angular = np.zeros((3, 6))
    for i in range(6):
        delta = np.zeros(6)
        delta[i] = h
        R_dot = (fk(q + delta)[:3, :3] - fk(q - delta)[:3, :3]) / (2 * h)
        skew = R_dot @ R.T
        angular[:, i] = [skew[2, 1], skew[0, 2], skew[1, 0]]
    np.testing.assert_allclose(jacobian_geometrico(q)[3:, :], angular, atol=1e-7, rtol=0)


@pytest.mark.parametrize(
    'objetivo',
    [[0.6, -0.3, 0.7], [0.5, 0.2, 0.8], [0.7, 0.0, 0.5]],
)
def test_ik_posicion_y_limites(objetivo):
    """Verificar convergencia, error y límites articulares."""
    q, exito, iteraciones, error = ik_position(objetivo, CONFIGURACIONES[2])

    error_calculado = np.linalg.norm(np.asarray(objetivo) - fk(q)[:3, 3])

    assert exito
    assert iteraciones <= 300
    assert error_calculado <= 1e-4
    assert abs(error - error_calculado) < 1e-12
    assert np.all(q >= Q_MIN)
    assert np.all(q <= Q_MAX)
