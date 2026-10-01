# Cinemática del KUKA KR 6 R900 sixx

Proyecto de IMT-342 Robótica, Universidad Católica Boliviana “San Pablo”.

**Grupo 3**
- Cristina Montaño
- Fernando Aramayo

Implementación de cinemática directa e inversa de posición para un
manipulador de seis articulaciones, con ROS 2 Jazzy y visualización en RViz2.

## Funcionalidades

- Cinemática directa mediante Denavit–Hartenberg estándar.
- Pose de tool0 expresada respecto a base_link.
- Jacobiano geométrico de posición de tamaño 3×6.
- Cinemática inversa mediante mínimos cuadrados amortiguados (DLS).
- Límites articulares y comprobación de convergencia.
- Publicación de la pose mediante /fk_pose.
- Recepción de objetivos cartesianos mediante /target.
- Lanzamiento conjunto de RViz2, robot_state_publisher, FK e IK.

## Requisitos

- Ubuntu 24.04.
- ROS 2 Jazzy instalado y configurado.
- Conexión a Internet para descargar dependencias.
- Entorno gráfico para RViz2.

La implementación matemática utiliza NumPy; no requiere SciPy ni SymPy.

## Instalación

Con ROS 2 Jazzy ya instalado:

```bash
sudo apt update
sudo apt install -y \
  git python3-colcon-common-extensions python3-vcstool python3-rosdep \
  python3-numpy ros-jazzy-rmw-cyclonedds-cpp
```

Si rosdep nunca se inicializó en la computadora, ejecutar una sola vez:

```bash
sudo rosdep init
```

Si ya está inicializado, omitir ese comando. Después:

```bash
rosdep update

git clone https://github.com/montanocmmc/Forward-and-Inverse-Kinematics-of-the-KUKA-KR6-R900.git
cd Forward-and-Inverse-Kinematics-of-the-KUKA-KR6-R900

source /opt/ros/jazzy/setup.bash
vcs import src < dependencies.repos

rosdep install --from-paths \
  src/grupo03_kuka_kr6_bringup \
  src/grupo03_kuka_kr6_kinematics \
  src/kuka_robot_descriptions/kuka_resources \
  src/kuka_robot_descriptions/kuka_agilus_support \
  --ignore-src --rosdistro jazzy -y

colcon build --symlink-install --packages-select \
  kuka_resources \
  kuka_agilus_support \
  grupo03_kuka_kr6_bringup \
  grupo03_kuka_kr6_kinematics

source entorno.sh
```

El archivo dependencies.repos fija la versión del modelo KUKA al commit:

```text
f0202b281d40d16c90ceeed23d1a9548dd0c3981
```

El modelo pertenece al repositorio externo:
https://github.com/kroshu/kuka_robot_descriptions

## Ejecución de FK e IK

Desde la raíz del proyecto:

```bash
source entorno.sh
ros2 launch grupo03_kuka_kr6_kinematics kinematics.launch.py
```

Este launch inicia:

- robot_state_publisher
- RViz2
- fk_node
- ik_node

El nodo IK publica inicialmente la configuración de referencia 3.
Después mantiene la última solución válida a 10 Hz.

En otra terminal, entrar en la raíz del proyecto y ejecutar:

```bash
source entorno.sh
ros2 topic pub --once /target geometry_msgs/msg/Point \
  "{x: 0.6, y: -0.3, z: 0.7}"
```

Los objetivos están expresados en metros respecto a base_link.
La IK controla la posición de tool0; su orientación queda libre.

Otros objetivos comprobados:

```bash
ros2 topic pub --once /target geometry_msgs/msg/Point \
  "{x: 0.5, y: 0.2, z: 0.8}"

ros2 topic pub --once /target geometry_msgs/msg/Point \
  "{x: 0.7, y: 0.0, z: 0.5}"
```

Enviar cada objetivo por separado y esperar la convergencia.

## Comprobación

```bash
ros2 node list
ros2 topic info /joint_states
ros2 topic echo /fk_pose --once
ros2 run tf2_ros tf2_echo base_link tool0
```

En modo IK debe existir un único publicador de /joint_states.
Detener tf2_echo con Ctrl+C.

## Visualización con controles manuales

Como alternativa al launch de cinemática:

```bash
source entorno.sh
ros2 launch grupo03_kuka_kr6_bringup display.launch.py
```

Este modo abre Joint State Publisher GUI.
Detener el launch de IK antes de utilizar el modo manual para evitar
publicadores simultáneos de /joint_states.

## Tópicos

| Tópico | Tipo | Función |
|---|---|---|
| /target | geometry_msgs/msg/Point | Objetivo de posición en base_link |
| /joint_states | sensor_msgs/msg/JointState | Posiciones articulares en radianes |
| /fk_pose | geometry_msgs/msg/PoseStamped | Pose calculada de tool0 en base_link |

## Método de cinemática inversa

- Inicialización: configuración 3; luego, última solución válida.
- Tolerancia de posición: 0.0001 m.
- Máximo de iteraciones: 300.
- Amortiguamiento DLS: 0.02.
- Norma máxima del incremento articular: 0.2 rad.
- Proyección a los límites articulares del modelo.
- Reducción del paso cuando no disminuye el error.

Si no converge, se conserva la última postura válida.
La falta de convergencia desde una inicialización no demuestra
que el objetivo sea inalcanzable.

El algoritmo no realiza planificación de trayectorias ni comprobación
de colisiones. La publicación articular se utiliza para visualización.

## Validación realizada

- Tres configuraciones de FK contrastadas con TF.
- Jacobiano contrastado con diferencias finitas.
- Tres objetivos de IK comprobados numéricamente y mediante ROS 2.
- Lanzamiento conjunto con un publicador de /joint_states.

Errores de posición obtenidos en las ejecuciones registradas:

| Objetivo [m] | Error aproximado [mm] |
|---|---:|
| (0.6, -0.3, 0.7) | 0.0219 |
| (0.5, 0.2, 0.8) | 0.00333 |
| (0.7, 0.0, 0.5) | 0.0304 |

Las soluciones y el número de iteraciones pueden cambiar según
la configuración inicial.

## Organización

- src/grupo03_kuka_kr6_bringup/: visualización manual.
- src/grupo03_kuka_kr6_kinematics/: funciones matemáticas, nodos y launch.
- dependencies.repos: versión de la dependencia KUKA.
- entorno.sh: carga ROS 2, CycloneDDS y la instalación del workspace.

Las carpetas build/, install/ y log/ se generan localmente y no se incluyen
en el repositorio. El modelo KUKA se descarga mediante dependencies.repos.

## Pruebas matemáticas automatizadas

Desde la raíz del proyecto:

```bash
source entorno.sh
cd src/grupo03_kuka_kr6_kinematics
python3 -m pytest -q test/test_kinematics.py
```

Si pytest no está instalado:

```bash
sudo apt install python3-pytest
```

Se incluyen 10 pruebas:
- Tres comparaciones de FK con posiciones registradas de TF.
- Cuatro comparaciones del Jacobiano con diferencias finitas.
- Tres comprobaciones de IK, tolerancia y límites articulares.

Resultado registrado: 10 pruebas satisfactorias.

También se verificó la compilación de los cuatro paquetes desde una
copia nueva del repositorio en la misma computadora con Ubuntu 24.04
y ROS 2 Jazzy.
