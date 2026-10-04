from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model_file = PathJoinSubstitution(
        [
            FindPackageShare('kuka_agilus_support'),
            'urdf',
            'kr6_r900_sixx.urdf.xacro',
        ]
    )

    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare('kuka_resources'),
            'config',
            'view_6_axis_urdf.rviz',
        ]
    )

    robot_description = {
        'robot_description': ParameterValue(
            Command(
                [
                    FindExecutable(name='xacro'),
                    ' ',
                    model_file,
                    ' mode:=mock',
                ]
            ),
            value_type=str,
        )
    }

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                'RMW_IMPLEMENTATION',
                'rmw_cyclonedds_cpp',
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[robot_description],
                output='screen',
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[robot_description],
                output='screen',
            ),
            Node(
                package='grupo03_kuka_kr6_kinematics',
                executable='fk_node',
                name='fk_node',
                output='screen',
            ),
            Node(
                package='grupo03_kuka_kr6_kinematics',
                executable='ik_node',
                name='ik_node',
                output='screen',
            ),
        ]
    )
