from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Encontrar el paquete que contiene la descripción del UR5e
    description_package = FindPackageShare("my_ur5e_description")
    
    # Ruta al archivo URDF Xacro del UR5e
    description_file = PathJoinSubstitution(
        [description_package, "urdf", "ur5e_soft_hand.urdf.xacro"]
    )

    # Parámetro para la descripción del robot generada desde Xacro
    robot_description = ParameterValue(
        Command(["xacro ", description_file, " ", "ur_type:=", "ur5e"]), value_type=str
    )

    # Nodo para publicar el estado del robot
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    # Nodo para la interfaz gráfica del publicador de estados de las juntas
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rvizconfig_file = PathJoinSubstitution([description_package, "rviz", "ur5e_soft_hand.rviz"])

    # Nodo para RViz sin archivo de configuración predefinido
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rvizconfig_file],
    )

    # Devolver el LaunchDescription con los nodos
    return LaunchDescription(
        [joint_state_publisher_gui_node, robot_state_publisher_node, rviz_node]
    )
