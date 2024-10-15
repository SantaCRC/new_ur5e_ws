import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    # Paths
    moveit_config_pkg = get_package_share_directory('my_ur5e_moveit')
    ur5e_description_pkg = get_package_share_directory('my_ur5e_control')
    qbhand_description_pkg = get_package_share_directory('qb_hand_description')

    # Generate MoveIt config using MoveItConfigsBuilder
    moveit_config = MoveItConfigsBuilder("my_robot_cell", package_name="my_ur5e_moveit").to_moveit_configs()
    
    # Include MoveIt launch file
    moveit_launch = generate_demo_launch(moveit_config)

    # Include controller spawner for UR5e
    ur_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur5e_description_pkg, 'launch', 'demo.launch.py')  # Asegúrate de que este archivo exista
        )
    )

    # Launch qbHand using the specific command (assuming bringup_qbhand2m.launch is in XML format)
    qbhand_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(qbhand_description_pkg, 'launch', 'bringup_qbhand2m.launch')
        ),
        launch_arguments={
            'standalone': 'true',
            'activate_on_initialization': 'true',
            'device_id': '1',
            'use_rviz': 'false'  
        }.items()
    )

    # Launch python script to merge joint states
    joint_state_merger_node = Node(
        package='my_ur5e_moveit',  # Asegúrate de que el paquete y el archivo merge.py están correctamente configurados
        executable='merge.py',
        name='joint_state_merger',
        output='screen'
    )


    return LaunchDescription([
        # Launch MoveIt
        moveit_launch,

        # Launch qbHand with specific arguments
        qbhand_launch,

        # Launch the joint state merger node
        joint_state_merger_node,

        # Spawn the UR5e controllers
        ur_controllers_launch,  # Descomentar si necesitas lanzar controladores de UR5e
    ])
