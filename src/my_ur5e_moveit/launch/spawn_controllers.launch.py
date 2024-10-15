from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("my_robot_cell", package_name="my_ur5e_moveit2").to_moveit_configs()
    return generate_spawn_controllers_launch(moveit_config)
