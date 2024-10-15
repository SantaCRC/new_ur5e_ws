from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_warehouse_db_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("my_robot_cell", package_name="my_ur5e_moveit2").to_moveit_configs()
    return generate_warehouse_db_launch(moveit_config)