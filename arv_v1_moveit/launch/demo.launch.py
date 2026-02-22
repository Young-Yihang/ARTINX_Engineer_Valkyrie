from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("arv_v1", package_name="arv_v1_moveit").to_moveit_configs()
    return generate_demo_launch(moveit_config)
