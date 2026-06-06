from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

def generate_launch_description():
    # Force the builder to ignore CHOMP and only load OMPL (Bi-RRT)
    moveit_config = (
        MoveItConfigsBuilder("so101_new_calib", package_name="so101_moveit_config")
        .planning_pipelines(pipelines=["ompl"]) 
        .to_moveit_configs()
    )
    
    launch_description = generate_move_group_launch(moveit_config)
    
    # Standard Sim Time handling
    launch_description.entities.insert(
        0,
        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")),
    )
    launch_description.entities.insert(
        0,
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    )
    
    return launch_description
