from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
                 'params_file:=./src/launch_pkg/config/mapper_params_mapping.yaml',
                 'use_sim_time:=false'],
            output='screen'
        )
    ])