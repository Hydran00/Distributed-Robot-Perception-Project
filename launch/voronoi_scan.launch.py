from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import xacro,os
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os,time
sim_time = False
def generate_launch_description():
    # include the launch file for point clouds
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ur_coppeliasim'), 'launch', 'multi_robot.launch.py')
        ),
        # suppress the output
        
    )
    voronoi_calculator_1 = Node(
        package='project',
        executable='voronoi_calculator',
        name='voronoi_calculator_1',
        output='screen',
        # prefix=['xterm -e gdb -ex run --args'],
        parameters=[{"use_sim_time": sim_time},
        {"debug": True},
        {"prefix_1": "1_"},
        {"prefix_2": "2_"},
        {"voronoi_frame": "world"},
        {"input_frame": "camera"},
        {"output_frame": "world"}, # world for point robot or 1_base_link 
        {"base_frame": "base_link"},
        {"input_frame_other_robot": "camera"},
        {"target_topic": "target_frame"},
        ]
    )
    voronoi_calculator_2 = Node(
        package='project',
        executable='voronoi_calculator',
        output='screen',
        name='voronoi_calculator_2',
        parameters=[{"use_sim_time": sim_time},
        {"debug": False},
        {"prefix_1": "2_"},
        {"prefix_2": "1_"},
        {"voronoi_frame": "world"}, # world for point robot or 2_base_link
        {"input_frame": "camera"},
        {"output_frame": "world"},
        {"base_frame": "base_link"},
        {"input_frame_other_robot": "camera"},
        {"target_topic": "target_frame"},
        ]
    )
    integrator = Node(
        package='project',
        executable='integrator',
        output='screen',
        parameters=[{"use_sim_time": sim_time},
            ]
    )
    integrator_t = TimerAction(period=2.0,
            actions=[integrator]
            )

    node_list = [
        voronoi_calculator_1,
        voronoi_calculator_2,
        
        ]
    return LaunchDescription(node_list)    


