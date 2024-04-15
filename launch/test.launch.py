from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import xacro,os
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os,time

def generate_launch_description():
    # include the launch file for point clouds
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ur_coppeliasim'), 'launch', 'multi_robot.launch.py')
        ),
        # suppress the output
        
    )
    pointcloud_accumulator = Node(
        package='project',
        executable='pointcloud_accumulator',
        output='screen',
        parameters=[{"use_sim_time": True},
                    {'voxel_grid_size': 0.01},
                    {'input_topic_1': '/cloud_out_1'},
                    {'input_topic_2': '/cloud_out_2'},
                    {'output_topic': '/total_cloud'},
                    {'frame_id': 'world'},
                    ]
    )
    
    rviz_config = os.path.join(get_package_share_directory('project'), 'rviz','robot.rviz')
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}]
    )
    voronoi_calculator = Node(
        package='project',
        executable='voronoi_calculator',
        output='screen',
        parameters=[{"use_sim_time": True},
            ]
    )
    integrator = Node(
        package='project',
        executable='integrator',
        output='screen',
        parameters=[{"use_sim_time": True},
            ]
    )
    integrator_t = TimerAction(period=2.0,
            actions=[integrator]
            )

    node_list = [
        # pointcloud_accumulator,
        # rviz
        voronoi_calculator
        ]
    return LaunchDescription(node_list + [integrator_t])


