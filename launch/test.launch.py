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
    voronoi_calculator_1 = Node(
        package='project',
        executable='voronoi_calculator',
        name='voronoi_calculator_1',
        output='screen',
        # prefix=['xterm -e gdb -ex run --args'],
        parameters=[{"use_sim_time": True},
        {"debug": True},
        {"prefix_1": "1_"},
        {"prefix_2": "2_"},
        {"voronoi_frame": "world"},
        {"input_frame": "camera"},
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
        parameters=[{"use_sim_time": True},
        {"debug": False},
        {"prefix_1": "2_"},
        {"prefix_2": "1_"},
        {"voronoi_frame": "world"},
        {"input_frame": "camera"},
        {"base_frame": "base_link"},
        {"input_frame_other_robot": "camera"},
        {"target_topic": "target_frame"},
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
    static_broadcaster_1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ["--y", "-0.75", "--frame-id", "world", "--child-frame-id","1_base_link"],
        parameters=[{"use_sim_time": True}],
        output="log"
    )
    static_broadcaster_2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ["--y","0.75","--yaw", "-3.1416", "--frame-id", "world", "--child-frame-id", "2_base_link"],
        parameters=[{"use_sim_time": True}],
        output="log"
    )
    node_list = [
        # pointcloud_accumulator,
        # rviz
        voronoi_calculator_1,
        # voronoi_calculator_2,
        # static_broadcaster_1,
        # static_broadcaster_2,
        
        ]
    return LaunchDescription(node_list )#+ [integrator_t])    


