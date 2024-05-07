from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import xacro,os
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.launch_description_sources import PythonLaunchDescriptionSource

import os

PREFIX_LIST = ['1_', '2_']

def generate_launch_description():
    # suppress the output
    tf_pub1 = Node(
        name='tf_pub' + PREFIX_LIST[0],
        package='project',
        executable='tf_publisher',
        output='screen',
        parameters=[{"use_sim_time": True},
                    {'prefix': PREFIX_LIST[0]}]                  
    )    
    tf_pub2 = Node(
        name='tf_pub' + PREFIX_LIST[1],
        package='project',
        executable='tf_publisher',
        output='screen',
        parameters=[{"use_sim_time": True},
                    {'prefix': PREFIX_LIST[1]}]          
    )

    static_trans_broadcaster1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1_base_link', '1_camera'],
        parameters=[{"use_sim_time": True}],
        output='screen',
    )
    static_trans_broadcaster2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '2_base_link', '2_camera'],
        parameters=[{"use_sim_time": True}],
        output='screen',
    )
    
    pointcloud_accumulator = Node(
        package='project',
        executable='pointcloud_accumulator',
        output='screen',
        parameters=[{"use_sim_time": True},
                    {'voxel_grid_size': 0.005},
                    {'max_cloud_size': 50000},
                    {'input_topic_1': '/cloud_out_1'},
                    {'input_topic_2': '/cloud_out_2'},
                    {'output_topic': '/total_cloud'},
                    {'frame_id': 'world'},
                    ]
    )
    
    rviz_config = os.path.join(get_package_share_directory('project'), 'rviz_config','rviz_point.rviz')
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}]
    )

        # include the launch file for point clouds
    point_clouds_converter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ur_coppeliasim'), 'launch', 'pc2_coppeliasim.launch.py')
        ),
        launch_arguments=[('prefixes', PREFIX_LIST),('handle_name_1','/depth1'), ('handle_name_2','/depth2')]
    )

    node_list = TimerAction(period=0.0,
            actions=[
                static_trans_broadcaster1,
                static_trans_broadcaster2,
                tf_pub1,
                tf_pub2,
                point_clouds_converter,
                pointcloud_accumulator,
                rviz,
                
            ])

    return LaunchDescription([node_list])


