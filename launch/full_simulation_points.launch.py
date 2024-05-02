from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import xacro,os
from launch_ros.actions import Node
from launch.actions import TimerAction
# from launch.launch_description_sources import PythonLaunchDescriptionSource

import os

PREFIX_LIST = ['1_', '2_']

def generate_launch_description():
    # suppress the output
    point_robot_1_tf_pub = Node(
        name='point_robot_1_tf_pub',
        package='point_robot_description',
        executable='tf_pub',
        output='screen',
        parameters=[{"use_sim_time": True},
                    {'prefix': PREFIX_LIST[0]}]                  
    )    
    point_robot_2_tf_pub = Node(
        name='point_robot_2_tf_pub',
        package='point_robot_description',
        executable='tf_pub',
        output='screen',
        parameters=[{"use_sim_time": True},
                    {'prefix': PREFIX_LIST[1]}]          
    )

    static_trans_broadcaster1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0.0141', '-0.02925', '3.14159', '0', '1.5708', '1_base_link', '1_camera'],
        output='screen',
    )
    static_trans_broadcaster2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0.0141', '-0.02925', '3.14159', '0', '1.5708', '2_base_link', '2_camera'],
        output='screen',
    )
    
    pointcloud_accumulator = Node(
        package='project',
        executable='pointcloud_accumulator',
        output='screen',
        parameters=[{"use_sim_time": True},
                    {'voxel_grid_size': 0.01},
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

    node_list = TimerAction(period=0.0,
            actions=[
                static_trans_broadcaster1,
                static_trans_broadcaster2,
                point_robot_1_tf_pub,
                point_robot_2_tf_pub,
                # pointcloud_accumulator,
                rviz,
                
            ])

    return LaunchDescription([node_list])


