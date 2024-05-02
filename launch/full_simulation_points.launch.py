from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import xacro,os
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os,time

PREFIX_LIST = ['1_', '2_']

def generate_launch_description():
    # include the launch file for point clouds
    description_package = get_package_share_directory('point_robot_description')
    xacro_path = os.path.join(description_package,"urdf","point_robot.xacro")
    robot_controllers = os.path.join(description_package,"config", "multi_robot_controllers.yaml")

    ur_type="ur3e"
    robot_description_content1 = xacro.process_file(xacro_path, mappings={"prefix":PREFIX_LIST[0]})
    robot_description_content2 = xacro.process_file(xacro_path, mappings={"prefix":PREFIX_LIST[1]})
                                                                        

    robot_description_content1 = robot_description_content1.toprettyxml(indent=' ')
    robot_description_content2 = robot_description_content2.toprettyxml(indent=' ')

    robot_description1 = {"robot_description": robot_description_content1}
    robot_description2 = {"robot_description": robot_description_content2}

    # suppress the output
    point_robot_1_tf_pub = Node(
        name='point_robot_1_tf_pub',
        package='point_robot_description',
        executable='tf_pub',
        output='screen',
        parameters=[{"use_sim_time": True},
                    {'prefix': '1_'}]                    
    )    
    point_robot_2_tf_pub = Node(
        name='point_robot_2_tf_pub',
        package='point_robot_description',
        executable='tf_pub',
        output='screen',
        parameters=[{"use_sim_time": True},
                    {'prefix': '2_'}]                    
    )
    robot_state_publisher1 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        namespace="robot" + PREFIX_LIST[0],
        parameters=[robot_description1,{"use_sim_time": True, "publish_frequency": 100.0}],
    )
    robot_state_publisher2 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        namespace="robot" + PREFIX_LIST[1],
        parameters=[robot_description2,{"use_sim_time": True, "publish_frequency": 100.0}],
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
    
    rviz_config = os.path.join(get_package_share_directory('project'), 'rviz_config','rviz.rviz')
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
                robot_state_publisher1,
                robot_state_publisher2,
                point_robot_1_tf_pub,
                point_robot_2_tf_pub,
                # pointcloud_accumulator,
                # rviz,
                
            ])

    return LaunchDescription([node_list])


