# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node



def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
                'robot_name',
                default_value='mecanum_bot',
                description='The name of the robot sdf model.'
            )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
                'robot_sdf',
                default_value=os.path.join(
                    get_package_share_directory('mecanum_bot_description'),
                    'mecanum_bot',
                    'model.sdf'
                ),
                description='The name of the robot sdf model.'
            )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
                'rviz',
                default_value='true',
                description='Open RViz.'
            )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'world',
            default_value='example_world.sdf',
            description='Filename of the world in the world subdirectory'
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')
    rviz_launch_arg = LaunchConfiguration('rviz')
    world = LaunchConfiguration('world')

    pkg_ros_gz_sim_demos = get_package_share_directory('mecanum_bot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    rviz_file = os.path.join(pkg_ros_gz_sim_demos, 'rviz', 'view_robot.rviz')


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_ros_gz_sim_demos,
            'worlds',
            world
        ])}.items(),
    )

    # Bridge to forward tf and joint states to ros2
    gz_topic = '/model/' + robot_name.perform(context)
    joint_state_gz_topic = '/world/' + world.perform(context).split('.')[0] + gz_topic + '/joint_state'
    link_pose_gz_topic = gz_topic + '/pose'
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (Gazebo -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint states (Gazebo -> ROS2)
            joint_state_gz_topic + '@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Link poses (Gazebo -> ROS2)
            link_pose_gz_topic + '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            link_pose_gz_topic + '_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Velocity and odometry (Gazebo -> ROS2)
            gz_topic + '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            gz_topic + '/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
        ],
        remappings=[
            (joint_state_gz_topic, 'joint_states'),
            (link_pose_gz_topic, '/tf'),
            (link_pose_gz_topic + '_static', '/tf_static'),
            (gz_topic + '/odometry', '/wheel_odom'),\
            (gz_topic + '/cmd_vel', '/cmd_vel'),
            ('/rgbd_camera/image', '/oak/rgb/image_raw'),
            ('/rgbd_camera/camera_info', '/oak/rgb/camera_info'),
            ('/rgbd_camera/depth_image', '/oak/stereo/image_raw'),
        ],
        parameters=[{'qos_overrides./tf_static.publisher.durability': 'transient_local'}],
        output='screen'
    )

    # Get the parser plugin convert sdf to urdf using robot_description topic
    with open(robot_sdf.perform(context), 'r') as infp:
        robot_desc = infp.read()
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    # Launch rviz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        condition=IfCondition(rviz_launch_arg),
        parameters=[
            {'use_sim_time': True},
        ]
    )

    return [
        gazebo,
        bridge,
        robot_state_publisher,
        rviz
    ]
