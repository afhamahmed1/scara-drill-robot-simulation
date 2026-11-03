"""Start Gazebo, add the SCARA arm, connect it to ROS 2 and open RViz.

    ros2 launch scara_sim sim.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('scara_sim')
    world = PathJoinSubstitution([pkg, 'worlds', 'drill_cell.sdf'])
    xacro_file = PathJoinSubstitution([pkg, 'urdf', 'scara.urdf.xacro'])
    bridge_config = PathJoinSubstitution([pkg, 'config', 'bridge.yaml'])
    rviz_config = PathJoinSubstitution([pkg, 'rviz', 'scara.rviz'])

    # Run xacro to turn the .xacro file into plain URDF text
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        # 1. Gazebo with our world (-r = start running, not paused)
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
            launch_arguments={'gz_args': ['-r ', world], 'on_exit_shutdown': 'true'}.items(),
        ),

        # 2. Publish the robot model and its frames (TF), from /joint_states
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        ),

        # 3. Put the arm into Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'scara'],
        ),

        # 4. Connect Gazebo topics to ROS 2 (list in config/bridge.yaml)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
        ),

        # 5. RViz to see the arm and its frames
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
        ),
    ])
