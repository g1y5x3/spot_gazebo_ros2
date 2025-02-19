import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterFile

from launch_ros.actions import Node


def generate_launch_description():
    world_file = LaunchConfiguration('world_file', default='simple_tunnel_02.sdf')
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value='simple_tunnel_02.sdf',
        description='Name of the world file to load'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false', 
        description='Open RViz.'
    )

    # Setup to launch the simulator and Gazebo world
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_spot_gazebo = get_package_share_directory('spot_gazebo')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': [
                    PathJoinSubstitution([
                        pkg_spot_gazebo, 
                        'worlds',
                        world_file
                    ]),
                    ' --render-engine ogre2'  # doesn't seem to improve simulation speed
                    # ' -v 4'   # show debug messages for gazebo
                ],
            }.items(),
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    pkg_spot_bringup = get_package_share_directory('spot_bringup')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(pkg_spot_bringup, 'config', 'spot_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }]
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    pkg_spot_description = get_package_share_directory('spot_description')
    sdf_file = os.path.join(pkg_spot_description, 'models', 'spot', 'model.sdf')
    with open(sdf_file, 'r') as infp: robot_desc = infp.read()
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
            {"publish_frequency": 200.0},
        ]
    )

    # Controller
    config_path = get_package_share_directory("champ_config")
    links_config = PathJoinSubstitution([config_path, 'config', 'links', 'links.yaml'])
    links_param = ParameterFile(param_file=links_config, allow_substs=True)

    joints_config = PathJoinSubstitution([config_path, 'config', 'joints', 'joints.yaml'])
    joints_param = ParameterFile(param_file=joints_config, allow_substs=True) 

    gait_config = PathJoinSubstitution([config_path, 'config', 'gait', 'gait.yaml'])
    gait_param = ParameterFile(param_file=gait_config, allow_substs=True) 

    urdf_file = os.path.join(pkg_spot_description, 'models', 'spot', 'model.urdf')

    quadruped_controller_node = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"gazebo": True},
            {"publish_joint_states": False},
            {"publish_foot_contacts": False},
            {"publish_joint_control": True},
            {"joint_controller_topic": "/spot/joint_trajectory"},
            {"urdf": urdf_file},
            links_param,
            joints_param,
            gait_param
        ],
        remappings=[("/cmd_vel/smooth", "/cmd_vel")],
    )

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_spot_bringup, 'config', 'spot.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        world_file_arg,
        rviz_arg,
        gz_sim,
        bridge,
        robot_state_publisher,
        quadruped_controller_node,
        rviz
    ])
