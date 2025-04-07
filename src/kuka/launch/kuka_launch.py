from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, SetEnvironmentVariable, RegisterEventHandler, TimerAction
from launch_ros.substitutions import FindPackageShare 

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription



import os

# Need to get it to source the gazebo_ros_control workspace




def generate_launch_description():
    pkg_path = get_package_share_directory('kuka')
    urdf_path = os.path.join(pkg_path, 'urdf', 'kr120r2500pro.urdf')
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'kuka.rviz')
    worlds_path = os.path.join(pkg_path, 'worlds', 'empty.sdf')



    os.environ["GZ_SIM_RESOURCE_PATH"] = os.path.join(os.getcwd(), "src") + ":"


    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()


    kuka_control = os.path.join(
        FindPackageShare("kuka").find("kuka"),
        "config",
        "controllers.yaml"
    )

    moveit_pkg_path = get_package_share_directory('kuka_moveit')
    srdf_path = os.path.join(moveit_pkg_path, 'config', 'kuka_kr120r2500pro.srdf')

    with open(srdf_path, 'r') as srdf_file:
        robot_srdf = srdf_file.read()


    print(kuka_control)

    bridge_params = os.path.join(pkg_path, 'config', 'gz_bridge.yaml')


    return LaunchDescription([


        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '1', '-r', worlds_path, '--gui'],
            output='screen'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-name', 'kuka_kr120r2500pro',
                '-topic', 'robot_description',
            ],
            output='screen'
        ),


        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_desc},
                {'robot_description_semantic': robot_srdf},
                {'use_sim_time': True}  # Enable sim time

            ]
        ),


        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': bridge_params}]
        ),


        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_map_base',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
            output='screen'
        ),

        TimerAction(
            period=4.0,  # Delay of 2 seconds
            actions=[
                Node(
                    package="controller_manager",
                    executable="ros2_control_node",
                    parameters=[
                        kuka_control,
                        {'use_sim_time': True}  # Enable use of simulated time
                    ],
                    output="screen"
                ),




            ]
        ),


        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["Arm_controller"],
            output="screen",
        ),

        TimerAction(
            period=8.0,  # Delay of 2 seconds
            actions=[

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(get_package_share_directory('kuka_moveit'), 'launch', 'my-demo.launch.py')
                    )
                ),

                # Node(
                #     package='rviz2',
                #     executable='rviz2',
                #     name='rviz2',
                #     output='screen',
                #     arguments=['-d', os.path.join(
                #         get_package_share_directory('kuka_moveit'),
                #         'launch',
                #         rviz_config_path  # or your RViz config file if named differently
                #     )],
                # ),




            ]
        ),

        # # Node(
        # #     package='moveit_ros_move_group',
        # #     executable='move_group',
        # #     output='screen',
        # #     parameters=[
        # #         {'robot_description': robot_desc},
        # #         {'robot_description_semantic': robot_srdf},
        # #         # Optionally add kinematics.yaml, joint_limits.yaml etc.
        # #     ]
        # # )





    ])
