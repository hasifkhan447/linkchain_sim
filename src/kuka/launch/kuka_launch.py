from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, SetEnvironmentVariable, RegisterEventHandler, TimerAction
from launch_ros.substitutions import FindPackageShare 

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import xacro
import os

# Need to get it to source the gazebo_ros_control workspace




def generate_launch_description():
    pkg_path = get_package_share_directory('kuka')

    gz_sim_path = os.path.join(os.getcwd(), "install", "gz_ros2_control", "lib") + ":"
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.path.join(os.getcwd(), "src") + ":"
    os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] = gz_sim_path
    print(gz_sim_path)

    xacro_urdf_path = os.path.join(pkg_path, 'urdf', 'kr120r2500pro.xacro')
    robot_desc = xacro.process_file(xacro_urdf_path).toprettyxml(indent='  ')
    print(robot_desc)





    urdf_path = os.path.join(pkg_path, 'urdf', 'kr120r2500pro.urdf')
    sdf_path = os.path.join(pkg_path, 'urdf', 'kr120r2500pro.sdf')

    rviz_config_path = os.path.join(pkg_path, 'rviz', 'kuka.rviz')
    worlds_path = os.path.join(pkg_path, 'worlds', 'empty.sdf')

    moveit_pkg_path = get_package_share_directory('kuka_moveit')

    srdf_path = os.path.join(moveit_pkg_path, 'config', 'kuka_kr120r2500pro.srdf')

    bridge_params = os.path.join(pkg_path, 'config', 'gz_bridge.yaml')
    
    with open(urdf_path, 'w') as f:
        f.write(robot_desc)


    with open(srdf_path, 'r') as srdf_file:
        robot_srdf = srdf_file.read()

        
    print(robot_desc)





    kuka_control = os.path.join(
        FindPackageShare("kuka").find("kuka"),
        "config",
        "controllers.yaml"
    )


    

    with open(sdf_path, 'r') as infp:
        sdf_desc = infp.read()


    return LaunchDescription([


        # Start Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '1', '-r', worlds_path, '--gui'],
            output='screen'
        ),

        # Spawn the robot using the SDF file directly
        ExecuteProcess(
            cmd=[
                'gz', 'service', 
                '-s', '/world/empty/create', 
                '--reqtype', 'gz.msgs.EntityFactory', 
                '--reptype', 'gz.msgs.Boolean', 
                '--timeout', '1000', 
                '--req', f'sdf_filename: "{urdf_path}" name: "urdf_model"'
            ],
            output='screen'
        ),

        
        # Don't need this since we'll get this from gazebo
        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     parameters=[
        #         {'use_sim_time': True},  # Enable sim time
        #         {'use_gui': True}
        #         ],  # Optional: Enable GUI
        # ),



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
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher",
            output="log",
            arguments=["--x 0.0 --y 0.0 --z 0.0 --roll 0.0 --pitch 0.0 --yaw  0.0 --frame-id base_link "],
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
            


        # I don't think I need this one
        # TimerAction(
        #     period=4.0,  # Delay of 2 seconds
        #     actions=[
        #         Node(
        #             package="controller_manager",
        #             executable="ros2_control_node",
        #             parameters=[
        #                 kuka_control,
        #                 {'use_sim_time': True}  # Enable use of simulated time
        #             ],
        #             remappings=[
        #                 ("/controller_manager/robot_description", "/robot_description"),
        #             ],
        #             output="screen"
        #         ),
        #     ]
        # ),



        # TimerAction(
        #     period=8.0,  # Delay of 2 seconds
        #     actions=[

        #         IncludeLaunchDescription(
        #             PythonLaunchDescriptionSource(
        #                 os.path.join(get_package_share_directory('kuka_moveit'), 'launch', 'my-demo.launch.py')
        #             )
        #         ),
        #     ]
        # ),






    ])
