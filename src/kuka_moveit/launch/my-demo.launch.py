import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    default_rviz_config = os.path.join(get_package_share_directory('kuka_moveit'), 'config/moveit.rviz')


    # Command-line arguments
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config,
        description="RViz configuration file",
    )


    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="mock_components",
        description="ROS 2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",
    )

    moveit_config = ( #TODO: Need to figure out how this works exactly

        MoveItConfigsBuilder("kuka_kr120r2500pro", package_name="kuka_moveit")
        .robot_description(
            file_path="config/kuka_kr120r2500pro.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )
            },
        )
        .robot_description_semantic(file_path="config/kuka_kr120r2500pro.srdf")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"]
        )
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True}  # <-- ADD THIS LINE

            ],
        # arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("kuka_moveit"), "launch", rviz_base]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    # # Static TF
    # static_tf_node = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_publisher",
    #     output="log",
    #     arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "map", "base_link"],
    # )

    # # Publish TF
    # robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name="robot_state_publisher",
    #     output="both",
    #     parameters=[moveit_config.robot_description, {'use_sim_time': True}  # Enable sim time
    #                 ],
    # )

    # ros2_control using FakeSystem as hardware

    print(f"share directory{get_package_share_directory("kuka_moveit")}")

    ros2_controllers_path = os.path.join(
        get_package_share_directory("kuka_moveit"),
        "config",
        "ros2_controllers.yaml",
    ),

    print(f"My ros2 controllers path {ros2_controllers_path}")
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            ros2_controllers_path,
            {'use_sim_time': True}  # Enable use of simulated time
            ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        # arguments=["--ros-args", "--log-level", "controller_manager:=debug"],

        output="screen",
    )



    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )


    arm_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["Arm_controller", "--controller-manager", "/controller_manager"
                    #    , "--ros-args", "--log-level", "debug" 
                        
                        ],
            output="screen",
        )


    # panda_arm_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["panda_arm_controller", "-c", "/controller_manager"],
    # )

    # panda_hand_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["panda_hand_controller", "-c", "/controller_manager"],
    # )

    # # Warehouse mongodb server
    # db_config = LaunchConfiguration("db")
    # mongodb_server_node = Node(
    #     package="warehouse_ros_mongo",
    #     executable="mongo_wrapper_ros.py",
    #     parameters=[
    #         {"warehouse_port": 33829},
    #         {"warehouse_host": "localhost"},
    #         {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
    #     ],
    #     output="screen",
    #     condition=IfCondition(db_config),
    # )

    return LaunchDescription(
        [
            rviz_config_arg,
            ros2_control_hardware_type,
            rviz_node,
            # static_tf_node,
            # robot_state_publisher,
            move_group_node,
            ros2_control_node,
            # joint_state_broadcaster_spawner,
            arm_controller_spawner
        ]
    )