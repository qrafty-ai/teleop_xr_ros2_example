import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # MoveIt Config
    # The MoveItConfigsBuilder will look for moveit_resources_panda_moveit_config
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(
            file_path="config/panda.urdf.xacro",
            mappings={"ros2_control_hardware_type": "mock_components"},
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description, {"use_sim_time": False}],
    )

    # ros2_control node
    # Load controller config from local config directory
    ros2_controllers_path = os.path.join(
        get_package_share_directory("teleop_xr_ros2_example"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path, {"use_sim_time": False}],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    # Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "panda_arm_controller",
            "-c",
            "/controller_manager",
        ],
    )

    panda_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "panda_hand_controller",
            "-c",
            "/controller_manager",
        ],
    )

    # Mock camera publisher for XR view
    mock_camera_publisher_node = Node(
        package="teleop_xr_ros2_example",
        executable="mock_camera_publisher.py",
        name="mock_camera_publisher",
        output="screen",
        parameters=[
            {"publish_rate": 30.0},
            {"image_width": 640},
            {"image_height": 480},
        ],
    )

    # Serialize spawners to avoid race conditions
    # joint_state_broadcaster -> panda_arm_controller -> panda_hand_controller

    start_arm_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[panda_arm_controller_spawner],
        )
    )

    start_hand_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=panda_arm_controller_spawner,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[panda_hand_controller_spawner],
                )
            ],
        )
    )

    # Move Group
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": False}],
    )

    # RViz
    # Use the local teleop_xr_rviz.rviz config
    rviz_config = os.path.join(
        get_package_share_directory("teleop_xr_ros2_example"),
        "config",
        "teleop_xr_rviz.rviz",
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
            {"use_sim_time": False},
        ],
    )

    # Static TF: world -> panda_link0
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
        parameters=[{"use_sim_time": False}],
    )

    # Teleop XR Node
    # Publishes to /joint_trajectory; adapter forwards arm joints to controller
    # no_urdf_topic=False by default, so it will fetch from /robot_description
    # Camera topics configured to match mock_camera_publisher
    teleop_xr_node = ExecuteProcess(
        cmd=[
            "python3",
            "-m",
            "teleop_xr.ros2",
            "--mode",
            "ik",
            "--robot-class",
            "franka",
            "--output-topic",
            "/panda_arm_controller/joint_trajectory",
            "--head-topic",
            "/camera/head/image_raw",
            "--wrist-left-topic",
            "/camera/wrist_left/image_raw",
            "--wrist-right-topic",
            "/camera/wrist_right/image_raw",
            "--extra-streams",
            "workspace",
            "/camera/workspace/image_raw",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            static_tf_node,
            robot_state_publisher,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            start_arm_controller_cmd,
            start_hand_controller_cmd,
            move_group_node,
            rviz_node,
            mock_camera_publisher_node,
            teleop_xr_node,
        ]
    )
