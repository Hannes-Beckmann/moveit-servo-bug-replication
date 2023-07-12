import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from moveit_configs_utils import MoveItConfigsBuilder

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("yu", package_name="yu_bringup")
        .robot_description(file_path="config/yu_gripper.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )
    print(moveit_config.package_path)

    planning_pipelines_config = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["pilz", "ompl"],
        "pilz": {
            "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
            "request_adapters": "",
            "start_state_max_bounds_error": 0.1,
        },
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        },
    }
    ompl_planning_yaml = load_yaml("yu_bringup", "config/ompl_planning.yaml")
    planning_pipelines_config["ompl"].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        "yu_bringup", "config/moveit_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            planning_pipelines_config,
            trajectory_execution,
            planning_scene_monitor_parameters,
        ],
    )

    # Get parameters for the Servo node
    servo_yaml = load_yaml("tracking", "config/yu_simulated_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # RViz
    rviz_config_file = (
        get_package_share_directory("tracking") + "/config/demo_rviz_config.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("yu_bringup"),
        "config",
        "ros2_controllers.yaml",
    )

    print(load_yaml("yu_bringup", "config/ros2_controllers.yaml"))

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    yu_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["yu_arm_controller", "-c", "/controller_manager"],
    )

    yu_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["yu_position_controller", "--inactive", "-c", "/controller_manager"],
    )

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Assuming ROS2 intraprocess communications works well, this is a more efficient way.
            # ComposableNode(
            #     package="moveit_servo",
            #     plugin="moveit_servo::ServoServer",
            #     name="servo_server",
            #     parameters=[
            #         servo_params,
            #         moveit_config.robot_description,
            #         moveit_config.robot_description_semantic,
            #     ],
            # ),
            # ComposableNode(
            #     package="robot_state_publisher",
            #     plugin="robot_state_publisher::RobotStatePublisher",
            #     name="robot_state_publisher",
            #     parameters=[moveit_config.robot_description],
            # ),
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{"child_frame_id": "/base_link", "frame_id": "/world"}],
            )
        ],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description, {"publish_frequency": 100.0}],
    )
    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC

    low_pass_filter_coeff = {"butterworth_filter_coeff": 1.0}
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            low_pass_filter_coeff,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    move_robot_node = Node(package="tracking", executable="move_fanuc")

    ethercat_node = Node(
        package="ethercat_node",
        executable="ethercat_yu_test",
        name="ethercat_node",
        parameters=[{"mock_hardware": True}, {"interface": "enp3s0"}],
    )

    return LaunchDescription(
        [
            run_move_group_node,
            ethercat_node,
            rviz_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            yu_arm_controller_spawner,
            servo_node,
            container,
            robot_state_publisher,
            yu_position_controller_spawner,
            # move_robot_node,
        ]
    )
