from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_dir = Path(get_package_share_directory("ec_description"))
    doc = xacro.process_file(str(description_dir / "urdf/hunter130.xacro"))
    robot_description = {"robot_description": doc.toxml()}

    share = Path(get_package_share_directory("ec_controller"))
    common = share / "config/controllers.yaml"
    policy = share / "config/policy"
    mimic_config = policy / "mimic.yaml"
    with mimic_config.open() as stream:
        default_motion = yaml.safe_load(stream)["mimic_controller"]["ros__parameters"]["motion_path"]

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, common, {
            "mimic_controller.node_options_args": [
                "--ros-args", "-p", ["mimic_controller:motion_path:=", LaunchConfiguration("motion")],
            ],
        }],
        output="both",
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )
    switcher = Node(
        package="ec_controller",
        executable="controller_switcher_node",
        name="controller_switcher",
        parameters=[common],
        output="screen",
    )

    def spawner(name, config, *, inactive=False):
        args = [name, "--controller-manager", "controller_manager", "--param-file", str(config)]
        if inactive:
            args.append("--inactive")
        return Node(
            package="controller_manager", executable="spawner", arguments=args,
            output="screen",
        )

    spawners = [
        spawner("joint_state_broadcaster", common),
        spawner("default_controller", policy / "default.yaml"),
        spawner("stand_controller", policy / "stand.yaml", inactive=True),
        spawner("walk_controller", policy / "amp.yaml", inactive=True),
        spawner("mimic_controller", mimic_config, inactive=True),
        spawner("quest_controller", mimic_config, inactive=True),
    ]
    handlers = [
        RegisterEventHandler(OnProcessStart(target_action=control_node, on_start=[spawners[0]]))
    ]
    for current, following in zip(spawners, [*spawners[1:], switcher]):
        handlers.append(RegisterEventHandler(OnProcessExit(target_action=current, on_exit=[following])))

    return LaunchDescription([
        DeclareLaunchArgument("motion", default_value=str(share / default_motion),
                              description="CSV motion for the prerecorded Mimic controller (CH6)"),
        *handlers,
        control_node,
        state_publisher,
    ])
