from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    OpaqueFunction,
)
from launch.substitutions import (
    LaunchConfiguration,
    FindExecutable,
    Command,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
import random
from math import pi, sqrt

CUR_PKG_NAME = "ugv_bringup"
DESC_PKG_NAME = "ugv_descriptions"
MAP_LEN = 20.0
GRID_LEN = 4.0
CUBE_LEN = 1.0


def create_spawn_box_nodes() -> list[tuple[float, float, float]]:
    grid_num = int(MAP_LEN / GRID_LEN)
    cube_dig = CUBE_LEN * sqrt(2.0)
    cube_radius = cube_dig / 2.0
    offset = MAP_LEN / -2.0 + cube_radius
    region = GRID_LEN - cube_dig
    obstacels = []
    for i in range(grid_num):
        for j in range(grid_num):
            # 中心不放障碍物
            if grid_num // 2 == 0 and i == j:
                continue
            if (
                grid_num // 2 != 0
                and abs(i - (grid_num - 1) / 2.0) < 1.0
                and abs(j - (grid_num - 1) / 2.0) < 1.0
            ):
                continue

            # 随机放置障碍物
            obstacels.append(
                (
                    offset + i * GRID_LEN + random.uniform(0, region),
                    offset + j * GRID_LEN + random.uniform(0, region),
                    random.uniform(-pi, pi),
                )
            )
    return obstacels


def spawn_boxes(context, *args, **kwargs):
    seed = int(LaunchConfiguration("world_seed").perform(context))
    random.seed(seed)
    box_sdf_file = PathJoinSubstitution(
        [
            FindPackageShare(CUR_PKG_NAME),
            "models/box.sdf",
        ]
    )
    return [
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-file",
                box_sdf_file,
                "-name",
                "box_" + str(i),
                "-x",
                str(pose[0]),
                "-y",
                str(pose[1]),
                "-z",
                "0.5",
                "-Y",
                str(pose[2]),
            ],
        )
        for i, pose in enumerate(create_spawn_box_nodes())
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "world_seed",
            default_value=str(random.randint(int(1e9), int(1e10))),
            description="Name of the model to view.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "model_name",
            default_value="",
            description="Name of the model to view.",
        )
    )

    model_base_dir = PathJoinSubstitution(
        [
            FindPackageShare(DESC_PKG_NAME),
            "models",
            LaunchConfiguration("model_name"),
        ]
    )

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [
                    model_base_dir,
                    "urdf/all.xacro",
                ]
            ),
            " use_gazebo:=True",
        ]
    )

    gz_bdg_config_file = PathJoinSubstitution([model_base_dir, "gz_bdg.yml"])

    odom_config_file = PathJoinSubstitution(
        [FindPackageShare(CUR_PKG_NAME), "launch/odom.yml"]
    )

    # 发布机器人描述
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content}],
    )

    # gazebo
    node_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": [
                "-r ",
                PathJoinSubstitution(
                    [
                        FindPackageShare(CUR_PKG_NAME),
                        "world/fence.sdf",
                    ]
                ),
            ]
        }.items(),
    )

    node_spawn_vehicle = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            LaunchConfiguration("model_name"),
            "-z",
            "0.5",
        ],
    )

    node_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": gz_bdg_config_file,
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    controller_name = "controller"
    node_spawn_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            controller_name,
            "--controller-manager",
            "/controller_manager",
        ],
    )

    node_odom = Node(
        package="robot_localization",
        executable="ekf_node",
        name="odom_filter_node",
        output="screen",
        parameters=[
            odom_config_file,
            {"use_sim_time": True},
        ],
    )

    node_frame_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
        parameters=[
            {
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
    )

    nodes_spawn_boxes = RegisterEventHandler(
        OnProcessExit(
            target_action=node_spawn_vehicle,
            on_exit=OpaqueFunction(function=spawn_boxes),
        )
    )

    nodes = [
        LogInfo(msg=["Current World Seed: ", LaunchConfiguration("world_seed")]),
        LogInfo(msg=[odom_config_file]),
        node_robot_state_publisher,
        node_gazebo,
        node_spawn_vehicle,
        node_gz_bridge,
        node_spawn_controller,
        node_odom,
        node_frame_map,
        nodes_spawn_boxes,
    ]

    return LaunchDescription(declared_arguments + nodes)
