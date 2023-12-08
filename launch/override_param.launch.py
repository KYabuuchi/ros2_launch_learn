#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, "../config/config.param.yaml")

    node1 = Node(
        package="override_param",
        executable="main",
        name="node1",
        parameters=[
            {
                "before1": "launch",
                "before2": "launch",
            },
            config_path,
            {
                "after1": "launch",
                "after2": "launch",
            },
        ],
    )

    node2 = Node(
        package="override_param",
        executable="main",
        name="node2",
        namespace="ns",
        parameters=[
            {
                "before1": "launch",
                "before2": "launch",
            },
            config_path,
            {
                "after1": "launch",
                "after2": "launch",
            },
        ],
    )

    node3 = Node(
        package="override_param",
        executable="main",
        name="node3",
        namespace="ns",
        parameters=[
            config_path,
            {
                "after1": "launch",
                "after2": "launch",
            },
        ],
    )

    return LaunchDescription([node1, node2, node3])
