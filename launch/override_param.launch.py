#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, "../config/config.param.yaml")

    node = Node(
        package="override_param",
        executable="main",
        name="node",
        parameters=[
            {
                "before1": "launch",
                # "before2": "launch",
            },
            config_path,
            {
                "after1": "launch",
                # "after2": "launch",
            },
        ],
    )
    # -> param,param, launch, launch

    node_ns = Node(
        package="override_param",
        executable="main",
        name="node",
        namespace="ns",
        parameters=[
            {
                "before1": "launch",
                # "before2": "launch",
            },
            config_path,
            {
                "after1": "launch",
                # "after2": "launch",
            },
        ],
    )
    # -> param, param, param, param

    return LaunchDescription([node, node_ns])
