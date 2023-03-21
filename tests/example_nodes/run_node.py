#!/usr/bin/env python3

import sys
from importlib.util import module_from_spec, spec_from_file_location
from typing import Type

import rclpy
from rclpy.node import Node


def main(node_class: Type[Node]) -> None:
    rclpy.init()

    node = node_class()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Ignore Ctrl+C
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: run_node.py <file.py> <NodeClassName> [...]")
        sys.exit(1)

    file_path = sys.argv[1]
    node_class_name = sys.argv[2]

    # Import the file
    spec = spec_from_file_location("my_module", file_path)
    assert spec is not None, f"{file_path} is not a valid file."
    my_module = module_from_spec(spec)
    spec.loader.exec_module(my_module)
    node_class = getattr(my_module, node_class_name)

    main(node_class)
