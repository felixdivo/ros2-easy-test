from os import environ
from pathlib import Path

LAUNCH_FILES = Path(__file__).parent / "example_launch_files"


def is_ros_version(what: str) -> bool:
    try:
        return environ["ROS_DISTRO"].lower() == what.lower()
    except KeyError:
        return False
