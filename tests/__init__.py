from os import environ


def is_ros_version(what: str) -> bool:
    try:
        return environ["ROS_DISTRO"].lower() == what.lower()
    except KeyError:
        return False
