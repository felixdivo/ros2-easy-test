# QoS Profile
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

# ROS2 interfaces
from std_msgs.msg import String

# Testing
from ros2_easy_test import ROS2TestEnvironment, with_single_node

# Module under test and interfaces
from ..example_nodes.well_behaved import LatchingNode


@with_single_node(
    LatchingNode,
    watch_topics={"/hello": String},
    qos_profiles={
        "/hello": QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_ALL)
    },
)
def test_plain_method(env: ROS2TestEnvironment) -> None:
    env.assert_message_published("/hello")
