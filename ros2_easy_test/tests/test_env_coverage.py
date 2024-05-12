# Standard library
import unittest
from time import sleep
from unittest import TestCase

# Testing
from pytest import mark
from std_msgs.msg import Empty, String

# What we are testing
from ros2_easy_test import ROS2TestEnvironment, with_launch_file, with_single_node

# Helpers
from . import LAUNCH_FILES

# Module under test and interfaces
from .example_nodes.well_behaved import EchoNode, Talker


@with_single_node(EchoNode, watch_topics={"/not_existing": Empty})
def test_plain_function_single_node(env: ROS2TestEnvironment) -> None:
    env.assert_no_message_published("/not_existing", time_span=0.5)


@with_launch_file(LAUNCH_FILES / "echo.yaml", watch_topics={"/not_existing": Empty})
def test_plain_function_launch_file(env: ROS2TestEnvironment) -> None:
    env.assert_no_message_published("/not_existing", time_span=0.5)


class TestSingleNodesForEnvCoverage(TestCase):
    """Makes sure that the ``ROS2TestEnvironment`` class is covered."""

    @mark.xfail(raises=AssertionError, reason="the chatter always talks", strict=True)
    @with_single_node(Talker, watch_topics={"/chatter": String})
    def test_failing_assert_no_message_published(self, env: ROS2TestEnvironment) -> None:
        env.assert_no_message_published("/chatter", time_span=2)  # Should raise

    @mark.xfail(raises=AssertionError, reason="no query - no response", strict=True)
    @with_single_node(EchoNode, watch_topics={"/mouth": String})
    def test_failing_assert_message_published(self, env: ROS2TestEnvironment) -> None:
        env.assert_message_published("/mouth", timeout=0.5)  # Should raise

    @mark.xfail(raises=AssertionError, reason="no query - no response", strict=True)
    @with_single_node(EchoNode, watch_topics={"/mouth": String})
    def test_failing_assert_messages_published(self, env: ROS2TestEnvironment) -> None:
        env.publish("/ear", String(data="Guude"))
        # Should raise because we only expect one message and not two
        env.assert_messages_published(
            "/mouth",
            number=2,
            individual_timeout=None,
            max_total_timeout=0.5,
        )

    @mark.xfail(
        raises=AssertionError, reason="after clearing the topic, no message should be ther", strict=True
    )
    @with_single_node(EchoNode, watch_topics={"/mouth": String})
    def test_mailbox_clearing(self, env: ROS2TestEnvironment) -> None:
        # If we publish and clear the mailbox, we should not get a response
        env.publish("/ear", String(data="Guude"))
        sleep(0.5)  # Make sure that the response is definitely published to /mouth

        env.clear_messages("/mouth")
        env.assert_message_published("/mouth", timeout=0)  # Should raise

    @mark.xfail(
        raises=AssertionError, reason="after clearing all topics, no message should be ther", strict=True
    )
    @with_single_node(EchoNode, watch_topics={"/mouth": String})
    def test_mailbox_clearing_all(self, env: ROS2TestEnvironment) -> None:
        """Almost the same as :func:`~test_mailbox_clearing`."""
        # If we publish and clear the mailbox, we should not get a response
        env.publish("/ear", String(data="Guude"))
        sleep(0.5)  # Make sure that the response is definitely published to /mouth

        env.clear_messages()  # All messages
        env.assert_message_published("/mouth", timeout=0)  # Should raise

    @mark.xfail(raises=Exception, reason="invalid call should raise", strict=True)
    @with_single_node(EchoNode)
    def test_mailbox_clearing_missing_topic(self, env: ROS2TestEnvironment) -> None:
        env.clear_messages(topic="/that/does/not/exist")

    @with_single_node(EchoNode)
    def test_mailbox_clearing_no_topics(self, env: ROS2TestEnvironment) -> None:
        env.clear_messages()  # This should work just fine (and do nothing)

    # It would be nice to have some more specific Exception type,
    # but it raises the private rclpy._rclpy_pybind11.RCLError
    @mark.xfail(
        raises=Exception,
        reason="specifiying a wrong message type is a common mistake and shall fail loudly",
        strict=True,
    )
    @with_single_node(EchoNode, watch_topics={"/mouth": Empty})
    def test_wrong_topic_type(self, env: ROS2TestEnvironment) -> None:
        pass


if __name__ == "__main__":
    unittest.main()
