# Standard library
import unittest
from time import sleep
from unittest import TestCase

# Testing
from pytest import mark
from std_msgs.msg import Empty, String

from ros2_easy_test import ROS2TestEnvironment, with_single_node

# Module under test and interfaces
from .example_nodes.well_behaved import EchoNode, Talker


class TestSingleNodesForEnvCoverage(TestCase):
    """Makes sure that the ``ROS2TestEnvironment`` class is covered."""

    @mark.xfail(raises=AssertionError, reason="the chatter always talks", strict=True)
    @with_single_node(Talker, watch_topics={"/chatter": String})
    def test_failing_assert_no_message_published(
        self, env: ROS2TestEnvironment
    ) -> None:
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

    @mark.xfail(raises=AssertionError, reason="no query - no response", strict=True)
    @with_single_node(EchoNode, watch_topics={"/mouth": String})
    def test_mailbox_clearing(self, env: ROS2TestEnvironment) -> None:
        # If we publish and clear the mailbox, we should not get a response
        env.publish("/ear", String(data="Guude"))
        sleep(0.5)  # Make sure that the response is definitely published to /mouth

        env.clear_messages("/mouth")
        env.assert_message_published("/mouth", timeout=0)  # Should raise

    @mark.xfail(
        raises=Exception,  # TODO: Maybe this should be a more specific exception?
        reason="specifiying a wrong message type is a common mistake and shall fail loudly",
        strict=True,
    )
    @with_single_node(EchoNode, watch_topics={"/mouth": Empty})
    def test_wrong_topic_type(self, _: ROS2TestEnvironment) -> None:
        pass


if __name__ == "__main__":
    unittest.main()
