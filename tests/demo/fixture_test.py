# Standard library
import unittest

# Testing
from ros2_easy_test import ROS2TestEnvironment, with_single_node
import pytest

# Module under test and interfaces
from ..example_nodes.well_behaved import EchoNode
from std_msgs.msg import String


@pytest.fixture
def the_message() -> str:
    return "The message which should get echoed back!"


@pytest.mark.xfail(
    raises=TypeError,
    reason="This is currently not supported",  # TODO: Support this
    strict=True,
)
@with_single_node(EchoNode, watch_topics={"/mouth": String})
def test_fixture_in_beginning(the_message: str, env: ROS2TestEnvironment) -> None:
    """A demo for combining a pytest fixture and ros2_easy_test."""
    env.publish("/ear", String(data=the_message))
    response: str = env.assert_message_published("/mouth").data
    assert response == the_message, (response, the_message)


@pytest.mark.xfail(
    raises=TypeError,
    reason="This is currently not supported",  # TODO: Support this
    strict=True,
)
@with_single_node(EchoNode, watch_topics={"/mouth": String})
def test_fixture_as_last_argument(env: ROS2TestEnvironment, the_message: str) -> None:
    """The same as :func:`~test_fixture_in_beginning`, but with a different parameter order."""
    env.publish("/ear", String(data=the_message))
    response: str = env.assert_message_published("/mouth").data
    assert response == the_message, (response, the_message)


if __name__ == "__main__":
    unittest.main()
