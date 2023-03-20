# Standard library
import unittest

# Testing
from ros2_easy_test import ROS2TestEnvironment, with_single_node

# Module under test and interfaces
from .example_nodes.well_behaved import EchoNode
from std_msgs.msg import String

# Hypothesis
from hypothesis import given, settings
from hypothesis.strategies import text, characters


@with_single_node(EchoNode, watch_topics={"/mouth": String})
@given(
    some_message=text(
        alphabet=characters(blacklist_categories=("Cs",), min_codepoint=1)
    )
)
@settings(max_examples=10)  # Remember that these tests are costly
def test_on_same_node(env: ROS2TestEnvironment, some_message: str) -> None:
    """This creates a single node and tests it with Hypothesis against many values."""

    env.publish("/ear", String(data=some_message))
    response: String = env.assert_message_published("/mouth").data
    assert response == some_message, (response, some_message)


@given(
    some_message=text(
        alphabet=characters(blacklist_categories=("Cs",), min_codepoint=1)
    )
)
@settings(max_examples=10)  # Remember that these tests are costly
@with_single_node(EchoNode, watch_topics={"/mouth": String})
def test_on_new_node_each(env: ROS2TestEnvironment, some_message: str) -> None:
    """This creates a single node and tests it with Hypothesis against many values."""

    env.publish("/ear", String(data=some_message))
    response: String = env.assert_message_published("/mouth").data
    assert response == some_message, (response, some_message)



if __name__ == "__main__":
    unittest.main()
