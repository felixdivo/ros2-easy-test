"""Tests that actions can be checked correctly."""

# ROS2 infrastructure
from example_interfaces.action import Fibonacci

# What we are testing
from ros2_easy_test import ROS2TestEnvironment, with_launch_file, with_single_node

# Module under test and interfaces
from . import LAUNCH_FILES
from .example_nodes.minimal_action_server import MinimalActionServer


@with_single_node(MinimalActionServer)
def test_fibonacci_action_direct(env: ROS2TestEnvironment) -> None:
    """Test calling an action."""

    feedbacks, result = env.send_action_goal_and_wait_for_result(
        name="fibonacci", goal=Fibonacci.Goal(order=4)
    )

    assert len(feedbacks) == 3
    assert feedbacks == [
        Fibonacci.Feedback(sequence=[0, 1, 1]),
        Fibonacci.Feedback(sequence=[0, 1, 1, 2]),
        Fibonacci.Feedback(sequence=[0, 1, 1, 2, 3]),
    ]

    assert result.result == Fibonacci.Result(sequence=[0, 1, 1, 2, 3])


@with_launch_file(LAUNCH_FILES / "fibonacci_action.yaml")
def test_fibonacci_action_launch_file(env: ROS2TestEnvironment) -> None:
    """Test calling an action."""

    feedbacks, result = env.send_action_goal_and_wait_for_result(
        name="fibonacci", goal=Fibonacci.Goal(order=4)
    )

    assert len(feedbacks) == 3
    assert feedbacks == [
        Fibonacci.Feedback(sequence=[0, 1, 1]),
        Fibonacci.Feedback(sequence=[0, 1, 1, 2]),
        Fibonacci.Feedback(sequence=[0, 1, 1, 2, 3]),
    ]

    assert result.result == Fibonacci.Result(sequence=[0, 1, 1, 2, 3])
