"""Tests that actions can be checked correctly."""


from action_msgs.msg import GoalStatus
from example_interfaces.action import Fibonacci
from rclpy.action.client import ClientGoalHandle

# What we are testing
from ros2_easy_test import ROS2TestEnvironment, with_single_node

# Module under test and interfaces
from .example_nodes.minimal_action_server_with_context import MinimalActionServerWithContext


@with_single_node(MinimalActionServerWithContext)
def test_fibonacci_action(env: ROS2TestEnvironment) -> None:
    """Test action."""

    goal_handle, feedbacks, result_response = env.send_action_goal_and_wait_for_response(
        name="fibonacci", goal_msg=Fibonacci.Goal(order=4)
    )
    assert isinstance(goal_handle, ClientGoalHandle)
    assert goal_handle.accepted is True
    assert goal_handle.status == GoalStatus.STATUS_SUCCEEDED

    assert isinstance(feedbacks, list)
    assert len(feedbacks) == 3
    assert feedbacks == [
        Fibonacci.Feedback(sequence=[0, 1, 1]),
        Fibonacci.Feedback(sequence=[0, 1, 1, 2]),
        Fibonacci.Feedback(sequence=[0, 1, 1, 2, 3]),
    ]

    assert result_response.status == GoalStatus.STATUS_SUCCEEDED
    assert result_response.result == Fibonacci.Result(sequence=[0, 1, 1, 2, 3])
