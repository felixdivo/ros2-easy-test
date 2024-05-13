"""Tests the base functionality of the library."""

# Standard library
import unittest
from abc import ABC
from typing import List
from unittest import TestCase

# Other ROS2 interfaces
from example_interfaces.action import Fibonacci
from example_interfaces.srv import AddTwoInts

# Testing
from pytest import mark
from std_msgs.msg import String

# What we are testing
from ros2_easy_test import ROS2TestEnvironment, with_launch_file, with_single_node

# Helpers
from . import LAUNCH_FILES

# Module under test and interfaces
from .example_nodes.minimal_action_server import MinimalActionServer
from .example_nodes.well_behaved import AddTwoIntsServer, EchoNode, Talker


class SharedTestCases(ABC):
    """This is shared between the different setup variants: ``with_single_node`` & ``with_launch_file``.

    Makes sure that the basic functionality of the library works as intended.
    """

    def test_publisher(self, env: ROS2TestEnvironment) -> None:
        response: String = env.assert_message_published("/chatter")
        self.assertEqual(response.data, "Hello World: 0")

    def test_subscriber_and_publisher(self, env: ROS2TestEnvironment) -> None:
        # There sould be silence in the beginning:
        env.assert_no_message_published("/mouth")

        # Use publish and assert_message_published to test the node
        env.publish("/ear", String(data="Guude"))
        response: String = env.assert_message_published("/mouth")
        self.assertEqual(response.data, "Guude")

        # Do the same with env.listen_for_messages
        env.publish("/ear", String(data="Guude"))
        all_messages: List[String] = env.listen_for_messages("/mouth")
        self.assertSequenceEqual(all_messages, [String(data="Guude")])

    def test_service(self, env: ROS2TestEnvironment) -> None:
        # Set up the request
        request = AddTwoInts.Request(a=-500, b=73)
        desired_sum = -500 + 73

        # Call the service asynchronously
        result: int = env.call_service("add_two_ints", request).sum
        self.assertEqual(result, desired_sum)

    def test_service_manual(self, env: ROS2TestEnvironment) -> None:
        # Set up the service
        adder = env.create_client(AddTwoInts, "add_two_ints")
        self.assertTrue(adder.wait_for_service(timeout_sec=5))

        # Set up the request
        request = AddTwoInts.Request(a=-500, b=73)
        desired_sum = -500 + 73

        # Call the service synchronously
        result_sync: int = adder.call(request).sum
        self.assertEqual(result_sync, desired_sum)

        # Call the service asynchronously with a helper
        future = adder.call_async(request)
        result_async_auto: int = env.await_future(future, timeout=2).sum
        self.assertEqual(result_async_auto, desired_sum)

        # Call the service asynchronously completely manually
        future = adder.call_async(request)
        # This needs to be done via the executor of the node, see ROS2TestEnvironment::await_future
        env.executor.spin_until_future_complete(future, timeout_sec=2)
        self.assertIsNone(future.exception())
        result_async: int = future.result().sum
        self.assertEqual(result_async, desired_sum)

    def test_multiple_messages(self, env: ROS2TestEnvironment, count: int = 5) -> None:
        for identifier in range(count):
            env.publish("/ear", String(data=f"Hi #{identifier}"))

        # This text also tests the message order, which *is* relevant!
        all_messages: List[String] = env.assert_messages_published(
            "/mouth",
            number=count,
            max_total_timeout=5 + count * 0.05,
        )
        expected = [String(data=f"Hi #{identifier}") for identifier in range(count)]
        self.assertListEqual(all_messages, expected)

    def test_calling_an_action(self, env: ROS2TestEnvironment) -> None:
        feedbacks, result = env.send_action_goal_and_wait_for_result(
            name="fibonacci", goal=Fibonacci.Goal(order=4)
        )

        assert len(feedbacks) == 3
        assert feedbacks == [
            Fibonacci.Feedback(sequence=[0, 1, 1]),
            Fibonacci.Feedback(sequence=[0, 1, 1, 2]),
            Fibonacci.Feedback(sequence=[0, 1, 1, 2, 3]),
        ]

        assert result == Fibonacci.Result(sequence=[0, 1, 1, 2, 3])

        # We call it again to test if resources are reused properly
        feedbacks2, result2 = env.send_action_goal_and_wait_for_result(
            name="fibonacci", goal=Fibonacci.Goal(order=3)
        )
        assert feedbacks2 == feedbacks[:2]
        assert result2 == Fibonacci.Result(sequence=[0, 1, 1, 2])


class TestSingleNode(SharedTestCases, TestCase):
    """This test case uses the ``with_single_node`` decorator to set up the test environment."""

    @with_single_node(Talker, watch_topics={"/chatter": String})
    def test_publisher(self, env: ROS2TestEnvironment) -> None:
        super().test_publisher(env)

    @with_single_node(Talker, watch_topics={"/chatter": String}, parameters={"/start_value": -42})
    def test_parameter_set(self, env: ROS2TestEnvironment) -> None:
        response: String = env.assert_message_published("/chatter")
        self.assertEqual(response.data, "Hello World: -42")

    @with_single_node(EchoNode, watch_topics={"/mouth": String})
    def test_subscriber_and_publisher(self, env: ROS2TestEnvironment) -> None:
        super().test_subscriber_and_publisher(env)

    @with_single_node(AddTwoIntsServer)
    def test_service(self, env: ROS2TestEnvironment) -> None:
        super().test_service(env)

    @with_single_node(AddTwoIntsServer)
    def test_service_manual(self, env: ROS2TestEnvironment) -> None:
        super().test_service_manual(env)

    @with_single_node(EchoNode, watch_topics={"/mouth": String})
    def test_multiple_messages(self, env: ROS2TestEnvironment) -> None:
        super().test_multiple_messages(env)

    @with_single_node(EchoNode, watch_topics={"/mouth": String})
    def test_multiple_messages_stress_test(self, env: ROS2TestEnvironment) -> None:
        super().test_multiple_messages(env, count=1000)

    @mark.xfail(
        raises=AssertionError,
        reason="an assertion error should propagate to the test case",
        strict=True,
    )
    @with_single_node(EchoNode)
    def test_assertion_raised(self, env: ROS2TestEnvironment) -> None:
        self.fail("This should fail the test case")

    @with_single_node(MinimalActionServer)
    def test_calling_an_action(self, env: ROS2TestEnvironment) -> None:
        super().test_calling_an_action(env)


class TestLaunchFile(SharedTestCases, TestCase):
    """This test case uses the ``with_launch_file`` decorator to set up the test environment."""

    @with_launch_file(LAUNCH_FILES / "talker.py", watch_topics={"/chatter": String})
    def test_publisher(self, env: ROS2TestEnvironment) -> None:
        super().test_publisher(env)

    @with_launch_file(
        LAUNCH_FILES / "talker_with_params.py",
        watch_topics={"/chatter": String},
        launch_arguments={"start_value": -42},
    )
    def test_parameter_set(self, env: ROS2TestEnvironment) -> None:
        response: String = env.assert_message_published("/chatter")
        self.assertEqual(response.data, "Hello World: -42")

    @with_launch_file(LAUNCH_FILES / "talker.yaml", watch_topics={"/chatter": String})
    def test_publisher_yaml(self, env: ROS2TestEnvironment) -> None:
        super().test_publisher(env)  # Should work just like the normal test

    @with_launch_file(LAUNCH_FILES / "talker.py", watch_topics={"/chatter": String}, debug_launch_file=True)
    def test_debugging(self, env: ROS2TestEnvironment) -> None:
        super().test_publisher(env)  # Should work just like the normal test

    @with_launch_file(LAUNCH_FILES / "talker.yaml", watch_topics={"/chatter": String}, debug_launch_file=True)
    def test_debugging_yaml(self, env: ROS2TestEnvironment) -> None:
        super().test_publisher(env)  # Should work just like the normal test

    @with_launch_file(LAUNCH_FILES / "echo.yaml", watch_topics={"/mouth": String})
    def test_subscriber_and_publisher(self, env: ROS2TestEnvironment) -> None:
        super().test_subscriber_and_publisher(env)

    @with_launch_file(LAUNCH_FILES / "adder.yaml", warmup_time=2)
    def test_service(self, env: ROS2TestEnvironment) -> None:
        super().test_service(env)

    @with_launch_file(LAUNCH_FILES / "adder.yaml", warmup_time=2)
    def test_service_manual(self, env: ROS2TestEnvironment) -> None:
        super().test_service_manual(env)

    @with_launch_file(LAUNCH_FILES / "echo.yaml", watch_topics={"/mouth": String})
    def test_multiple_messages(self, env: ROS2TestEnvironment) -> None:
        super().test_multiple_messages(env)

    @with_launch_file(LAUNCH_FILES / "echo.yaml", watch_topics={"/mouth": String})
    def test_multiple_messages_stress_test(self, env: ROS2TestEnvironment) -> None:
        super().test_multiple_messages(env, count=1000)

    @mark.xfail(
        raises=AssertionError,
        reason="an assertion error should propagate to the test case",
        strict=True,
    )
    @with_launch_file(LAUNCH_FILES / "talker.yaml", warmup_time=2)
    def test_assertion_raised(self, env: ROS2TestEnvironment) -> None:
        self.fail("This should fail the test case")

    @with_launch_file(LAUNCH_FILES / "fibonacci_action.yaml")
    def test_calling_an_action(self, env: ROS2TestEnvironment) -> None:
        super().test_calling_an_action(env)


if __name__ == "__main__":
    unittest.main()
