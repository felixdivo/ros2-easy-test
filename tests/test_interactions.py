"""Tests the base functionality of the library."""

# Standard library
from abc import ABC
import unittest
from unittest import TestCase
from typing import List
from pathlib import Path

# Testing
from pytest import mark
from ros2_easy_test import ROS2TestEnvironment, with_single_node, with_launch_file

# Module under test and interfaces
from .example_nodes.well_behaved import Talker, EchoNode, AddTwoIntsServer
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts


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
        # Set up the service
        adder = env.create_client(AddTwoInts, "add_two_ints")
        self.assertTrue(adder.wait_for_service(timeout_sec=5))

        # Set up the request
        request = AddTwoInts.Request(a=-500, b=73)
        desired_sum = -500 + 73

        # Call the service synchronously
        result: int = adder.call(request).sum
        self.assertEqual(result, desired_sum)

        # Call the service asynchronously
        future = adder.call_async(request)
        env.executor.spin_until_future_complete(future, timeout_sec=2)
        self.assertIsNone(future.exception())
        result: int = future.result().sum
        self.assertEqual(result, desired_sum)

    def test_multiple_messages(self, env: ROS2TestEnvironment, count: int = 5) -> None:
        for identifier in range(count):
            env.publish("/ear", String(data=f"Hi #{identifier}"))

        # This text also tests the message order, which is relevant!
        all_messages: List[String] = env.assert_messages_published(
            "/mouth",
            number=count,
            max_total_timeout=2,
        )
        expected = [String(data=f"Hi #{identifier}") for identifier in range(count)]
        self.assertListEqual(all_messages, expected)


class TestSingleNode(SharedTestCases, TestCase):
    """This test case uses the ``with_single_node`` decorator to set up the test environment."""

    @with_single_node(Talker, watch_topics={"/chatter": String})
    def test_publisher(self, env: ROS2TestEnvironment) -> None:
        super().test_publisher(env)

    @with_single_node(
        Talker, watch_topics={"/chatter": String}, parameters={"/start_value": -42}
    )
    def test_parameter_set(self, env: ROS2TestEnvironment) -> None:
        response: String = env.assert_message_published("/chatter")
        self.assertEqual(response.data, f"Hello World: -42")

    @with_single_node(EchoNode, watch_topics={"/mouth": String})
    def test_subscriber_and_publisher(self, env: ROS2TestEnvironment) -> None:
        super().test_subscriber_and_publisher(env)

    @with_single_node(AddTwoIntsServer)
    def test_service(self, env: ROS2TestEnvironment) -> None:
        super().test_service(env)

    @with_single_node(EchoNode, watch_topics={"/mouth": String})
    def test_multiple_messages(self, env: ROS2TestEnvironment) -> None:
        super().test_multiple_messages(env)


class TestLaunchFile(SharedTestCases, TestCase):
    """This test case uses the ``with_launch_file`` decorator to set up the test environment."""

    BASE = Path(__file__).parent / "example_launch_files"

    @mark.xfail(
        raises=AssertionError,
        reason="a missing file should bring down the test case",
        strict=True,
    )
    @with_launch_file(BASE / "definitely_missing_2783468.launch.py")
    def test_missing_test_file(self, env: ROS2TestEnvironment) -> None:
        pass

    @mark.xfail(
        raises=AssertionError,
        reason="a non-launch file should bring down the test case",
        strict=True,
    )
    @with_launch_file(Path(__file__).parent.parent / "README.md")
    def test_non_launch_file(self, env: ROS2TestEnvironment) -> None:
        pass

    @with_launch_file(BASE / "talker.py", watch_topics={"/chatter": String})
    def test_publisher(self, env: ROS2TestEnvironment) -> None:
        super().test_publisher(env)

    @with_launch_file(BASE / "talker.yaml", watch_topics={"/chatter": String})
    def test_publisher_yaml(self, env: ROS2TestEnvironment) -> None:
        super().test_publisher(env)  # Should work just like the normal test

    @with_launch_file(
        BASE / "talker.py", watch_topics={"/chatter": String}, debug_launch_file=True
    )
    def test_debugging(self, env: ROS2TestEnvironment) -> None:
        super().test_publisher(env)  # Should work just like the normal test

    @with_launch_file(
        BASE / "talker.yaml", watch_topics={"/chatter": String}, debug_launch_file=True
    )
    def test_debugging_yaml(self, env: ROS2TestEnvironment) -> None:
        super().test_publisher(env)  # Should work just like the normal test

    @with_launch_file(BASE / "echo.yaml", watch_topics={"/mouth": String})
    def test_subscriber_and_publisher(self, env: ROS2TestEnvironment) -> None:
        super().test_subscriber_and_publisher(env)

    @with_launch_file(BASE / "adder.yaml", warmup_time=2)
    def test_service(self, env: ROS2TestEnvironment) -> None:
        super().test_service(env)

    @with_launch_file(BASE / "echo.yaml", watch_topics={"/mouth": String})
    def test_multiple_messages(self, env: ROS2TestEnvironment) -> None:
        super().test_multiple_messages(env)

    @mark.xfail(
        raises=AssertionError,
        reason="an assertion error should propagate to the test case",
        strict=True,
    )
    @with_launch_file(BASE / "talker.py", warmup_time=0)
    def test_assertion_raised(self, _: ROS2TestEnvironment) -> None:
        self.fail("This should fail the test case")


if __name__ == "__main__":
    unittest.main()
