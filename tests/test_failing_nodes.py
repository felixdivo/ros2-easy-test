"""Tests how the library handles failing nodes (e.g. exceptions in ``__init__()``, in calllbacks, etc.)."""

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
from .example_nodes.failing import (
    OnPurposeFail,
    NodeRaiseInInit,
    NodeRaiseInTimer,
    NodeRaiseOnRequest,
)
from std_msgs.msg import String, Empty
from example_interfaces.srv import AddTwoInts


class TestSingleNode(TestCase):
    """This test case uses the ``with_single_node`` decorator to set up the test environment."""

    @mark.xfail(
        raises=OnPurposeFail,
        reason="Failures in the node's __init__ method should be propagated to the test.",
        strict=True,
    )
    @with_single_node(NodeRaiseInInit)
    def test_failing_node_init(self, _: ROS2TestEnvironment) -> None:
        pass

    @mark.xfail(
        raises=OnPurposeFail,  # Not AssertionError, since that is probably secondary
        reason="Failures in the node's __init__ method should be propagated to the test (even if the test fails, since thats likely the reason for the failure).",
        strict=True,
    )
    @with_single_node(NodeRaiseInInit)
    def test_failing_node_init_and_failing_test(self, _: ROS2TestEnvironment) -> None:
        self.fail("This will also raise an exception")

    @mark.xfail(
        raises=Exception,
        reason="Failures in a timer callback should be propagated to the test.",
        strict=False,  # TODO: It would be nice to fix this some time -> make it strict=True
    )
    @with_single_node(NodeRaiseInTimer, watch_topics={"/nothing/is/here": Empty})
    def test_failing_node_timer(self, env: ROS2TestEnvironment) -> None:
        # Give the node some time to fail
        env.assert_no_message_published("/nothing/is/here")

    @mark.xfail(
        raises=Exception,
        reason="Failures in a subscriber callback should be propagated to the test.",
        strict=False,  # TODO: It would be nice to fix this some time -> make it strict=True
    )
    @with_single_node(NodeRaiseOnRequest, watch_topics={"/mouth": String})
    def test_node_subscriber_callback_failure(self, env: ROS2TestEnvironment) -> None:
        env.publish("/ear", String(data="Hello! You won't fail because of me, right?"))
        env.assert_no_message_published("/mouth")

    @mark.xfail(
        raises=Exception,
        reason="Failures in a service callback should be propagated to the test.",
        strict=False,  # TODO: It would be nice to fix this some time -> make it strict=True
        run=False,  # TODO: Don't even run this. If deadlocks, and @mark.timeout(2) won't help
    )
    @with_single_node(NodeRaiseOnRequest)
    def test_node_service_callback_failure_synchronously(
        self, env: ROS2TestEnvironment
    ) -> None:
        # Set up the service and request
        adder = env.create_client(AddTwoInts, "add_two_ints")
        self.assertTrue(adder.wait_for_service(timeout_sec=5))  # This should be quick

        # Call the service synchronously
        adder.call(AddTwoInts.Request(a=-500, b=73))

        self.fail("This should never be reached")

    @mark.xfail(
        raises=Exception,
        reason="Failures in a service callback should be propagated to the test.",
        strict=False,  # TODO: It would be nice to fix this some time -> make it strict=True
        # At least this one doesn't deadlock, but it still doesn't work
    )
    @with_single_node(NodeRaiseOnRequest)
    def test_node_service_callback_failure_asynchronously(
        self, env: ROS2TestEnvironment
    ) -> None:
        # Set up the service and request
        adder = env.create_client(AddTwoInts, "add_two_ints")
        self.assertTrue(adder.wait_for_service(timeout_sec=5))  # This should be quick

        # Call the service asynchronously
        future = adder.call_async(AddTwoInts.Request(a=-500, b=73))
        env.executor.spin_until_future_complete(future, timeout_sec=0.5)

        # Inspect the result
        self.assertIsNone(future.exception())  # This should not raise an exception
        self.assertFalse(future.done())  # This should never be done


if __name__ == "__main__":
    unittest.main()
