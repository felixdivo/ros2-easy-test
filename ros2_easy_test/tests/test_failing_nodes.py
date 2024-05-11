"""Tests how the library handles failing nodes (e.g. exceptions in ``__init__()``, in calllbacks, etc.)."""

# Standard library
import unittest
from abc import ABC
from pathlib import Path
from unittest import TestCase

from example_interfaces.srv import AddTwoInts

# Testing
from pytest import mark
from std_msgs.msg import Empty, String

# What we are testing
from ros2_easy_test import ROS2TestEnvironment, with_launch_file, with_single_node

# Helpers
from . import is_ros_version

# Module under test and interfaces
from .example_nodes.failing import (
    NodeRaiseInInit,
    NodeRaiseInTimer,
    NodeRaiseOnRequest,
    OnPurposeError,
)

BASE = Path(__file__).parent / "example_launch_files"


class SharedTestCases(ABC):
    """This is shared between the different setup variants: ``with_single_node`` & ``with_launch_file``."""

    def test_failing_node_timer(self, env: ROS2TestEnvironment) -> None:
        # Give the node some time to fail
        env.assert_no_message_published("/nothing/is/here")

    def test_subscriber_callback_failure(self, env: ROS2TestEnvironment) -> None:
        env.publish("/ear", String(data="Hello! You won't fail because of me, right?"))
        env.assert_no_message_published("/mouth")

    def test_service_callback_failure_sync(self, env: ROS2TestEnvironment) -> None:
        # Set up the service and request
        adder = env.create_client(AddTwoInts, "add_two_ints")
        self.assertTrue(adder.wait_for_service(timeout_sec=5))  # This should be quick

        # Call the service synchronously
        adder.call(AddTwoInts.Request(a=-500, b=73))

        self.fail("This should never be reached")

    def test_service_callback_failure_async(self, env: ROS2TestEnvironment) -> None:
        # Set up the service and request
        adder = env.create_client(AddTwoInts, "add_two_ints")
        self.assertTrue(adder.wait_for_service(timeout_sec=5))  # This should be quick

        # Call the service asynchronously
        future = adder.call_async(AddTwoInts.Request(a=-500, b=73))
        env.executor.spin_until_future_complete(future, timeout_sec=0.5)

        # Inspect the result
        self.assertIsNone(future.exception())  # This should not raise an exception
        self.assertFalse(future.done())  # This should never be done


class TestSingleNode(SharedTestCases, TestCase):
    """This test case uses the ``with_single_node`` decorator to set up the test environment."""

    @mark.xfail(
        raises=OnPurposeError,
        reason="Failures in the node's __init__ method should be propagated to the test.",
        strict=True,
    )
    @with_single_node(NodeRaiseInInit)
    def test_failing_node_init(self, env: ROS2TestEnvironment) -> None:
        pass

    @mark.xfail(
        raises=OnPurposeError,  # Not AssertionError, since that is probably secondary
        reason=(
            "Failures in the node's __init__ method should be propagated to the test "
            "(even if the test fails, since thats likely the reason for the failure)."
        ),
        strict=True,
    )
    @with_single_node(NodeRaiseInInit)
    def test_failing_node_init_and_failing_test(self, env: ROS2TestEnvironment) -> None:
        self.fail("This will also raise an exception")

    @mark.xfail(
        raises=Exception,
        reason="Failures in a timer callback should be propagated to the test.",
        strict=False,  # TODO: It would be nice to fix this some time -> make it strict=True
    )
    @with_single_node(NodeRaiseInTimer, watch_topics={"/nothing/is/here": Empty})
    def test_failing_node_timer(self, env: ROS2TestEnvironment) -> None:
        super().test_failing_node_timer(env)

    @mark.xfail(
        raises=Exception,
        reason="Failures in a subscriber callback should be propagated to the test.",
        strict=False,  # TODO: It would be nice to fix this some time -> make it strict=True
    )
    @with_single_node(NodeRaiseOnRequest, watch_topics={"/mouth": String})
    def test_subscriber_callback_failure(self, env: ROS2TestEnvironment) -> None:
        super().test_subscriber_callback_failure(env)

    @mark.xfail(
        raises=Exception,
        reason="Failures in a service callback should be propagated to the test.",
        strict=False,  # TODO: It would be nice to fix this some time -> make it strict=True
        run=False,  # TODO: Don't even run this. If deadlocks, and @mark.timeout(2) won't help
    )
    @with_single_node(NodeRaiseOnRequest)
    def test_service_callback_failure_sync(self, env: ROS2TestEnvironment) -> None:
        super().test_service_callback_failure_sync(env)

    @mark.xfail(
        raises=Exception,
        reason="Failures in a service callback should be propagated to the test.",
        strict=False,  # TODO: It would be nice to fix this some time -> make it strict=True
        # At least this one doesn't deadlock, but it still doesn't work
    )
    @with_single_node(NodeRaiseOnRequest)
    def test_service_callback_failure_async(self, env: ROS2TestEnvironment) -> None:
        super().test_service_callback_failure_async(env)


class TestLaunchFile(SharedTestCases, TestCase):
    """This test case uses the ``with_launch_file`` decorator to set up the test environment."""

    @mark.xfail(
        raises=OnPurposeError,
        reason="Failures in the node's __init__ method should be propagated to the test.",
        strict=False,  # TODO: It would be nice to fix this some time -> make it strict=True
    )
    @with_launch_file(BASE / "raise_in_init.yaml")
    def test_failing_node_init(self, env: ROS2TestEnvironment) -> None:
        pass

    @mark.xfail(
        raises=Exception,
        reason=(
            "Failures in the node's __init__ method should be propagated to the test "
            "(even if the test fails, since thats likely the reason for the failure)."
        ),
        strict=True,
    )
    @with_launch_file(BASE / "raise_in_init.yaml")
    def test_failing_node_init_and_failing_test(self, env: ROS2TestEnvironment) -> None:
        self.fail("This will also raise an exception")

    @mark.xfail(
        raises=Exception,
        reason="Failures in a timer callback should be propagated to the test.",
        strict=False,  # TODO: It would be nice to fix this some time -> make it strict=True
    )
    @with_launch_file(BASE / "raise_in_timer.yaml", watch_topics={"/nothing/is/here": Empty})
    def test_failing_node_timer(self, env: ROS2TestEnvironment) -> None:
        super().test_failing_node_timer(env)

    @mark.xfail(
        raises=Exception,
        reason="Failures in a subscriber callback should be propagated to the test.",
        strict=False,  # TODO: It would be nice to fix this some time -> make it strict=True
    )
    @with_launch_file(BASE / "raise_on_request.yaml", watch_topics={"/mouth": String})
    def test_subscriber_callback_failure(self, env: ROS2TestEnvironment) -> None:
        super().test_subscriber_callback_failure(env)

    @mark.xfail(
        raises=Exception,
        reason="Failures in a service callback should be propagated to the test.",
        strict=False,  # TODO: It would be nice to fix this some time -> make it strict=True
        run=False,  # TODO: Don't even run this. If deadlocks, and @mark.timeout(2) won't help
    )
    @with_launch_file(BASE / "raise_on_request.yaml", warmup_time=2)
    def test_service_callback_failure_sync(self, env: ROS2TestEnvironment) -> None:
        super().test_service_callback_failure_sync(env)

    @mark.xfail(
        raises=Exception,
        reason="Failures in a service callback should be propagated to the test.",
        strict=False,  # TODO: It would be nice to fix this some time -> make it strict=True
        # At least this one doesn't deadlock, but it still doesn't work
    )
    @with_launch_file(BASE / "raise_on_request.yaml", warmup_time=2)
    def test_service_callback_failure_async(self, env: ROS2TestEnvironment) -> None:
        super().test_service_callback_failure_async(env)


class TestBadLaunchFiles(TestCase):
    @mark.xfail(
        raises=AssertionError,
        reason="a missing file should bring down the test case",
        strict=True,
    )
    @with_launch_file(BASE / "definitely" / "missing_2783468.launch.py", warmup_time=2)
    def test_missing_launch_file(self, env: ROS2TestEnvironment) -> None:
        pass

    @mark.xfail(
        raises=AssertionError,
        reason="a non-launch file should bring down the test case",
        strict=True,
    )
    @with_launch_file(Path(__file__).parent.parent / "README.md", warmup_time=2)
    def test_non_launch_file(self, env: ROS2TestEnvironment) -> None:
        pass

    @mark.xfail(
        raises=AssertionError,
        reason="a bad launch file and raising test should still fail the test case",
        strict=True,
    )
    @with_launch_file(Path(__file__).parent.parent / "README.md", warmup_time=2)
    def test_assertion_raised_and_launch_failed(self, env: ROS2TestEnvironment) -> None:
        self.fail("This should fail the test case")


if __name__ == "__main__":
    unittest.main()
