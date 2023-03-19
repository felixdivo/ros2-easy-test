"""This module contains the decorators to be applied to test functions."""

# Standard library
from signal import SIGINT
from subprocess import Popen
from subprocess import TimeoutExpired
from threading import Thread
from time import sleep
import unittest

# Typing
from typing import Any
from typing import Callable
from typing import Dict
from typing import Optional
from typing import Type
from typing import TypeVar

# ROS
import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import InvalidHandle
from rclpy.node import Node
from rclpy.parameter import Parameter

# Environment
from .env import ROS2TestEnvironment

# Helpers
from .launch_file import LaunchFileProvider


NodeType = TypeVar("NodeType", bound=Node)  # pylint: disable=invalid-name
TestCaseType = TypeVar("TestCaseType", bound=unittest.TestCase)  # pylint: disable=invalid-name
TestFunction = Callable[[TestCaseType, ROS2TestEnvironment], None]


#: The time to give a node for a successful shutdown. It is dependent on whether it runs in CI or not.
SHUTDOWN_TIMEOUT: float = 2


def with_single_node(
    node_class: Type[NodeType], *, parameters: Optional[Dict[str, Any]] = None, **kwargs
) -> Callable[[TestFunction], Callable[[TestCaseType], None]]:
    """Marks a test case that shall be wrapped by a ROS2 context and be given an environment to interact.

    This function is not fundamentally restricted to one node, but more are simply not implemented as of
    now. See :func:`~with_launch_file` for more complex scenarios.

    Warning:
        Make sure that the initializer of ``node_class`` passes along any keyword arguments to the super
        class :class:`rclpy.node.Node`.
        To do this, have the ``__init__`` of your node accept ``**kwargs`` as the last argument and call
        the super class constructor with these, e.g.
        ``super().__init__("node name", automatically_declare_parameters_from_overrides=True, **kwargs)``.

    Args:
        node_class:
            Class of the node to instantiate. Assumed to accept no extra parameters besides **any** keyword
            arguments that are passed along to :class:`rclpy.node.Node`.
        parameters: The parameters to be set for the node as ``("key", value)`` pairs
        kwargs: Passed to the :class:`ros2_easy_test.ROS2TestEnvironment`

    See Also:
        :func:`~with_launch_file`
    """

    def decorator(test_function: TestFunction) -> Callable[[TestCaseType], None]:
        def wrapper(self: TestCaseType) -> None:
            context = Context()
            try:
                rclpy.init(context=context)

                parameters_tuples = parameters or {}
                params = [Parameter(name=name, value=value) for name, value in parameters_tuples.items()]

                node: Node
                if params:
                    node = node_class(parameter_overrides=params, context=context)
                else:
                    node = node_class(context=context)

                # We need at least two threads: One to spin() and one to execute the test case (as the latter
                # blocks). We better provide 4, since more nodes/tasks might get spun up by some tests and
                # threads are rather cheap.
                executor = MultiThreadedExecutor(num_threads=4, context=context)

                try:
                    # The node should be executed
                    assert executor.add_node(node), "failed to add node under test"

                    # Also, the environment needs to execute (e.g. to capture messages in the background)
                    environment = ROS2TestEnvironment(context=context, **kwargs)
                    assert executor.add_node(environment), "failed to add environment"

                    # Finally, we want to launch the actual test and wait for it to complete (indefinitely)
                    test_function_task = executor.create_task(test_function, self, environment)
                    thread = Thread(
                        target=executor.spin_until_future_complete, args=(test_function_task,), daemon=True
                    )
                    thread.start()
                    thread.join()

                finally:
                    for running_node in executor.get_nodes():
                        running_node.destroy_node()

                    has_finished = executor.shutdown(SHUTDOWN_TIMEOUT)
                    assert has_finished, f"Executor shutdown did not complete in {SHUTDOWN_TIMEOUT} seconds"

                # Make sure that the executor and the nodes are cleaned up/freed afterwards.
                # Cleanup is critical for correctness since subsequent tests may NEVER reference old
                # resources! That would (and did) cause very strange bugs.
                # These are sanity checks and should never fail.
                try:
                    node.get_name()
                except InvalidHandle:
                    pass  # This is what we expect to happen
                else:
                    raise Exception("Node did not properly shut down after test")

                try:
                    environment.get_name()
                except InvalidHandle:
                    pass  # This is what we expect to happen
                else:
                    raise Exception("The Environment did not properly shut down after test")

                assert not executor.get_nodes(), "The executor still holds some nodes"

                # Raise the exception that the test case might have raised, e.g. due to asserts
                exception = test_function_task.exception()
                if exception is not None:
                    raise exception from None

                # TODO: We should also raise exceptions that are raised in callbacks, but that's tricky
                # because executor.spin_until_future_complete will not raise them.
                # Currently, we can just point the users to look at the console for a
                # "The following exception was never retrieved: [...]" message.

            finally:
                rclpy.try_shutdown(context=context)

                # Make sure that the context is freed afterwards. This is a sanity check and should never fail.
                assert not context.ok(), "Context did not properly shut down after test"

        return wrapper

    return decorator


def with_launch_file(  # noqa: C901
    launch_file: str, *, debug_launch_file: bool = False, warmup_time: float = 5, **kwargs
) -> Callable[[TestFunction], Callable[[TestCaseType], None]]:
    """Marks a test case that shall be wrapped by a ROS2 context and be given an environment to interact.

    Args:
        launch_file: Either:
            1) The path to the launch file to start for the test.
            2) The literal launch file (must contain a newline to be detected as such).
        debug_launch_file: If set to ``True``, instruct ``ros2 launch`` to be more verbose and run in debug
            mode. It only affects the output on failing tests.
            However, it might also cause sudden failures, therefore the default is ``False``.
        warmup_time:
            The time to sleep while letting the ROS2 system spin up. Must be zero or larger.
            Strange bugs will occur when this value is set too low: No messages can be exchanged,
            independently of how long the test waits. The default should suffice on most computers.
            If run on the CI platform, this value will be doubled.
        kwargs: Passed to the :class:`ros2_easy_test.ROS2TestEnvironment`

    See Also:
        :func:`~with_single_node`
    """

    assert warmup_time >= 0, f"warmup_time must be zero or larger but was {warmup_time}"

    with LaunchFileProvider(launch_file) as launch_file_path:

        def decorator(test_function: TestFunction) -> Callable[[TestCaseType], None]:
            def wrapper(self: TestCaseType) -> None:

                # Inherits stdout and stderr from parent, so logging reaches the console
                additional_params = ["--debug"] if debug_launch_file else []
                process = Popen(  # pylint: disable=consider-using-with
                    ["ros2", "launch", launch_file_path, "--noninteractive", *additional_params]
                )

                context = Context()
                try:
                    rclpy.init(context=context)

                    # We need at least two threads: One to spin() and one to execute the test case
                    # (as the latter blocks). We better provide 4, since more nodes/tasks might get spun up
                    # by some tests and threads are rather cheap.
                    executor = MultiThreadedExecutor(num_threads=4, context=context)

                    try:
                        environment = ROS2TestEnvironment(context=context, **kwargs)
                        assert executor.add_node(environment), "failed to add environment"

                        # Give the launch process time to start up
                        # We do it here such that the environment may spin too, while we wait
                        executor.spin_until_future_complete(executor.create_task(sleep, warmup_time))

                        test_function_task = executor.create_task(test_function, self, environment)

                        thread = Thread(
                            target=executor.spin_until_future_complete,
                            args=(test_function_task,),
                            daemon=True,
                        )
                        thread.start()
                        thread.join()

                    finally:
                        # This should only kill the environment, no other node is registered
                        for node in executor.get_nodes():
                            node.destroy_node()

                        has_finished = executor.shutdown(SHUTDOWN_TIMEOUT)
                        assert (
                            has_finished
                        ), f"Executor shutdown did not complete in {SHUTDOWN_TIMEOUT} seconds"

                    # Make sure that the executor and the node are cleaned up/freed afterwards.
                    # Cleanup is critical for correctness since subsequent tests may NEVER reference old
                    # resources! That would (and did) cause very strange bugs.
                    # These are sanity checks and should never fail.
                    try:
                        environment.get_name()
                    except InvalidHandle:
                        pass
                    else:
                        raise Exception("The Environment did not properly shut down after test")

                    assert not executor.get_nodes(), "The executor still holds some nodes"

                finally:
                    rclpy.try_shutdown(context=context)

                # Make sure that the context is freed afterwards.
                # This is a sanity check and should never fail.
                assert not context.ok(), "Context did not properly shut down after test"

                # Signal the child launch process to finish; This is much like pressing Ctrl+C on the console
                process.send_signal(SIGINT)
                try:
                    # Might raise a TimeoutExpired if it takes too long
                    return_code = process.wait(timeout=SHUTDOWN_TIMEOUT / 2)
                except TimeoutExpired:
                    process.terminate()
                    # return_code will be larger than 130
                    return_code = process.wait(timeout=SHUTDOWN_TIMEOUT / 2)

                # Both SUCCESS (0) or the result code of SIGINT (130) are acceptable
                return_code_problematic = return_code not in {0, 130}
                test_function_exception = test_function_task.exception()

                if return_code_problematic:
                    if test_function_exception is None:
                        raise AssertionError(
                            f"The ROS launch process FAILED with exit code {return_code} "
                            "(please inspect stdout and stderr) BUT the test case SUCCEEDED"
                        )

                    raise AssertionError(
                        f"The ROS launch process FAILED with exit code {return_code} "
                        "(please inspect stdout and stderr) AND the test FAILED with "
                        f'exception: "{test_function_exception}".'
                    ) from test_function_exception

                if test_function_exception is not None:
                    raise test_function_exception from None

            return wrapper

        return decorator
