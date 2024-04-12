"""This module contains the decorators to be applied to test functions.

Warning:
    If the node crashes due to an exception in service callback, a test calling the service will deadlock
    indefinitely. In that case, exiting with ``Ctrl + C`` will probably print the exception from the node.
"""

# Standard library
from functools import wraps
from inspect import signature
from pathlib import Path
from signal import SIGINT
from subprocess import Popen, TimeoutExpired
from time import sleep

# Typing
from typing import Any, Callable, Coroutine, Dict, Optional, Type, TypeVar, Union

# ROS
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import InvalidHandle, Node
from rclpy.parameter import Parameter

# Environment
from .env import ROS2TestEnvironment

# Helpers
from .launch_file import LaunchFileProvider

# Function manipulation
from makefun import remove_signature_parameters  # isort: skip


__all__ = ["with_launch_file", "with_single_node"]


# From python 3.10+ on, we should make these typing.TypeAlias'es
# Currently not possible to type the following any better
_TestReturnType = TypeVar("_TestReturnType", None, Coroutine[Any, Any, None])
TestFunctionBefore = Callable[..., _TestReturnType]
# The same but taking one kwarg less (the env), which we can't statically type
TestFunctionAfter = Callable[..., _TestReturnType]


#: The time to give a node for a successful shutdown.
_DEFAULT_SHUTDOWN_TIMEOUT: float = 2


def with_single_node(
    node_class: Type[Node],
    *,
    parameters: Optional[Dict[str, Any]] = None,
    time_limit: Optional[float] = 60,
    shutdown_timeout=_DEFAULT_SHUTDOWN_TIMEOUT,
    **kwargs,
) -> Callable[[TestFunctionBefore], TestFunctionAfter]:
    """Marks a test case that shall be wrapped by a ROS2 context and be given an environment to interact.

    This function is not fundamentally restricted to one node, but more are simply not implemented as of
    now. See :func:`~with_launch_file` for more complex scenarios.

    Note:
        Your test function *must* accept the environment as a keyword-parameter called ``env``,
        e.g. ``def test(env: ROS2TestEnvironment)``.

    Note:
        Make sure that the initializer of ``node_class`` passes along any keyword arguments to the super
        class :class:`rclpy.node.Node`.
        To do this, have the ``__init__`` of your node accept ``**kwargs`` as the last argument and call
        the super class constructor with these, e.g.
        ``super().__init__("node name", **kwargs)``.

    Args:
        node_class:
            Class of the node to instantiate. Assumed to accept no extra parameters besides **any** keyword
            arguments that are passed along to :class:`rclpy.node.Node`.
        parameters: The parameters to be set for the node as ``("key", value)`` pairs
        time_limit:
            The time in seconds to give the test case to complete.
            This only applies to the test function, not the node setup etc.
            If it takes longer than this, the test will fail.
            Set to ``None`` to disable the timeout.
        shutdown_timeout:
            The time to give a node for a successful shutdown. If it takes longer than this,
            the test will fail.
        kwargs: Passed to the :class:`ros2_easy_test.env.ROS2TestEnvironment`

    See Also:
        :func:`~with_launch_file`
    """

    def decorator(test_function: TestFunctionBefore) -> TestFunctionAfter:
        @wraps(test_function)  # Copies the docstring and other metadata
        def wrapper(*args_inner, **kwargs_inner) -> None:
            context = Context()
            try:
                context.init()

                # Package the given parameters up in the ROS2 appropriate format
                parameters_tuples = parameters or {}
                ros_parameters = [
                    Parameter(name=name, value=value) for name, value in parameters_tuples.items()
                ]

                node: Node
                if ros_parameters:
                    node = node_class(  # type: ignore[call-arg]
                        parameter_overrides=ros_parameters, context=context
                    )
                else:
                    node = node_class(context=context)  # type: ignore[call-arg]

                # We need at least two threads: One to spin() and one to execute the test case (as the latter
                # blocks). We better provide 4, since more nodes/tasks might get spun up by some tests and
                # threads are rather cheap.
                executor = MultiThreadedExecutor(num_threads=4, context=context)

                try:
                    # The node should be executed
                    assert executor.add_node(node), "Failed to add node under test."

                    # Also, the environment needs to execute (e.g. to capture messages in the background)
                    environment = ROS2TestEnvironment(context=context, **kwargs)
                    assert executor.add_node(environment), "Failed to add environment."

                    # Finally, we want to launch the actual test and wait for it to complete (indefinitely)
                    test_function_task = executor.create_task(
                        test_function, *args_inner, env=environment, **kwargs_inner
                    )
                    # The type ignore is needed due to a bug in ROS2 Humble+
                    executor.spin_until_future_complete(
                        test_function_task, timeout_sec=time_limit  # type: ignore[arg-type]
                    )

                    test_function_exception = test_function_task.exception()
                    if not test_function_task.done():
                        # Then test_function_exception can only be None, so we can overwrite it
                        test_function_exception = TimeoutError(
                            f"The test case did not complete in time ({time_limit} seconds). "
                            "Consider passing a higher time_limit or setting it to None to disable it."
                        )
                        # Then, make sure to kill the test function as cleanup
                        test_function_task.cancel()

                finally:
                    for running_node in executor.get_nodes():
                        running_node.destroy_node()

                    has_finished = executor.shutdown(shutdown_timeout)
                    assert has_finished, f"Executor shutdown did not complete in {shutdown_timeout} seconds."

                # Make sure that the executor and the nodes are cleaned up/freed afterwards.
                # Cleanup is critical for correctness since subsequent tests may NEVER reference old
                # resources! That would (and did) cause very strange bugs.
                # These are sanity checks and should never fail.
                try:
                    node.get_name()
                except InvalidHandle:
                    pass  # This is what we expect to happen
                else:  # pragma: no cover
                    raise Exception("Node did not properly shut down after test.")

                try:
                    environment.get_name()
                except InvalidHandle:
                    pass  # This is what we expect to happen
                else:  # pragma: no cover
                    raise Exception("The Environment did not properly shut down after test.")

                assert not executor.get_nodes(), "The executor still holds some nodes."

                # Raise the exception that the test case might have raised, e.g. due to asserts
                if test_function_exception is not None:
                    raise test_function_exception from None

                # TODO: We should also raise exceptions that are raised in callbacks, but that's tricky
                # because executor.spin_until_future_complete will not raise them.
                # Currently, we can just point the users to look at the console for a
                # "The following exception was never retrieved: [...]" message.

            finally:
                context.try_shutdown()

            # Make sure that the context is freed afterwards. This sanity check should never fail.
            assert not context.ok(), "Context did not properly shut down after test."

        # This is required to make pytest fixtures work
        # In principle, we could make the parameter name easily adjustable,
        # but that is probably a rare use case
        wrapper.__signature__ = remove_signature_parameters(  # type: ignore[attr-defined]
            signature(test_function), "env"
        )
        return wrapper

    return decorator


def with_launch_file(  # noqa: C901
    launch_file: Union[Path, str],
    *,
    debug_launch_file: bool = False,
    warmup_time: float = 2,
    time_limit: Optional[float] = 60,
    shutdown_timeout=_DEFAULT_SHUTDOWN_TIMEOUT,
    **kwargs,
) -> Callable[[TestFunctionBefore], TestFunctionAfter]:
    """Marks a test case that shall be wrapped by a ROS2 context and be given an environment to interact.

    Note:
        Your test function *must* accept the environment as a keyword-parameter called ``env``,
        e.g. ``def my_test(env: ROS2TestEnvironment): ...``.

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
            independently of how long the test waits.
            The default should suffice on most computers,
            it is rather conservative and higher numbers will slow down each test case even more.
        time_limit:
            The time in seconds to give the test case to complete.
            This only applies to the test function, not the node setup etc.
            If it takes longer than this, the test will fail.
            Set to ``None`` to disable the timeout.
        shutdown_timeout:
            The time to give a node for a successful shutdown. If it takes longer than this,
            the test will fail.
            It applies individually to both shutting down the environment and the launch process.
        kwargs: Passed to the :class:`ros2_easy_test.env.ROS2TestEnvironment`

    See Also:
        :func:`~with_single_node`
    """

    assert warmup_time >= 0, f"Warmup_time must be zero or larger but was {warmup_time}."

    def decorator(test_function: TestFunctionBefore) -> TestFunctionAfter:
        @wraps(test_function)  # Copies the docstring and other metadata
        def wrapper(*args_inner, **kwargs_inner) -> None:
            # Provide the launch file
            with LaunchFileProvider(launch_file) as launch_file_path:
                # Prepare the "ros2 launch" child process
                # It inherits stdout and stderr from parent, so logging reaches the (pytest) console
                ros2_debug_parameters = ["--debug"] if debug_launch_file else []
                ros2_parameters = [
                    "ros2",
                    "launch",
                    str(launch_file_path),
                    "--noninteractive",
                    *ros2_debug_parameters,
                ]

                context = Context()
                try:
                    context.init()

                    # We need at least two threads: One to spin() and one to execute the test case
                    # (as the latter blocks). We better provide 4, since more nodes/tasks might get spun up
                    # by some tests and threads are rather cheap.
                    executor = MultiThreadedExecutor(num_threads=4, context=context)

                    try:
                        # We first start the environment, such that any topics that are watched can be
                        # captured right from the start
                        environment = ROS2TestEnvironment(context=context, **kwargs)
                        assert executor.add_node(environment), "failed to add environment"

                        # We should not need any warmup time here, as the environment is fully ready once the
                        # node class (the environment) is instantiated.
                        # TODO: However, this warmup and cleaning seems to suppress some rather rare errors.
                        executor.spin_until_future_complete(executor.create_task(sleep, 2))
                        environment.clear_messages()

                        # Now, we are ready to start the system under test using "ros2 launch"
                        ros2_process = Popen(ros2_parameters)

                        # Give the launch process time to start up. Otherwise, the timeouts on the first
                        # test asserts will be off and the system wil generally behave strangely.
                        # TODO: Ususally, this shouldn't need to be this high. Reducing it would be awesome.
                        executor.spin_until_future_complete(executor.create_task(sleep, warmup_time))

                        test_function_task = executor.create_task(
                            test_function, *args_inner, env=environment, **kwargs_inner
                        )
                        # The type ignore is needed due to a bug in ROS2 Humble+
                        executor.spin_until_future_complete(
                            test_function_task, timeout_sec=time_limit  # type: ignore[arg-type]
                        )

                        test_function_exception = test_function_task.exception()
                        if not test_function_task.done():
                            # Then test_function_exception can only be None, so we can overwrite it
                            test_function_exception = TimeoutError(
                                f"The test case did not complete in time ({time_limit} seconds). "
                                "Consider passing a higher time_limit or setting it to None to disable it."
                            )
                            # Then, make sure to kill the test function as cleanup
                            test_function_task.cancel()

                    finally:
                        # This should only kill the environment, no other node is registered
                        for node in executor.get_nodes():
                            node.destroy_node()

                        has_finished = executor.shutdown(shutdown_timeout)
                        assert (
                            has_finished
                        ), f"Executor shutdown did not complete in {shutdown_timeout} seconds."

                    # Make sure that the executor and the node are cleaned up/freed afterwards.
                    # Cleanup is critical for correctness since subsequent tests may NEVER reference old
                    # resources! That would (and did) cause very strange bugs.
                    # These are sanity checks and should never fail.
                    try:
                        environment.get_name()
                    except InvalidHandle:
                        pass
                    else:  # pragma: no cover
                        raise Exception("The Environment did not properly shut down after test.")

                    assert not executor.get_nodes(), "The executor still holds some nodes."

                finally:
                    context.try_shutdown()

                    # Make sure that the context is freed afterwards. This sanity check should never fail.
                    assert not context.ok(), "Context did not properly shut down after test."

                    try:
                        ros2_process
                    except NameError:
                        pass  # Maybe, the launch process was not even started
                    else:
                        # Signal the child launch process to finish
                        # This is much like pressing Ctrl+C on the console
                        ros2_process.send_signal(SIGINT)
                        try:
                            # Might raise a TimeoutExpired if it takes too long
                            return_code = ros2_process.wait(timeout=shutdown_timeout / 2)
                        except TimeoutExpired:  # pragma: no cover
                            ros2_process.terminate()
                            # return_code will be larger than 130
                            return_code = ros2_process.wait(timeout=shutdown_timeout / 2)

                # Both SUCCESS (0) or the result code of SIGINT (130) are acceptable
                return_code_problematic = return_code not in {0, 130}

                if return_code_problematic:
                    if test_function_exception is None:
                        raise AssertionError(
                            f"The ROS launch process FAILED with exit code {return_code} "
                            "(please inspect stdout and stderr) BUT the test case SUCCEEDED."
                        ) from None

                    raise AssertionError(
                        f"The ROS launch process FAILED with exit code {return_code} "
                        "(please inspect stdout and stderr) AND the test FAILED with "
                        f'exception: "{test_function_exception}".'
                    ) from test_function_exception

                if test_function_exception is not None:
                    raise test_function_exception from None

        # This is required to make pytest fixtures work
        # In principle, we could make the parameter name easily adjustable,
        # but that is probably a rare use case
        wrapper.__signature__ = remove_signature_parameters(  # type: ignore[attr-defined]
            signature(test_function), "env"
        )
        return wrapper

    return decorator
