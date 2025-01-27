"""This module provides the environment by which tests can interact with nodes."""

# Standard library
from collections import defaultdict
from importlib import import_module
from queue import Empty, SimpleQueue
from threading import Event, RLock
from time import monotonic, sleep

# Typing
from typing import Any, Dict, List, Mapping, Optional, Tuple, Type, cast

# ROS
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import CallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.client import Client, SrvTypeRequest, SrvTypeResponse
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSHistoryPolicy, QoSProfile
from rclpy.task import Future

RosMessage = Any  # We can't be more specific for now

__all__ = ["ROS2TestEnvironment"]

#: The normal timeout for asserts, like waiting for messages. This has to be surprisingly high.
_DEFAULT_TIMEOUT: Optional[float] = 2


class ROS2TestEnvironment(Node):
    """This class provides a way to interact with nodes and assert that events happen or do not.

    **Publishing** messages works by creating a publishers on the fly.
    However, one must ensure that the type of the message that is published stays the same across each test.

    For **subscribers**, the topics and their types must be declared at the instantiation via the
    ``watch_topics`` argument. This cannot be avoided due to how *rclpy* works.
    Then, a mailbox is created for each topic and incoming messages are collected each (without a limit).
    This is performed in the background, meaning that for example :meth:`~assert_message_published` may return
    a message that was collected before the method was called.
    To prevent this from happening, one might use :meth:`~clear_messages`
    (although the concurrency is often the desired behaviour).
    Also, mind the ``timeout`` / ``time_span`` arguments as they determine how long to wait while checking the
    assertion (default: ``1`` second).
    Asserting that a message is sent or failing to assert that no message is sent will return early.

    **Services** are handled similarly to publishers, being created on the fly when used for the first time.

    Note:
        The :class:`ROS2TestEnvironment` is a :class:`rclpy.node.Node` too, which allows to implement custom
        functionality too.

    Note:
        Timings and timeouts in this class are only approximate.

    Args:
        watch_topics: The topics (and their type) to watch for incoming messages.
            You may pass ``None`` to not watch any topic.
        qos_profile: The topics and their QoS profile to subscribe to or publish on.
            You may pass ``None`` to use the default profile.
            All topics without an explicit profile use ``QoSProfile(history=QoSHistoryPolicy.KEEP_ALL)``.
    """

    def __init__(
        self,
        *,
        watch_topics: Optional[Mapping[str, Type]] = None,
        qos_profiles: Optional[Mapping[str, QoSProfile]] = None,
        **kwargs,
    ) -> None:
        super().__init__("_testing_ROS2TestEnvironment", **kwargs)

        # Collects the messages that were received
        # (Simple)Queues are synchronized, but we need the lock to synchronize access to the mailbox dict
        self._subscriber_mailboxes_lock = RLock()
        self._subscriber_mailboxes: Dict[str, SimpleQueue[RosMessage]] = {}

        self._qos_profiles = defaultdict(
            lambda: QoSProfile(history=QoSHistoryPolicy.KEEP_ALL), qos_profiles or {}
        )

        # Set up the subscribers, which place the messages in the mailboxes
        for topic, message_type in (watch_topics or {}).items():

            def callback(message: RosMessage, arriving_topic=topic) -> None:
                self._get_mailbox_for(arriving_topic).put(message)  # potentially blocks

            # This might run concurrently with mailboxes registered previously
            with self._subscriber_mailboxes_lock:
                self._subscriber_mailboxes[topic] = SimpleQueue()

            # After the mailboxes are set up, we can start receiving messages
            self.create_subscription(message_type, topic, callback, self._qos_profiles[topic])

        # Prepare to collect publishers; these are set up on the fly when used the first time in `publish()`
        self._registered_publishers_lock = RLock()
        self._registered_publishers: Dict[str, Publisher] = {}

        # Similar for services
        self._registered_services_lock = RLock()
        self._registered_services: Dict[str, Client] = {}

        # Similar for actions
        self._registered_actions_lock = RLock()
        self._registered_actions: Dict[str, ActionClient] = {}
        # A MutuallyExclusive CallbackGroup is required to force all the callbacks: goal_response, feedback
        # and goal_done to run sequentially. This relieves us from some synchronization burden.
        self._action_cb_group: CallbackGroup = MutuallyExclusiveCallbackGroup()

    def _get_mailbox_for(self, topic: str) -> "SimpleQueue[RosMessage]":
        """Checks that a topic is watched for and returns the appropriate mailbox. Handles synchronization.

        Args:
            topic: The topic to check and fetch the mailbox of

        Returns:
            The mailbox queue of the arrived messages for ``topic``

        Raises:
            AssertionError: If ``topic`` is not being watched on
        """

        with self._subscriber_mailboxes_lock:
            assert topic in self._subscriber_mailboxes, (
                f"topic {topic} it not being watched: please specify it in the constructor"
            )

            return self._subscriber_mailboxes[topic]

    def _get_publisher(self, topic: str, msg_type: Type) -> Publisher:
        """Get or else create the correct publisher and cache it."""
        with self._registered_publishers_lock:
            try:
                return self._registered_publishers[topic]
            except KeyError:
                publisher = self.create_publisher(msg_type, topic, self._qos_profiles[topic])
                self._registered_publishers[topic] = publisher
                return publisher

    def publish(self, topic: str, message: RosMessage) -> None:
        """Publish the message on the topic.

        This method creates the publisher internally if required and caches it.
        Nothing needs to be done manually.

        Args:
            topic: The topic to publish on
            message: The message to be sent. It's type must match the one of all other messages sent on this
                ``topic`` before and after.
        """
        self._get_publisher(topic, type(message)).publish(message)

    def assert_no_message_published(self, topic: str, time_span: float = 0.5) -> None:
        """Asserts that no message is published on the given topic within the given time.

        Args:
            topic: The topic to listen on
            time_span:
                The approximate amount of time to listen before returning.
                This value is set to be rather low (when compared to the other timeouts) since in a
                successful test, it may never return early and causes a lot of waiting time.
        """

        try:
            message = self._get_mailbox_for(topic).get(block=True, timeout=time_span)
        except Empty:
            pass  # this is what we expect
        else:
            raise AssertionError(
                f"A message was published on topic {topic} although none was expected: {repr(message)}"
            ) from None

    def assert_message_published(self, topic: str, timeout: Optional[float] = _DEFAULT_TIMEOUT) -> RosMessage:
        """Asserts a message is published on the given topic within after at most the given time.

        This method might return early if a message is received before the timeout as occurred.

        Args:
            topic: The topic to listen on
            timeout: The approximate amount of time to listen before failing. ``None`` means waiting forever.

        Returns:
            The message that was received
        """

        try:
            return self._get_mailbox_for(topic).get(block=True, timeout=timeout)
        except Empty:
            raise AssertionError(
                f"No message was published on topic {topic} after {timeout} seconds"
            ) from None

    def assert_messages_published(
        self,
        topic: str,
        number: int,
        individual_timeout: Optional[float] = _DEFAULT_TIMEOUT,
        max_total_timeout: Optional[float] = 30.0,
    ) -> List[RosMessage]:
        """Asserts that some messages are published on the given topic within after at most the given time.

        This method might return early if ALL messages are received before the timeout as occurred.

        Args:
            topic: The topic to listen on
            number: The expected number of messages to arrive
            individual_timeout:
                The approximate amount of time per message to listen before failing.
                ``None`` means waiting forever (if ``max_total_timeout`` is also set to ``None``).
                The timeout that is used is calculated by ``number * timeout`` but runs for all messages
                together, and NOT for each one individually.
                In other words, it is accepted that when waiting for ``number = 10`` messages for
                ``timeout = 2.0`` seconds (that means the total timeout is ``10 * 2 = 20`` seconds) and then
                all messages arrive after ``19.9`` seconds. If they arrive after ``20.5`` seconds, this method
                will have already raised an ``AssertionError``.
            max_total_timeout:
                This caps the total timeout that is computed from ``number * timeout`` if not set to ``None``.
                This prevents excessively long tests times until a test fails when dealing with many messages.
                Most -- if not all -- tests should in any way not take more time than the defualt of this.

        Returns:
            All messages that were received. It is guaranteed that ``len(result) == number``.
        """

        if individual_timeout is None:
            # We still do not want to exceed the total timeout, so we use it here
            # Note, that it might be None as well, but then it is intentional
            timeout = max_total_timeout
        else:
            timeout = number * individual_timeout
            if max_total_timeout is not None and timeout > max_total_timeout:
                timeout = max_total_timeout

        remaining_time = timeout
        count = 0
        collected_messages: List[RosMessage] = []

        while count < number:
            start = monotonic()
            try:
                collected_messages.append(self.assert_message_published(topic, timeout=remaining_time))
            except AssertionError:
                raise AssertionError(
                    f"Only {count} messages out of {number} expected ones were published on "
                    f"topic {topic} after a total timeout of {timeout} seconds"
                ) from None
            else:
                if remaining_time is not None:
                    remaining_time = max(0.0, remaining_time - (monotonic() - start))
                count += 1

        assert len(collected_messages) == number, "internal counting error"
        return collected_messages

    def listen_for_messages(
        self, topic: str, time_span: Optional[float] = _DEFAULT_TIMEOUT
    ) -> List[RosMessage]:
        """Collects all messages which arrive in the given time on the topic.

        Args:
            topic: The topic to listen on
            time_span: The approximate amount of time to listen before returning.
                If ``None``, this method will immediately return all messages that are currently in the queue.

        Returns:
            A (possibly empty) list of messages that were received
        """

        # First, wait for the given time span
        if time_span is not None:
            sleep(time_span)

        # Then, collect messages that have arrived until the queue is empty
        messages = []
        mailbox = self._get_mailbox_for(topic)
        try:
            while True:
                messages.append(mailbox.get_nowait())

        except Empty:
            pass  # We are done with fetching items

        return messages

    def clear_messages(self, topic: Optional[str] = None) -> None:
        """Clear all messages in the mailbox of the given ``topic`` or in all mailboxes.

        Args:
            topic: The topic to clear the mailbox of, or all topics if ``None``
        """

        if topic is None:
            with self._subscriber_mailboxes_lock:  # This is reentrant
                for topic in self._subscriber_mailboxes.keys():
                    self.clear_messages(topic=topic)

        else:
            # There is no clear() in SimpleQueue
            self.listen_for_messages(topic, time_span=None)  # ignore the result

    def await_future(self, future: Future, timeout: Optional[float] = 10) -> Any:
        """Waits for the given future to complete.

        Args:
            future: The ROS Future to wait for
            timeout: The timeout to wait for the future

        Raises:
            TimeoutError: If the future did not complete within the timeout
            Exception: If the future completed with an exception

        Returns:
            The result of the future
        """

        event = Event()

        def unblock(_: Future) -> None:
            nonlocal event
            event.set()

        # This immediately calls the callback if the future is already done
        future.add_done_callback(unblock)

        if not event.wait(timeout):
            # Timed out. remove_pending_request() to free resources
            self.remove_pending_request(future)
            raise TimeoutError(f"Future did not complete within {timeout} seconds")

        # Potential exception is raised here
        return future.result()

    def _get_service_client(self, name: str, request_class: Type) -> Client:
        """Returns the service client for the given service name."""

        with self._registered_services_lock:
            try:
                return self._registered_services[name]
            except KeyError:
                # Get the type of the service
                # This is a bit tricky but relieves the user from passing it explicitly
                module = import_module(request_class.__module__)
                # We cut away the trailing "_Request" from the type name, which has length 8
                base_type_name = request_class.__name__[:-8]
                base_type_class: Type = getattr(module, base_type_name)

                client = self.create_client(base_type_class, name)
                self._registered_services[name] = client
                return client

    def call_service(
        self,
        name: str,
        request: SrvTypeRequest,
        timeout_availability: Optional[float] = 1,
        timeout_call: Optional[float] = 10,
    ) -> SrvTypeResponse:  # type: ignore[type-var]
        """Calls the given service with the given request once available and returns the response.

        The service type if inferred automatically from the request type.

        Args:
            name: The name of the service
            request: The request to send to the service
            timeout_availability: The timeout to wait for the service to be available
            timeout_call: The timeout to wait for the service to respond

        Returns:
            The response of the service

        Raises:
            TimeoutError: If the service did not respond within the timeout
        """
        client = self._get_service_client(name, type(request))
        # The type ignore is needed due to a bug in ROS2 Humble+
        is_ready = client.wait_for_service(timeout_availability)  # type: ignore[arg-type]
        if not is_ready:
            raise TimeoutError(f"Service {name} did not become ready within {timeout_availability} seconds")

        future = client.call_async(request)
        return cast(SrvTypeResponse, self.await_future(future, timeout=timeout_call))

    def _get_action_client(self, name: str, goal_class: Type) -> ActionClient:
        """Returns the action client for the given action class."""

        with self._registered_actions_lock:
            try:
                return self._registered_actions[name]
            except KeyError:
                # This is a bit tricky but relieves the user from passing it explicitly
                module = import_module(goal_class.__module__)
                # We cut away the trailing "_Goal" from the type name, which has length 5
                base_type_name = goal_class.__name__[:-5]
                base_type: Type = getattr(module, base_type_name)

                client = ActionClient(self, base_type, name, callback_group=self._action_cb_group)
                self._registered_actions[name] = client
                return client

    def send_action_goal(
        self,
        name: str,
        goal: Any,
        collect_feedback: bool = True,
        timeout_availability: Optional[float] = 1,
        timeout_receive_goal: Optional[float] = 1,
    ) -> Tuple[ClientGoalHandle, List[Any]]:
        """Sends the goal to the given action but does not handle the result.

        This does not check if the goal was accepted and does not attempt to retrieve the result.

        Args:
            name: The name of the action
            goal: Goal message to send to the action server.
            collect_feedback: Whether to collect feedbacks asynchronously. Defaults to True.
                This is provided for performance reasons.
            timeout_availability: The timeout to wait for the action to be available. Defaults to 1.
            timeout_receive_goal: The timeout to wait for the action server to accept the goal. Defaults to 1.

        Raises:
            TimeoutError: If the action server does not respond within the specified timeouts.

        Returns:
            Return the goal handle and feedback list (that might still grow asynchronously).

        See Also:
            :meth:`send_action_goal_and_wait_for_result`
        """

        # Create an action client
        client = self._get_action_client(name, type(goal))

        # Wait for the action to be available
        if not client.wait_for_server(timeout_availability):
            raise TimeoutError(f"Action {name} did not become ready within {timeout_availability} seconds")

        # Prepare collecting feedbacks
        feedbacks: List[RosMessage] = []

        def feedback_callback(feedback_msg: RosMessage) -> None:
            feedbacks.append(feedback_msg.feedback)

        # Send the goal and wait for the acceptence
        send_goal_future = client.send_goal_async(
            goal, feedback_callback=feedback_callback if collect_feedback else None
        )
        goal_handle: ClientGoalHandle = self.await_future(send_goal_future, timeout=timeout_receive_goal)

        # Return the results to the test case
        return goal_handle, feedbacks

    def send_action_goal_and_wait_for_result(
        self,
        name: str,
        goal: Any,
        collect_feedback: bool = True,
        timeout_availability: Optional[float] = 1,
        timeout_accept_goal: Optional[float] = 1,
        timeout_get_result: Optional[float] = 10,
    ) -> Tuple[List[Any], Any]:
        """Sends the goal to the given action and returns the response handle, feedbacks and result.

        Args:
            name: The name of the action
            goal: Goal message to send to the action server.
            collect_feedback: Whether to collect feedbacks asynchronously. Defaults to True.
                This is provided for performance reasons.
            timeout_availability: The timeout to wait for the action to be available. Defaults to 1.
            timeout_accept_goal: The timeout to wait for the action server to accept the goal. Defaults to 1.
            timeout_get_result: The timeout to wait for the action server to return the result.
                Defaults to 10.

        Raises:
            TimeoutError: If the action server does not respond within the specified timeouts.
            AssertionError: If the goal was not accepted or the goal did not succeed.

        Returns:
            Return the goal handle, all the feedback responses and the final result.

        See Also:
            :meth:`send_action_goal`
        """

        # Send the goal and wait for the handle to be available
        goal_handle, feedbacks = self.send_action_goal(
            name=name,
            goal=goal,
            collect_feedback=collect_feedback,
            timeout_availability=timeout_availability,
            timeout_receive_goal=timeout_accept_goal,
        )

        # Make sure the goal was accepted
        assert goal_handle.accepted, f"Goal was not accepted: {goal_handle.status=}"

        # Wait for the result
        result = self.await_future(goal_handle.get_result_async(), timeout=timeout_get_result)

        # Make sure the goal was reached successfully
        assert result.status == GoalStatus.STATUS_SUCCEEDED, f"Goal did not succeed: {result.status=}"

        # Return the results to the test case
        return feedbacks, result.result

    def destroy_node(self):
        # Actions don't get destroyed automatically
        with self._registered_actions_lock:
            for client in self._registered_actions.values():
                client.destroy()

        return super().destroy_node()
