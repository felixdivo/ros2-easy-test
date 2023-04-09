"""This module provides the environment by which tests can interact with nodes."""

# Standard library
from queue import Empty, SimpleQueue
from threading import RLock
from time import sleep, time

# Typing
from typing import Any, Dict, List, Mapping, Optional, Type

# ROS
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSHistoryPolicy, QoSProfile

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
    To prevent this from happening, one might use:meth:`clear_messages` (although the concurrency is often the
    desired behaviour).
    Also, mind the ``timeout`` / ``time_span`` arguments as they determine how long to wait while checking the
    assertion (default: ``1`` second).
    Asserting that a message is sent or failing to assert that no message is sent will return early.

    Note:
        The :class:`ROS2TestEnvironment` is a :class:`rclpy.node.Node` too, which allows to implement custom
        functionality too. In particular, one can simply call services using code similar to:
        ``client = env.create_client(GetLocalTangentPoint, "/service/get_local_tangent_point")``.

    Note:
        Timings and timeouts in this class are only approximate.

    Args:
        watch_topics: The topics (and their type) to watch for incoming messages.
            You may pass ``None`` to not watch any topic.
    """

    def __init__(self, *, watch_topics: Optional[Mapping[str, Type]] = None, **kwargs) -> None:
        super().__init__("_testing_ROS2TestEnvironment", **kwargs)

        # Collects the messages that were received
        # (Simple)Queues are synchronized, but we need the lock to synchronize access to the mailbox dict
        self._subscriber_mailboxes_lock = RLock()
        self._subscriber_mailboxes: Dict[str, SimpleQueue[RosMessage]] = {}

        self._qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_ALL)

        # Set up the subscribers, which place the messages in the mailboxes
        for topic, message_type in (watch_topics or {}).items():

            def callback(message: RosMessage, arriving_topic=topic) -> None:
                self._get_mailbox_for(arriving_topic).put(message)  # potentially blocks

            # This might run concurrently with mailboxes registered previously
            with self._subscriber_mailboxes_lock:
                self._subscriber_mailboxes[topic] = SimpleQueue()

            # After the mailboxes are set up, we can start receiving messages
            self.create_subscription(message_type, topic, callback, self._qos_profile)

        # Prepare to collect publishers; these are set up on the fly when used the first time in `publish()`
        self._registered_publishers_lock = RLock()
        self._registered_publishers: Dict[str, Publisher] = {}

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
            assert (
                topic in self._subscriber_mailboxes
            ), f"topic {topic} it not being watched: please specify it in the constructor"

            return self._subscriber_mailboxes[topic]

    def publish(self, topic: str, message: RosMessage) -> None:
        """Publish the message on the topic.

        This method creates the publisher internally if required and caches it.
        Nothing needs to be done manually.

        Args:
            topic: The topic to publish on
            message: The message to be sent. It's type must match the one of all other messages sent on this
                ``topic`` before and after.
        """

        # Get or else create the correct publisher and cache it
        publisher: Publisher
        with self._registered_publishers_lock:
            try:
                publisher = self._registered_publishers[topic]
            except KeyError:
                publisher = self.create_publisher(type(message), topic, self._qos_profile)
                self._registered_publishers[topic] = publisher

        publisher.publish(message)

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
                f"A message was published on topic {topic} although none was expected: " f"{repr(message)}"
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
            start = time()
            try:
                collected_messages.append(self.assert_message_published(topic, timeout=remaining_time))
            except AssertionError:
                raise AssertionError(
                    f"Only {count} messages out of {number} expected ones were published on "
                    f"topic {topic} after a total timeout of {timeout} seconds"
                ) from None
            else:
                if remaining_time is not None:
                    remaining_time = max(0.0, remaining_time - (time() - start))
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

    def clear_messages(self, topic: str) -> None:
        """Clear all messages in the mailbox of the given ``topic``.

        Args:
            topic: The topic to clear the mailbox of
        """

        self.listen_for_messages(topic, time_span=None)  # ignore the result
