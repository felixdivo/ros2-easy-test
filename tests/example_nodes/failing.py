try:
    from typing import Never
except ImportError:
    # Python version <3.11
    from typing import NoReturn as Never

from example_interfaces.srv import AddTwoInts
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile

from std_msgs.msg import String


class OnPurposeError(RuntimeError):
    """Raised on purpose to demonstrate the behavior of the library."""


class NodeRaiseInInit(Node):
    def __init__(self, *args, **kwargs) -> Never:
        super().__init__("this_will_be_quick", *args, **kwargs)

        raise OnPurposeError("the init method failed on purpose")


class NodeRaiseInTimer(Node):
    def __init__(self, *args, **kwargs):
        super().__init__("just_wait_for_it", *args, **kwargs)

        self._timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self) -> Never:
        raise OnPurposeError("something went wrong on purpose to demonstrate the behavior")


class NodeRaiseOnRequest(Node):
    def __init__(self, *args, **kwargs):
        super().__init__("just_call_it", *args, **kwargs)

        # Evene though we don't use this publisher, we add it to make the topic visible
        qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_ALL)
        self._mouth = self.create_publisher(String, "/mouth", qos_profile)
        self.create_subscription(String, "/ear", self.callback, qos_profile)

        self.create_service(AddTwoInts, "add_two_ints", self.callback)

    def callback(self, *_, **__) -> Never:
        raise OnPurposeError("you should have never called me, I'm a bad node")
