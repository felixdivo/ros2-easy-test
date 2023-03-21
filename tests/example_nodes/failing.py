try:
    from typing import Never
except ImportError:
    # Python version <3.11
    from typing import NoReturn as Never

from example_interfaces.srv import AddTwoInts
from rclpy.node import Node
from std_msgs.msg import String


class OnPurposeFail(RuntimeError):
    """Raised on purpose to demonstrate the behavior of the library."""


class NodeRaiseInInit(Node):
    def __init__(self, *args, **kwargs) -> Never:
        super().__init__("this_will_be_quick", *args, **kwargs)

        raise OnPurposeFail("the init method failed on purpose")


class NodeRaiseInTimer(Node):
    def __init__(self, *args, **kwargs):
        super().__init__("just_wait_for_it", *args, **kwargs)

        self._timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self) -> Never:
        raise OnPurposeFail(
            "something went wrong on purpose to demonstrate the behavior"
        )


class NodeRaiseOnRequest(Node):
    def __init__(self, *args, **kwargs):
        super().__init__("just_call_it", *args, **kwargs)

        # Evene though we don't use this publisher, we add it to make the topic visible
        self._mouth = self.create_publisher(String, "/mouth", 0)
        self.create_subscription(String, "/ear", self.callback, 0)

        self.create_service(AddTwoInts, "add_two_ints", self.callback)

    def callback(self, *_, **__) -> Never:
        raise OnPurposeFail("you should have never called me, I'm a bad node")
