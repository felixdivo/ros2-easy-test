try:
    from typing import Never
except ImportError:
    # Python version <3.11
    from typing import NoReturn as Never

from rclpy.node import Node

from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts


class OnPruposeFail(RuntimeError):
    """Raised on purpose to demonstrate the behavior of the library."""


class NodeRaiseInInit(Node):
    def __init__(self, *args, **kwargs) -> Never:
        super().__init__("this_will_be_quick", *args, **kwargs)

        raise OnPruposeFail("the init method failed on purpose")


class NodeRaiseInTimer(Node):
    def __init__(self, *args, **kwargs):
        super().__init__("just_wait_for_it", *args, **kwargs)

        self._timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self) -> Never:
        raise OnPruposeFail(
            "something went wrong on purpose to demonstrate the behavior"
        )


class NodeRaiseOnRequest(Node):
    def __init__(self, *args, **kwargs):
        super().__init__("just_call_it", *args, **kwargs)

        self.create_subscription(String, "/ear", self.callback, 0)
        self.create_service(AddTwoInts, "add_two_ints", self.callback)

    def callback(self, *_, **__) -> Never:
        raise OnPruposeFail("you should have never called me, I'm a bad node")
