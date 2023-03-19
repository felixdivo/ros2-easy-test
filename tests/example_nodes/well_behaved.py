"""Some examples were inspired by the ROS 2 demos, e.g. from [*demo_nodes_py*](https://github.com/ros2/demos/tree/rolling/demo_nodes_py)."""

from rclpy.node import Node

from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts


class Talker(Node):
    """Periodically speaks up."""

    def __init__(self, *args, **kwargs):
        super().__init__("talker", *args, **kwargs)

        self._counter = 0
        self._pub = self.create_publisher(String, "/chatter", 0)
        self._timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self) -> None:
        self._pub.publish(String(data=f"Hello World: {self._counter}"))
        self._counter += 1


class EchoNode(Node):
    def __init__(self, *args, **kwargs):
        super().__init__("copycat-talker", *args, **kwargs)

        mouth = self.create_publisher(String, "/mouth", 0)
        # Immediately forwards it and holds a reference to the publisher:
        self.create_subscription(String, "/ear", mouth.publish, 0)


class AddTwoIntsServer(Node):
    def __init__(self, *args, **kwargs):
        super().__init__("add_two_ints_server", *args, **kwargs)

        self.create_service(AddTwoInts, "add_two_ints", self.callback)

    def callback(self, request: AddTwoInts.Request, response: AddTwoInts.Response):
        response.sum = request.a + request.b
        return response
