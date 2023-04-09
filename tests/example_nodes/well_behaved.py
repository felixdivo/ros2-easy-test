from example_interfaces.srv import AddTwoInts
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSHistoryPolicy


class Talker(Node):
    """Periodically speaks up."""

    def __init__(self, *args, **kwargs):
        super().__init__("talker", *args, **kwargs)

        self.declare_parameter("/start_value", value=0)  # Set a default
        self._counter = self.get_parameter("/start_value").value

        self._pub = self.create_publisher(String, "/chatter", 0)
        self._timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self) -> None:
        self._pub.publish(String(data=f"Hello World: {self._counter}"))
        self._counter += 1


class EchoNode(Node):
    def __init__(self, *args, **kwargs):
        super().__init__("copycat_talker", *args, **kwargs)

        qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_ALL)
        mouth = self.create_publisher(String, "/mouth", qos_profile)
        # Immediately forwards it and also holds a reference to the publisher:
        self.create_subscription(String, "/ear", mouth.publish, qos_profile)


class AddTwoIntsServer(Node):
    def __init__(self, *args, **kwargs):
        super().__init__("add_two_ints_server", *args, **kwargs)

        self.create_service(AddTwoInts, "/add_two_ints", self.callback)

    def callback(self, request: AddTwoInts.Request, response: AddTwoInts.Response):
        response.sum = request.a + request.b
        return response
