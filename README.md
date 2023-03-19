# ROS2 easy-test

A Python test framework for ROS2 allowing simple and expressive assertions based on message interactions.

## Installation

Just run `pip install git+git://github.com/felixdivo/ros2-easy-test` (for now).

## Examples

### For a single Python node

```python
from ros2_easy_test import ROS2TestEnvironment, with_launch_file, with_single_node
from my_nodes import Talker
from std_msgs.msg import String

@with_single_node(Talker, watch_topics={"/chatter": String})
def test_simple_publisher(env: ROS2TestEnvironment) -> None:
    response: String = env.assert_message_published("/chatter", timeout=5)
    assert response.data == "Hello World: 0"
```

You can optionally pass `parameters={"some.thing": 30.2}`.

### For any lanuch file

The nodes may be implemented in any programming lanugage (C++, Python ...).

```python
@with_launch_file(
    "example_launch_file.yaml",
    watch_topics={"/some/interesting/topic": ColorRGBA},
)
def test_simple_update_launch_file(env: ROS2TestEnvironment) -> None:
    env.publish("/topic/for/node_input", ColorRGBA(r=0.5, g=0.2, b=0.9, a=1.0))
    response_color = env.assert_message_published("/some/interesting/response")
    assert response_color.r == 0.5
```

The launch file may be written in any language [that is supported by ROS2](https://docs.ros.org/en/rolling/How-To-Guides/Launch-file-different-formats.html), like YAML, XML, or Python.

### More

You can also arrange you tests in a `uinttest.TestCase`, nothing special is happening here. See `tests/` for some examples.

Similarly, you can use other tools like `hypothesis`. (TODO: add example)

## Usage

### How you can interact with the node(s)

Using `ROS2TestEnvironment`, you can call:

- `publish(topic: str, message: RosMessage) -> None`
- `listen_for_messages(topic: str, time_span: float) -> List[RosMessage]`
- Note that `ROS2TestEnvironment` is a `rclpy.node.Node` and thus has all the methods of a node. This means, that you can create a service by `env.create_service(...)` and then use it with `env.call(...)`.
- `clear_messages(topic: str) -> None` to forget all messages that have been received so far.

In addition, nothing stops you from using any other means of interacting with ROS2 that would work otherwise.

### What you can test

Using `ROS2TestEnvironment`, you can assert:
- `assert_message_published(topic: str, timeout: float) -> RosMessage`
- `assert_no_message_published(topic: str, timeout: float) -> None`
- `assert_messages_published(topic: str, number: int, ...) -> List[RosMessage]`

Generally, that no exceptions are thrown, e.g. when nodes are initialized (see limitations below).

### Current limitations

- If a callback (e.g. of a subscriber in the node) raises an exception the test does not fail automatically with the exception as the reason, as that is currently [not supported in ROS2](https://discourse.ros.org/t/what-is-the-expected-behavior-of-rclcpp-in-case-of-an-exception-raised-in-a-user-callback/27527). It will probably still fail because some expected message is not detected by the test. In those cases, cou will have to look for messages like `The following exception was never retrieved: [...]` in the stderr output of pytest. It will probably be mixed in with other messages if you view it one the console.
- A failing service might deadlock a test. Consider adding timeouts.

Contributions to address these or other shortcomings are more than welcome!

## License

See [LICENSE](LICENSE).
