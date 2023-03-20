# ROS2 easy-test

[![python version](https://img.shields.io/badge/python-3.8+-green)](https://devguide.python.org/#status-of-python-branches)
[![ros2 version](https://img.shields.io/badge/ROS2-Humble%20Hawksbill+-green)](https://docs.ros.org/en/rolling/Releases.html)

A Python test framework for ROS2 allowing for:
- simple and expressive assertions based on message/service interactions (black box testing)
- easy integration of existing nodes and launch files
- testing of nodes implemented in any programming language (C++, Python, ...)
- works with and without tools like `colcon test` and `pytest`
- has no other dependencies
- is tested, used in practice, documented, and maintained

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

### For any launch file

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

Note that, however, this method is much slower than the one above. One reason for this is the requirement of a fixed warm-up time for the nodes to be started. This is because the test environment has to wait for the nodes to be ready before it can start listening for messages.

### More

You can also arrange youe tests in a `unittest.TestCase`, nothing special is happening here. See `tests/` for some examples.

## Usage

### How you can interact with the node(s)

Using `ROS2TestEnvironment`, you can call:

- `publish(topic: str, message: RosMessage) -> None`
- Note that `ROS2TestEnvironment` is a `rclpy.node.Node` and thus has all the methods of a node. This means, that you can create a service by `env.create_service(...)` and then use it with `env.call(...)`.
- `listen_for_messages(topic: str, time_span: float) -> List[RosMessage]`
- `clear_messages(topic: str) -> None` to forget all messages that have been received so far.

In addition, nothing stops you from using any other means of interacting with ROS2 that would work otherwise.

### What you can test (recommended way of obtaining messages)

Using `ROS2TestEnvironment`, you can assert:
- `assert_message_published(topic: str, timeout: float) -> RosMessage`
- `assert_no_message_published(topic: str, timeout: float) -> None`
- `assert_messages_published(topic: str, number: int, ...) -> List[RosMessage]`

Generally, you always test that no exceptions are thrown, e.g. when nodes are initialized (see limitations below).

### Combining with other tools

Some hints:
- If you want to use [pytest markers](https://docs.pytest.org/en/7.1.x/how-to/mark.html) like `@pytest.mark.skipif(...)`, add that above (=before) the `@with_single_node(...)`/`@with_launch_file(...)` decorator and it will work just fine.
- Similarly, you can seamlessly use other tools which annotate test functions, like `hypothesis` (or [pytest fixtures](https://docs.pytest.org/en/6.2.x/fixture.html)). Generally, you have to be mindful about the order of the decorators here. See `tests/demo_hypothesis_test.py` for two simple examples.
- The `ROS2TestEnvironment` is added as the last postional argument to the test function (i.e. right before the keyword arguments).

### Current limitations

- If a callback (e.g. of a subscriber in the node) raises an exception the test does not fail automatically with the exception as the reason, as that is currently [not supported in ROS2](https://discourse.ros.org/t/what-is-the-expected-behavior-of-rclcpp-in-case-of-an-exception-raised-in-a-user-callback/27527). It will probably still fail because some expected message is not detected by the test. In those cases, you will have to look for messages like `The following exception was never retrieved: [...]` in the stderr output of pytest. It will probably be mixed in with other messages if you view it one the console.
- A failing service might deadlock a test. Consider adding timeouts.
- It takes some time to set up the test environment each time, particularly with `@with_launch_file`. You may wish to append `--durations=0 --durations-min=1.0` to your pytest call to show the slowest tests ([more info](https://docs.pytest.org/en/latest/how-to/usage.html#profiling-test-execution-duration)). There is probably room for improvement here, especially with reducing the required warm-up time.

Contributions to address these or other shortcomings are more than welcome!

## License

See [LICENSE](LICENSE).

## TODOs

- Add tests using the nodes in `tests/example_nodes/failing.py`
- Decide on where to put documentaion (README, `__init__.py`, website)
- Setup CI and add badages to repo README
- Compare to other tools, e.g. https://github.com/ros2/launch
- Add branch protection
- Make public, spread the word
