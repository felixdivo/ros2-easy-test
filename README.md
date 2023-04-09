# ROS2 easy-test

[![python version](https://img.shields.io/badge/python-3.8+%20(matching%20the%20ROS%20version)-green)](https://devguide.python.org/versions/)
[![ros2 version](https://img.shields.io/badge/ROS2-Foxy%20Fitzroy+-green)](https://docs.ros.org/en/rolling/Releases.html)

[![Python package](https://github.com/felixdivo/ros2-easy-test/actions/workflows/python-package.yaml/badge.svg)](https://github.com/felixdivo/ros2-easy-test/actions/workflows/python-package.yaml)

A Python test framework for ROS2 allowing for:
- simple and expressive assertions based on message/service interactions (black box testing)
- easy integration of existing nodes and launch files
- testing of nodes implemented in any programming language (C++, Python, ...)
- works with and without tools like `colcon test` and `pytest`
- is minimalistic and has very few dependencies
- is tested, used in practice, documented, and maintained

## Installation

At the moment, you can run the following to install this package.
```shell
pip install git+git://github.com/felixdivo/ros2-easy-test
```
In the future, we intend to publish this repository to be easily installed with rosdep and via PyPI.

## Examples

The following two examples show off the usage of the Python decorators `@with_single_node` and `@with_launch_file`, which provide the core functionality of this package.
To get a better grasp of their inner workings, have a look at their implementation [here](ros2_easy_test/decorators.py).
Besides the simple examples here, you can embed everything in `unittest.TestCase` as well. 
To check out how, have a look at the provided [tests/](tests/) for some advanced examples.

### Testing a Node

Simple settings where a single node shall be tested can make use of the decorator `@with_single_node` as in the following example.

```python
from ros2_easy_test import ROS2TestEnvironment, with_launch_file, with_single_node
from my_nodes import Talker
from std_msgs.msg import String

@with_single_node(Talker, watch_topics={"/chatter": String})
def test_simple_publisher(env: ROS2TestEnvironment) -> None:
    response: String = env.assert_message_published("/chatter", timeout=5)
    assert response.data == "Hello World: 0"
```

You can optionally provide more parameters to the test setting, i.e., additionally pass `parameters={"some.thing": 30.2}` to the decorator.
The argument of the test function receiving the `ROS2TestEnvironment` *must* be named `env`.

### Testing a Launch File

For more complex scenarios involving multiple nodes using a launch file (both nodes and launch file being implemented in [any language supported by ROS2](https://docs.ros.org/en/rolling/How-To-Guides/Launch-file-different-formats.html)), the `@with_launch_file` decorator can be used.

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

You can also pass the literal launch file contents as a `str` instead of a path like `"example_launch_file.yaml"`.
The argument of the test function receiving the `ROS2TestEnvironment` *must* be named `env`.

Note that, however, this method is much slower than the one above. 
One reason for this is the requirement of a fixed warm-up time for the nodes to be started. 
This is because the test environment has to wait for the nodes to be ready before it can start listening for messages.

## Usage

### How you can interact with the node(s)

Using `ROS2TestEnvironment`, you can call:
- `publish(topic: str, message: RosMessage) -> None`
- `listen_for_messages(topic: str, time_span: float) -> List[RosMessage]`
- `clear_messages(topic: str) -> None` to forget all messages that have been received so far.

Note that `ROS2TestEnvironment` is a `rclpy.node.Node` and thus has all the methods of a ROS2 node. 
As an example, you can create a service by `env.create_service(...)` and then use it with `env.call(...)`.
In addition, nothing stops you from using any other means of interacting with ROS2 that would work otherwise.

### What you can test (recommended way of obtaining messages)

Using `ROS2TestEnvironment`, you can assert:
- `assert_message_published(topic: str, timeout: float) -> RosMessage`
- `assert_no_message_published(topic: str, timeout: float) -> None`
- `assert_messages_published(topic: str, number: int, ...) -> List[RosMessage]`

Generally, you can always test that no exceptions are thrown, e.g., when nodes are initialized (see limitations below).

### Combining with other tools

Some hints:
- If you want to use [pytest markers](https://docs.pytest.org/en/7.1.x/how-to/mark.html) like `@pytest.mark.skipif(...)`, add that above (=before) the `@with_single_node(...)`/`@with_launch_file(...)` decorator and it will work just fine.
- Similarly, you can seamlessly use other tools which annotate test functions, like [hypothesis](https://hypothesis.readthedocs.io/en/latest/) or [pytest fixtures](https://docs.pytest.org/en/6.2.x/fixture.html).
  Generally, you have to be mindful of the order of the decorators here.
  The `ROS2TestEnvironment` is always added as a keyword argument called `env` to the test function.
  See `tests/demo/` for a few examples.

## Limitations, Design, and Other Projects

See [TODO: Link to docs](#Limitations, Design, and Other Projects)

## Contributing

You can install the development dependencies with `pip install -e ".[dev]"`. After this, you will have access to the configured formatters `black` and `isort`.

You can run the test with simply `pytest`. Coverage reports and timings will be printed on the command line, and a fresh line-by-line coverage report is in `htmlcov/index.html`.

Building the documentation is simple too:
```shell
cd doc
make html
# open build/html/index.html in you browser

# You can also run a small webserver with
cd build/html
python -m http.server
```

## License

See [LICENSE](LICENSE).

Initially developed by [Felix Divo](https://github.com/felixdivo) at [*Sailing Team Darmstadt e. V.*](https://www.st-darmstadt.de/), a student group devoted to robotic sailing based in Darmstadt, Germany.
Thanks to [Simon Kohaut](https://github.com/simon-kohaut) for his kind and nuanced feedback.

## TODOs

- Decide on a name
- Publish docs, reference from repo/README
- Make public, spread the word
- push to PyPI
