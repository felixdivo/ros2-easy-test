# ROS2 easy-test

[![PyPI version](https://img.shields.io/pypi/v/ros2-easy-test.svg?color=blue)](https://pypi.org/project/ros2-easy-test/)
[![license](https://img.shields.io/pypi/l/ros2-easy-test.svg?color=blue)](https://github.com/felixdivo/ros2-easy-test/blob/main/LICENSE)
[![ros2 version](https://img.shields.io/badge/ROS2-Humble%20Hawksbill+-blue)](https://docs.ros.org/en/rolling/Releases.html)
[![Python version](https://img.shields.io/badge/python-3.8+%20(matching%20ROS)-blue)](https://devguide.python.org/versions/)

[![CI status](https://github.com/felixdivo/ros2-easy-test/actions/workflows/python-package.yaml/badge.svg)](https://github.com/felixdivo/ros2-easy-test/actions/workflows/python-package.yaml)
[![documentation status](https://readthedocs.org/projects/ros2-easy-test/badge/)](https://ros2-easy-test.readthedocs.io/en/latest/)
[![Ruff](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json)](https://github.com/astral-sh/ruff)
[![static type checker](https://img.shields.io/badge/static%20typing-mypy-black)](https://mypy-lang.org/)

A Python test framework for [ROS2](https://ros.org/) allowing for:
- simple and expressive assertions based on message and service interactions (black box testing)
- easy integration of existing nodes and launch files
- testing of nodes implemented in any programming language (C++, Python, etc.)
- works with and without tools like `colcon test` and `pytest`
- is minimalistic and has [very few dependencies](https://github.com/felixdivo/ros2-easy-test/blob/main/pyproject.toml)
- is typed and [documented](https://ros2-easy-test.readthedocs.io/en/latest/)
- is tested, used in practice, and maintained

## Installation

Just run:
```shell
pip install ros2-easy-test
```

## Examples

The following two examples demonstrate the usage of the Python decorators `@with_single_node` and `@with_launch_file`, which provide this package's core functionality.
To get a better understanding of their inner workings, please have a look at their implementation [here](ros2_easy_test/decorators.py).
Besides the simple examples here, you can embed everything in `unittest.TestCase` as well. 
To check out how, have a look at the provided test in [tests/demo/](tests/demo/) for some advanced examples.

### Testing a Node

In simple settings, where a single node is to be tested, the decorator `@with_single_node` can be used:

```python
from ros2_easy_test import ROS2TestEnvironment, with_launch_file, with_single_node
from my_nodes import Talker
from std_msgs.msg import String

@with_single_node(Talker, watch_topics={"/chatter": String})
def test_simple_publisher(env: ROS2TestEnvironment) -> None:
    response: String = env.assert_message_published("/chatter", timeout=5)
    assert response.data == "Hello World: 0"
```
It is also possible to specify custom QoSProfiles for the `ROS2TestEnvironment` to use when subscribing or publishing on the specified topics. See [`tests/demo/latching_topic_test.py`](tests/demo/latching_topic_test.py) for an example. If no profile is specified, the default `QoSProfile(history=QoSHistoryPolicy.KEEP_ALL)` is used.

You can optionally provide more parameters to the test setting, i.e., additionally pass `parameters={"some.thing": 30.2}` to the decorator.
The argument of the test function receiving the `ROS2TestEnvironment` *must* be named `env`.

### Testing a Launch File

For more complex scenarios involving multiple nodes using a launch file (both nodes and launch file being implemented in [any language supported by ROS2](https://docs.ros.org/en/rolling/How-To-Guides/Launch-file-different-formats.html)), the `@with_launch_file` decorator can be used.

```python
@with_launch_file(
    "example_launch_file.yaml",
    watch_topics={"/some/interesting/response": ColorRGBA},
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
- `call_service(name: str, request: Request, ...) -> Response`
- `send_action_goal(name: str, goal: Any, ...) -> Tuple[ClientGoalHandle, List[FeedbackMsg]]`
- `send_action_goal_and_wait_for_result(name: str, goal: Any, ...) -> Tuple[List[FeedbackMsg], ResultMsg]`

Note that a `ROS2TestEnvironment` is a normal [`rclpy.node.Node`](https://docs.ros2.org/latest/api/rclpy/api/node.html) and thus has all the methods of any other ROS2 node.
So feel free to offer a service with `env.create_service()` and cover more specific use cases.
Extend as you please!

In addition, nothing stops you from using any other means of interacting with ROS2 that would work otherwise.

### What you can test (recommended way of obtaining messages)

Using `ROS2TestEnvironment`, you can assert:
- `assert_message_published(topic: str, timeout: Optional[float]) -> RosMessage`
- `assert_no_message_published(topic: str, timeout: Optional[float]) -> None`
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

See [the documentation on that](https://ros2-easy-test.readthedocs.io/en/latest/design_and_limits.html).
Please read it before suggesting major features or changes.

## Contributing

Basic installation is easiest with the provided `Dockerfile`.
For the last piece of setup, either open it in the provided [Devcontainer](https://code.visualstudio.com/docs/remote/containers) or maunally run `rosdep install --from-paths ros2_easy_test && colcon build --symlink-install && pip install -e './ros2_easy_test[dev]` afterward.

After this, you will have access to the configured formatter (`ruff format`) and linter (`ruff check`).

You can run the test with simply `pytest`. Coverage reports and timings will be printed on the command line, and a fresh line-by-line coverage report is in `htmlcov/index.html`.

Building the documentation is simple, too:
```shell
# Install the required dependencies
pip install -e ".[doc]"

# Build the documentation
cd doc
make html
# open build/html/index.html in your browser

# You can also run a small webserver to serve the static files with
cd build/html
python -m http.server
```

## Changelog

See [Releases](https://github.com/felixdivo/ros2-easy-test/releases).

## License

See [LICENSE](LICENSE).

Initially developed by [Felix Divo](https://github.com/felixdivo) at [*Sailing Team Darmstadt e. V.*](https://www.st-darmstadt.de/), a student group devoted to robotic sailing based in Darmstadt, Germany.
Thanks to [Simon Kohaut](https://github.com/simon-kohaut) for his kind and nuanced feedback, and all [the other awesome contributors](https://github.com/felixdivo/ros2-easy-test/graphs/contributors) for their help and support!
