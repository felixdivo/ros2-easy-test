"""
``ros2_easy_test`` is a Python test framework for ROS2 allowing simple and expressive assertions mainly based
on message interaction.

Mini-Guide
----------

The idea is simple: Decorate a test function with ``@with_single_node(...)`` or with
``@with_launch_file(...)`` and let the test function receive a :class:`~ROS2TestEnvironment` object.
The test can then interact with ROS2 nodes by calling appropriate methods on the environment for publishing
messages or asserting that certain ones are received.

:func:`~with_single_node` can only be used with Python nodes but test coverage can be collected out of the
box.
:func:`~with_launch_file` allows to test nodes written in other languages too (using Python test code).
However, it requires to write a (small) launch file written in YAML, XML, Python, etc. and is slower.

Note:
    When using ``@with_single_node(...)``, make sure that your node passes along any keyword arguments.
    See :func:`~with_single_node` for details.

Warning:
    If the node crashes due to an exception in service callback, a test calling the service will deadlock
    indefinitely. In that case, exiting with ``Ctrl + C`` will probably print the exception in the node.

Example
-------

This is a fully contained example (``examples/simple_example.py``):

.. literalinclude:: ../../src/ros2_easy_test/examples/simple_example.py
  :emphasize-lines: 12,13,14,20,21,22,23,26,29,32,33,34,35,39,42
  :language: Python

Design Goals
------------

- Provide a method for writing node tests that can assert correct behaviour via sending messages and observing
  message output.
- Allow for concise tests. Little to no infrastructure code should be required for usual scenarios. Automate
  common checks were possible (e.g. nodes should not crash).
- Support testing single and multiple nodes in combination (i.e. support *Unit tests* (of single nodes),
  *Integration tests* and *System tests* as per
  `ROS2 terminology
  <https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Developer-Guide.html#testing>`__.
- Be well documented and easy to get started.

Design Constraints
------------------

- Be easy to maintain by needing only a few lines of code (below a thousand). Thus, actions and services are
  currently not supported. Also, only use public APIs wherever possible.
- Don't reinvent the wheel and benefit from future improvements: Use existing functionality of Python and
  ROS2. This includes: ``unittest``, *pytest* and the ROS2 launch system.
- Efficiency is not a concern as this mini-framework is intended to only be used for testing and not in a real
  robot deployment.
- Must work with ``colcon test`` and also with just *pytest* alone (for simpler IDE integration).

"""

__author__ = "Felix Divo <felix.divo@sailingteam.tu-darmstadt.de>"
__version__ = "0.1.0"

from .decorators import with_launch_file, with_single_node
from .env import ROS2TestEnvironment

__all__ = ["ROS2TestEnvironment", "with_launch_file", "with_single_node"]
