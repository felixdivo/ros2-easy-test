Limitations, Design, and Other Projects
=======================================

Current Limitations
-------------------

.. note::
  Contributions to address these or other shortcomings are more than welcome!
  There are also various ``TODO:`` comments throughout the repository -- approaching any of those will benefit the library a lot!

  `Pull requests <https://github.com/felixdivo/ros2-easy-test/pulls>`__ are the way to go.  
  If you are unsure about anything, feel free to open an `issue <https://github.com/felixdivo/ros2-easy-test/issues>`__.
  See this project's `README <https://github.com/felixdivo/ros2-easy-test#Contributing>`__ for details on getting started.

- If a callback (e.g. of a subscriber in the node) raises an exception the test does not fail automatically with the exception as the reason, as that is currently
  `not supported in ROS2 <https://discourse.ros.org/t/what-is-the-expected-behavior-of-rclcpp-in-case-of-an-exception-raised-in-a-user-callback/27527>`__.
  It will probably still fail because some expected message is not detected by the test.
  In those cases, you will have to look for messages like ``The following exception was never retrieved: [...]`` in the stderr output of pytest.
  It will probably be mixed in with other messages if you view it on the console.
- Similar "silent" crashes can occur in other places, including timers and services.
  See ``tests/test_failing_nodes.py`` for unit tests on that behavior.
  A lot of them are marked with a ``TODO:``, since it is currently not straightforward to detect such issues.
- A failing service might deadlock a test. Consider calling services asynchronously with timeouts.
- It takes some time to set up the test environment each time, particularly with ``@with_launch_file``.
  You may wish to append ``--durations=0 --durations-min=1.0`` to your pytest call to show the slowest tests
  (`more info <https://docs.pytest.org/en/latest/how-to/usage.html#profiling-test-execution-duration>`__).
  There is probably room for improvement here, especially with reducing the required warm-up time.
- The provided stack traces could be more useful. One should determine whether to use :func:`functools.wraps` or not.
  See the comments in 

Design Considerations
---------------------

These were the initial design considerations. Over time, this may shift.

Goals
~~~~~

- Provide a method for writing node tests that can assert correct behavior via sending messages and observing
  message output.
- Allow for concise tests. Little to no infrastructure code should be required for typical scenarios.
- Automate common checks where possible (e.g. nodes should not crash).
- Support testing single and multiple nodes in combination (i.e. support *Unit tests* (of single nodes),
  *Integration tests* and *System tests* as per
  `ROS2 terminology
  <https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Developer-Guide.html#testing>`__).
- Be well-documented and easy to get started.

Constraints
~~~~~~~~~~~

- Be easy to maintain by needing only a few lines of code (below a thousand). Thus, actions and services are
  currently not supported. Also, only use public APIs wherever possible.
- Don't reinvent the wheel and benefit from future improvements: Use existing functionality of Python and
  ROS2. This includes: :mod:`unittest`, :mod:`pytest`, and the
  `ROS2 launch system <https://design.ros2.org/articles/roslaunch.html>`__.
- Efficiency is not a primary concern as this mini-framework is intended to only be used for testing
  and not in a real robot deployment, where performance is much more of a concern.
- Must work with ``colcon test`` (see `here <https://colcon.readthedocs.io/en/released/reference/verb/test.html>`__) and also with just *pytest* alone (for simpler IDE integration).

Comparison to Other Projects
----------------------------

Several other projects are attempting to make `testing of ROS2 systems <https://docs.ros.org/en/rolling/Tutorials/Intermediate/Testing/Testing-Main.html>`__ easier.
However:

- With `launch_pytest <https://github.com/ros2/launch/tree/rolling/launch_pytest>`__, it is not easy to interact with the system, e.g. using publishers and subscribers.
- The same holds for `launch_testing <https://github.com/ros2/launch/tree/rolling/launch_testing>`__.

As a side note, this functionality could not have been implemented with `pytest fixtures <https://docs.pytest.org/en/latest/explanation/fixtures.html>`__,
since we need to be able to receive different parameters for each test.
