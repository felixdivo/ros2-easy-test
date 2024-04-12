"""Provides a helper for the decorators. You will in general not need to use this module directly."""

# Standard library
from functools import cached_property
from pathlib import Path
from tempfile import TemporaryDirectory

# Typing
from types import TracebackType
from typing import Any, ContextManager, List, Literal, Optional, Type, Union, cast


class LaunchFileProvider(ContextManager[Path]):
    """Provides the given launch file as a path, creating and deleting a temp file if a literal was given.

    This can be used to handle arguments like ``launch_file`` in :func:`ros2_easy_test.with_launch_file`.

    Args:
        launch_file: Either:
            1) The path to a launch file.
            2) A literal launch file (must contain a newline somewhere to be detected as such).
    """

    def __init__(self, launch_file: Union[Path, str]):
        self._launch_file = launch_file

        # A list of context managers to exit upon exiting this class
        self._exit_list: List[ContextManager[Any]] = []

    @cached_property
    def is_path(self) -> bool:
        """If the launch file does not contain a newline, it is a path."""

        if isinstance(self._launch_file, Path):
            return True
        else:
            return "\n" not in self._launch_file and "\r" not in self._launch_file

    def __enter__(self) -> Path:
        if self.is_path:
            launch_file_path = self._launch_file
            if not isinstance(launch_file_path, Path):
                launch_file_path = Path(self._launch_file)
        else:
            assert isinstance(self._launch_file, str)
            directory = TemporaryDirectory()
            self._exit_list.append(cast(ContextManager[str], directory))
            directory_name = Path(directory.__enter__())
            launch_file_path = directory_name / "launch_file"
            with open(launch_file_path, "w", encoding="utf-8") as file:
                file.write(self._launch_file)

        return launch_file_path

    def __exit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc_val: Optional[BaseException],
        exc_tb: Optional[TracebackType],
    ) -> Literal[False]:
        for manager in self._exit_list:
            manager.__exit__(exc_type, exc_val, exc_tb)
        return False
