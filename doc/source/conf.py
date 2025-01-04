# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Setup -------------------------------------------------------------------

import re
import sys
from pathlib import Path
from unittest.mock import Mock

# -- Mock ROS2 ---------------------------------------------------------------

try:
    import rclpy
except ImportError:
    sys.modules["rclpy"] = Mock()
else:
    del rclpy

try:
    import action_msgs
except ImportError:
    sys.modules["action_msgs"] = Mock()
else:
    del action_msgs

# -- Project information -----------------------------------------------------

project = "ROS2 easy-test"

# From: https://stackoverflow.com/a/5872024/3753684
with open(Path(__file__).parents[2] / "ros2_easy_test" / "ros2_easy_test" / "__init__.py") as init:
    metadata = dict(re.findall('''__([a-z]+)__ = "([^"]+)"''', init.read()))

copyright = f"2021, {metadata['author']}"
author = metadata["author"]

# The version info for the project, acts as replacement for
# |version| and |release|, also used in various other places throughout the
# built documents.
#
# The short X.Y version.
version = metadata["version"].split("-", maxsplit=1)[0]
# The full version, including alpha/beta/rc tags
release = metadata["version"]

# -- General configuration ---------------------------------------------------

primary_domain = "py"

# If this is True, todo and todolist produce output, else they produce nothing.
# The default is False.
todo_include_todos = True

language = "en"

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.intersphinx",
    "sphinx.ext.napoleon",
    "sphinx.ext.autosummary",
    "sphinx.ext.doctest",
    "sphinx_rtd_theme",
]

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = []

source_suffix = [".rst"]

intersphinx_mapping = {
    "python": ("https://docs.python.org/3", None),
    "pytest": ("https://docs.pytest.org/en/stable", None),
    "pytest-cov": ("https://pytest-cov.readthedocs.io/en/stable", None),
    "hypothesis": ("https://hypothesis.readthedocs.io/en/latest", None),
    "rclpy": ("https://docs.ros2.org/latest/api/rclpy/", None),
}

nitpicky = True

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes or:
# - https://sphinx-themes.org/
# - https://www.writethedocs.org/guide/tools/sphinx-themes/
html_theme = "sphinx_rtd_theme"

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = []  # '_static'

html_sidebars = {"**": ["globaltoc.html", "relations.html", "sourcelink.html", "searchbox.html"]}

autodoc_mock_imports = ["rclpy"]

# -- Options for LaTeX output ---------------------------------------------

latex_engine = "pdflatex"

latex_elements = {
    "papersize": "a4paper",
    # 'pointsize': '10pt',
}

# Grouping the document tree into LaTeX files. List of tuples
# (source start file, target name, title,
#  author, documentclass [howto, manual, or own class]).
assert "_" not in project, "underscores are not rendered as such in LaTeX"
assert "_" not in author, "underscores are not rendered as such in LaTeX"
latex_documents = [("index", "ros2_easy_test.tex", f"{project} Documentation", author, "manual")]
