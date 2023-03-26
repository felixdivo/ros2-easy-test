# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
from os.path import abspath
from os.path import dirname
from os.path import join
import sys

sys.path.insert(0, abspath(join(dirname(__file__), "../../")))  # for scripts/

import ros2_easy_test  # noqa: E402

# -- Project information -----------------------------------------------------

project = "ros2_easy_test"
copyright = ros2_easy_test.__author__
author = ros2_easy_test.__author__

# The version info for the project, acts as replacement for
# |version| and |release|, also used in various other places throughout the
# built documents.
#
# The short X.Y version.
version = ros2_easy_test.__version__.split("-")[0]
# The full version, including alpha/beta/rc tags
release = ros2_easy_test.__version__

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

# -- Options for LaTeX output ---------------------------------------------

latex_engine = "pdflatex"

latex_elements = {
    # The paper size ('letterpaper' or 'a4paper').
    "papersize": "a4paper",
    # The font size ('10pt', '11pt' or '12pt').
    # 'pointsize': '10pt',
}

# Grouping the document tree into LaTeX files. List of tuples
# (source start file, target name, title,
#  author, documentclass [howto, manual, or own class]).
latex_documents = [("index", "ros2_easy_test.tex", f"{project} Documentation", author, "manual")]
