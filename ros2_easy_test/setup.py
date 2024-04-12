from xml.etree import ElementTree

from setuptools import find_packages, setup


# Try to get package info
# Since checking everything for existence would be cumbersome, we disable strict typing here
# mypy: no-strict-optional
package = ElementTree.parse("package.xml").getroot()
PACKAGE_NAME = package.find("name").text


setup(
    name=PACKAGE_NAME,
    version=package.find("version").text,
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
    ],
    zip_safe=True,
    maintainer=package.find("maintainer").text,
    maintainer_email=package.find("maintainer").attrib["email"],
    description=package.find("description").text,
    license=package.find("license").text,
    url="https://github.com/felixdivo/ros2-easy-test",
    entry_points={"console_scripts": []},
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
        "Natural Language :: English",
        "License :: OSI Approved :: MIT License",
        "Intended Audience :: Developers",
        "Framework :: Robot Framework :: Library",
        "Topic :: Software Development :: Testing",
        "Topic :: Software Development :: Testing :: BDD",
        "Typing :: Typed",
    ],
    requires_python=">=3.8",
    install_requires=["setuptools", "makefun>=1.15.2"],
    extras_require={
        "dev": [
            "black~=24.2",
            "ruff",
            "mypy",
            "pytest",
            "pytest-cov",
            "pytest-sugar",
            "hypothesis",  # To demonstrate how it works with this
        ],
        "doc": [
            "sphinx",
            "sphinx_rtd_theme",
        ],
    },
)
