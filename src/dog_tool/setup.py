from setuptools import setup, find_packages

package_name = "dog_tool"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test_*", "tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="tyl",
    author_email="2684711880@qq.com",
    maintainer="tyl",
    maintainer_email="2684711880@qq.com",
    description="Visualization and debugging tools for the dog robot (Python).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "lf_foot_monitor = dog_tool.lf_foot_monitor:main",
        ],
    },
)
