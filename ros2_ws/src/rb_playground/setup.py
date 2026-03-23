from setuptools import find_packages, setup

package_name = "rb_playground"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="leemou",
    maintainer_email="mjzizou@naver.com",
    description="Scratch ROS2 Python playground package for quick local experiments.",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "hello_node = rb_playground.hello_node:main",
        ],
    },
)
