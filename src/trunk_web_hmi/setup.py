from glob import glob
from setuptools import find_packages, setup

package_name = "trunk_web_hmi"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wxl",
    maintainer_email="fanholmes29@gmail.com",
    description="Web HMI MVP for trunk robot control, planning, waypoints, and diagnostics.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "web_hmi_backend = trunk_web_hmi.server:main",
        ],
    },
)
