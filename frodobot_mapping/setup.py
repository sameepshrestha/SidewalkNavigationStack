from glob import glob
from setuptools import find_packages, setup

package_name = "frodobot_mapping"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/frodobot_mapping"]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SamepAI",
    maintainer_email="sshres32@gmu.edu",
    description="Traversability mapping for FrodoBots Earth Rover",
    license="MIT",
    entry_points={
        "console_scripts": [
            "traversability_map_node = frodobot_mapping.traversability_map:main",
        ],
    },
)
