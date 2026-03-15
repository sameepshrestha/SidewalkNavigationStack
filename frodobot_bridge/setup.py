from glob import glob
from setuptools import find_packages, setup

package_name = 'frodobot_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
    ],
    install_requires=['setuptools', 'requests', 'numpy', 'opencv-python'],
    zip_safe=True,
    maintainer='SamepAI',
    maintainer_email='sshres32@gmu.edu',
    description='ROS 2 bridge for the FrodoBots Earth Rover SDK',
    license='MIT',
    scripts=[
        'nodes/sdk_bridges',
    ],
)
