from setuptools import find_packages, setup

package_name = 'frodobot_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/heading_fusion.launch.py', 'launch/localization.launch.py']),
        ('share/' + package_name + '/config', ['config/heading_fusion.yaml', 'config/navsat_transform_example.yaml', 'config/ekf_example.yaml']),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenAI',
    maintainer_email='user@example.com',
    description='ROS 2 heading fusion node for GPS course-over-ground + IMU gyro, with optional external global yaw source.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'heading_fusion_node = frodobot_localization.heading_fusion_node:main',
        ],
    },
)
