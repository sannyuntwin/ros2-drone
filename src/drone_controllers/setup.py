from setuptools import find_packages, setup

package_name = 'drone_controllers'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maintainer',
    maintainer_email='sannyuntwin21@gmail.com',
    description='ROS 2 drone controller nodes: PX4 offboard control and object tracking',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_node = drone_controllers.offboard_node:main',
            'tracking_node = drone_controllers.tracking_node:main',
        ],
    },
)
