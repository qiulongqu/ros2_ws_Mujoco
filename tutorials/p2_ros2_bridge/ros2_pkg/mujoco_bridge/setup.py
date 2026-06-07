from setuptools import setup

package_name = 'mujoco_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[f'{package_name}.{package_name}'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         [f'resource/{package_name}']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS2 Mujoco bridge node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bridge_node = mujoco_bridge.mujoco_bridge.bridge_node:main',
        ],
    },
)