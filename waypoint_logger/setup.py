from setuptools import setup

package_name = 'waypoint_logger'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matthew O\'Kelly',
    maintainer_email='mokelly@seas.upenn.edu',
    description='The waypoint_logger package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_logger_node = waypoint_logger.waypoint_logger_node:main'
        ],
    },
)
