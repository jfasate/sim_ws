import os
from glob import glob
from setuptools import setup

package_name = 'lpv_mpc'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'numpy', 'qpsolvers'],
    zip_safe=True,
    maintainer='jayesh',
    maintainer_email='jayesh@todo.com',
    description='LPV-MPC controller for F1Tenth',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lpv_mpc_node = lpv_mpc.lpv_mpc_node:main',
        ],
    },
)
