from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tl_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='YOLO vision detection package for tl_robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = tl_vision.yolo_node:main',
            'calib_node = tl_vision.calib_node:main',
            'fk_test_node = tl_vision.fk_test_ui:main',
            'control_node = tl_vision.control_node:main',
        ],
    },
)