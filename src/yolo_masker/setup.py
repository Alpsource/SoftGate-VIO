import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'yolo_masker'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alp Demirel',
    maintainer_email='',
    description='YOLOv11-based dynamic object masker with ego-motion-compensated flow classifier for OpenVINS.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_masker = yolo_masker.yolo_masker_node:main',
        ],
    },
)
