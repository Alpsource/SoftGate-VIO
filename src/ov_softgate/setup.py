import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ov_softgate'

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
    description='SoftGate-VIO: GT semantic masker, stereo DOV tracker, and path recorder for dynamic-object-aware VIO evaluation.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'masker = ov_softgate.semantic_masker:main',
            'path_recorder = ov_softgate.path_recorder:main',
            'hybrid_speed_estimator = ov_softgate.hybrid_speed_estimator:main',
        ],
    },
)
