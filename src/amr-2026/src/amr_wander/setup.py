from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'amr_wander'

setup(
    name=package_name,
    version='0.0.1',
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
    maintainer='storagy',
    maintainer_email='bluephysi01@gmail.com',
    description='Lightweight reactive obstacle-avoidance wandering node for Storagy (test use).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wander = amr_wander.wander_node:main',
        ],
    },
)
