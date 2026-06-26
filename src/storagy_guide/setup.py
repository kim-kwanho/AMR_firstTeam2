from glob import glob
from setuptools import find_packages, setup

package_name = 'storagy_guide'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/params', glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AMR_firstTeam1',
    maintainer_email='ladawon29@gmail.com',
    description='Guide-side Nav2 mission control for the Toy Guide demo.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'guide_nav_node = storagy_guide.guide_nav_node:main',
        ],
    },
)
