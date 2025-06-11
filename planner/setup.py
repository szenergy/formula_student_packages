from setuptools import setup
import os
from glob import glob

package_name = 'planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files if they exist
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include config files if they exist
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='szonyibalazs',
    maintainer_email='user@todo.todo',
    description='Formula Student path planning package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ft_fsd_node = planner.ft_fsd:main',
        ],
    },
)
