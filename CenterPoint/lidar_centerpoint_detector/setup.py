from setuptools import find_packages, setup

package_name = 'lidar_centerpoint_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/launch_cone_detector.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dobayt',
    maintainer_email='dobayt@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cone_detector = lidar_centerpoint_detector.cone_detector:main',
            'cone_detector_multisweep = lidar_centerpoint_detector.cone_detector_multisweep:main',
        ],
    },
)
