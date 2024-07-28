from setuptools import setup

package_name = 'deprojection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='szonyibali@gmail.com',
    description='DeprojectionFS',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'deprojection_node = deprojection.deprojection_node:main'
        ],
    },
)