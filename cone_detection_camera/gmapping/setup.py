from setuptools import setup

package_name = 'gmapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='szonyibali@gmail.com',
    description='gmapping node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gmapping_node = gmapping.gmapping_node:main'
        ],
    },
)