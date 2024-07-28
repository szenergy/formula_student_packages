from setuptools import setup

package_name = 'detection_onnx'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='szonyibali@gmail.com',
    description='detection onnx model',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_onnx_node = detection_onnx.detection_onnx_node:main'
        ],
    },
)