from setuptools import find_packages
from setuptools import setup

package_name = 'greengrass_bridge'

setup(
    name=package_name,
    version='0.14.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/greengrass_bridge.launch.py', ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='AWS RoboMaker',
    author_email='ros-contributions@amazon.com',
    maintainer='AWS RoboMaker',
    maintainer_email='ros-contributions@amazon.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: MIT0',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Collecting stats and printing to the OLED display'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'greengrass_bridge = greengrass_bridge.app:main'
        ],
    },
)
