from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros2_livox_cloudcompare'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'test'), glob(os.path.join('test', '*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mickey',
    maintainer_email='mickey.li@ucl.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_livox_cloudcompare = ros2_livox_cloudcompare.ros2_livox_cloudcompare:main'
        ],
    },
)
