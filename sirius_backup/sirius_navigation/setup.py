from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sirius_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kotantu-desktop',
    maintainer_email='Frog7352@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cancel_navigation = sirius_navigation.cancel_navigation:main',
            'assisted_teleop = sirius_navigation.assisted_teleop:main',
            'ekf_pose_initializer = sirius_navigation.ekf_pose_initializer:main',
        ],
    },
)
