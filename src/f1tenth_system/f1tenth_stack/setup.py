from setuptools import setup
import os
from glob import glob

package_name = 'f1tenth_stack'
submodule = 'f1tenth_stack/auto'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, submodule],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hongrui Zheng',
    maintainer_email='billyzheng.bz@gmail.com',
    description='Onboard drivers for vesc and sensors for F1TENTH vehicles.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'throttle_interpolator = f1tenth_stack.throttle_interpolator:main',
            'tf_publisher = f1tenth_stack.tf_publisher:main',
			'wall_follower = f1tenth_stack.auto.wall_follow:main',
			'wall_follower_nosafe = f1tenth_stack.auto.wall_follow_nosafebrake:main',
			'reset_safe_brake = f1tenth_stack.auto.reset_safe_brake:main',
            'pure_pursuit = f1tenth_stack.auto.pure_pursuit:main',
            'ftg = f1tenth_stack.auto.follow_the_gap:main',
        ],
    },
)
