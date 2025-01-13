import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'dobot_ros2_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aar',
    maintainer_email='hbin6358@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'dobot_bringup  = dobot_ros2_driver.dobot_bringup:main',
          'feedback  = dobot_ros2_driver.feedback:main',
        ],
    },
)
