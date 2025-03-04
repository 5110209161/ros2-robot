from setuptools import find_packages, setup

package_name = 'robot_palletizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'spawn_pallet = robot_palletizer.spawn_pallet:main',
            'dynamic_stack_spawner = robot_palletizer.dynamic_stack_spawner:main'
        ],
    },
)
