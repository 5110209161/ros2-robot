from setuptools import find_packages, setup

package_name = 'ros2_execution_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
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
            'ros2_execution_file = ros2_execution_py.ros2_execution_file:main',
            'ros2_orchestration_client = ros2_execution_py.ros2_orchestration_client:main',
            'ros2_orchestration_server = ros2_execution_py.ros2_orchestration_server:main'
        ],
    },
)
