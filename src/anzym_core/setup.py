from setuptools import find_packages, setup

package_name = 'anzym_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/joystick_control_launch.py',
            'launch/bringup_launch.py',
            'launch/camera_launch.py',
            'launch/slam_launch.py'
        ]),
        ('share/' + package_name + '/config', [
            'config/ydlidar.yaml',
            'config/mapper_params_online_async.yaml',
            'config/ekf_params.yaml'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pcarff',
    maintainer_email='pcarff@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'driver_node = anzym_core.driver_node:main',
        ],
    },
)
