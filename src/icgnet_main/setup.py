import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'icgnet_main'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'models', 'coke_can'), glob('models/coke_can/*')),
        (os.path.join('share', package_name, 'config', 'moveit'), glob('config/moveit/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='burger',
    maintainer_email='burger@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'grasp_service_node = icgnet_main.grasp_service_node:main',
            'test_move_to_pose = icgnet_main.test_move_to_pose:main',
            'direct_jtc_test = icgnet_main.direct_jtc_test:main',
            'spawn_object = icgnet_main.spawn_object:main',
            'spawn_one_entity = icgnet_main.spawn_one_entity:main',
        ],
    },
)
