import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'datagen_scripts'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        #(os.path.join('share', package_name), glob('config/*')),
        (os.path.join('share', package_name), glob('moveit_configs/*')),
        (os.path.join('share', package_name), glob('gz_configs/launch_params.yaml')),
        (os.path.join('share', package_name), glob('nav2_configs/*')),
        (os.path.join('share', package_name), glob('map/*')),
        (os.path.join('share', package_name), glob('rviz/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Danilo G. Schneider',
    maintainer_email='danilo_gsch@hotmail.com',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'map_restricter = datagen_scripts.map_restricter:main',
                'moveit2 = datagen_scripts.moveit2:main',
                'test_pose_goal = datagen_scripts.test_pose_goal:main',
                'test_pose_goal_5 = datagen_scripts.test_pose_goal_5:main',
                'random_goal_handler = datagen_scripts.random_goal_handler:main',
        ],
    },
)
