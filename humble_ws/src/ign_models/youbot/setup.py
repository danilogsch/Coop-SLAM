import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'youbot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'meshes/arm'), glob('meshes/arm/*')),
        (os.path.join('share', package_name,'meshes/base'), glob('meshes/base/*')),
        (os.path.join('share', package_name,'meshes/gripper'), glob('meshes/gripper/*')),
        (os.path.join('share', package_name,'meshes/plate'), glob('meshes/plate/*')),
        (os.path.join('share', package_name,'meshes'), glob('meshes/hokuyo.dae')),
        (os.path.join('share', package_name,'meshes'), glob('meshes/hokuyo_convex.stl')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='danilo',
    maintainer_email='danilo_gsch@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
