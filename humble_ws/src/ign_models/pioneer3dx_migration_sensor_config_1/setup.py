import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'pioneer3dx_migration_sensor_config_1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('urdf/*.urdf')),
        (os.path.join('share', package_name), glob('model.sdf')),
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
