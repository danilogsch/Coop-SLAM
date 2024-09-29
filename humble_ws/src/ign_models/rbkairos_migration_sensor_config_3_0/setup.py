import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'rbkairos_migration_sensor_config_3_0'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        #(os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name), glob('params/*')),
        (os.path.join('share', package_name,'materials/textures'), glob('materials/textures/*')),
        (os.path.join('share', package_name,'rbkairos_migration_sensor_config_3_0_0/materials/textures'), glob('materials/textures/*')),
        (os.path.join('share', package_name,'rbkairos_migration_sensor_config_3_0_1/materials/textures'), glob('materials/textures/*')),
        (os.path.join('share', package_name,'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name,'rbkairos_migration_sensor_config_3_0_0/meshes'), glob('meshes/*')),
        (os.path.join('share', package_name,'rbkairos_migration_sensor_config_3_0_1/meshes'), glob('meshes/*')),
        (os.path.join('share', package_name), glob('model.sdf')),
        (os.path.join('share', package_name), glob('urdf/*.urdf')),
        (os.path.join('share', package_name), glob('urdf/*.txt'))
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
