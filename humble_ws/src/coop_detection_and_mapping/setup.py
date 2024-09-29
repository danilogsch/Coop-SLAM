from setuptools import find_packages, setup

package_name = 'coop_detection_and_mapping'

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
    maintainer='Danilo G. Schneider',
    maintainer_email='danilo_gsch@hotmail.com',
    description='Object detection and Semantic segmentation nodes coop Mapping in dynamic environments',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'detection_publisher = coop_detection_and_mapping.detection_publisher:main',
        	'centralized_ogm_builder = coop_detection_and_mapping.centralized_ogm_builder:main',
        	'groundtruth_gen = coop_detection_and_mapping.groundtruth_gen:main',
        	'human_data_saver = coop_detection_and_mapping.human_data_saver:main',
        ],
    },
)
