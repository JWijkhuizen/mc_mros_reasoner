import os
from glob import glob
from setuptools import setup

package_name = 'mros2_reasoner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['owl/tomasys.owl']),
        ('share/' + package_name, ['owl/test1.owl']),
        ('share/' + package_name, ['owl/kb.owl']),
         # Include all launch files. This is the most important line here!
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=[
                'setuptools',
                'owlready2>=0.24'],
    zip_safe=True,
    maintainer='Carlos Hernandez Corbato',
    maintainer_email='c.h.corbato@tudelft.nl',
    description='Implements the meta-controller node for ROS2, including the reasoner',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mros2_reasoner_node = mros2_reasoner.mros2_reasoner_node:main'
        ],
    },
)