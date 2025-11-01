from setuptools import setup
import os, glob

package_name = 'obstacle_predictor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Riyan Cyriac Jose',
    maintainer_email='joseriyancyriac@gmail.com',
    description='ROS 2 predictor node for dynamic obstacles (one-cycle-lag friendly).',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'predictor_node_cv = obstacle_predictor.predictor_node_cv:main',
            'predictor_node_kf = obstacle_predictor.predictor_node_kf:main',
            'predictor_node_var = obstacle_predictor.predictor_node_var:main'
        ],
    },
)