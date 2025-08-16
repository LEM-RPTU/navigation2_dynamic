from setuptools import setup
import os, glob

package_name = 'obstacle_predictor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Riyan Cyriac Jose',
    maintainer_email='joseriyancyriac@gmail.com',
    description='ROS 2 predictor node for dynamic obstacles (one-cycle-lag friendly).',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'predictor_node = obstacle_predictor.predictor_node:main',
        ],
    },
)