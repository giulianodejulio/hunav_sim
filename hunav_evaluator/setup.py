from setuptools import setup
import os
from glob import glob

package_name = 'hunav_evaluator'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('lib', 'python3.8', 'dist-packages', package_name), glob('hunav_evaluator/*.py')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'rospy',  # ROS 1 Python library
        'std_msgs',
        'geometry_msgs',
        'hunav_msgs',
    ],
    zip_safe=True,
    maintainer='Noé Pérez-Higueras',
    maintainer_email='noeperez@upo.es',
    description='This package collects the data of the hunav simulations and computes different metrics',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hunav_evaluator_node = hunav_evaluator.hunav_evaluator_node:main'
        ],
    },
)
