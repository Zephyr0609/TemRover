"""Package build file: installs the Python package, launch files, configs, models, worlds and the web page.
"""
from glob import glob

from setuptools import find_packages, setup

package_name = 'scout_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Everything the launch files load at run time is installed into the package's share folder
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/description', glob('description/*.xacro')),
        ('share/' + package_name + '/worlds', glob('worlds/*.*')),
        ('share/' + package_name + '/worlds/royal_park', glob('worlds/royal_park/*')),
        ('share/' + package_name + '/web', glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sumitjadhav',
    maintainer_email='sumitsantosh@student.unimelb.edu.au',
    description='Autonomous survey-line navigation for the TEMRover: Scout 2.0 towing the EM Loupe Tx and Rx carts',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        # Nodes started with `ros2 run scout_navigation <name>`
        'console_scripts': [
            'survey_navigator = scout_navigation.survey_navigator:main',
            'scout_can_bridge = scout_navigation.scout_can_bridge:main',
            'imu_driver = scout_navigation.imu_driver:main',
        ],
    },
)
