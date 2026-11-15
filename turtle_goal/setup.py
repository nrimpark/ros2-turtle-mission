from glob import glob
from setuptools import find_packages, setup

package_name = 'turtle_goal'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='narim',
    maintainer_email='narim@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mission_controller = turtle_goal.mission_controller:main',
            'mission_environment = turtle_goal.mission_environment:main',
            'mission_goal_publisher = turtle_goal.mission_goal_publisher:main',
            'turtle_controller = turtle_goal.turtle_controller:main',
            'goal_publisher = turtle_goal.goal_publisher:main',
        ],
    },
)
