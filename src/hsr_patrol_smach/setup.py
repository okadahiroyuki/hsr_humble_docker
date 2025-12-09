from setuptools import setup

package_name = 'hsr_patrol_smach'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='SMACH-based patrol state machine for HSR on ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run hsr_patrol_smach hsr_patrol_smach_node
            'hsr_patrol_smach_node = hsr_patrol_smach.patrol_state_machine:main',
        ],
    },
)
