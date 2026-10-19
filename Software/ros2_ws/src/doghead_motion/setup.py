from setuptools import find_packages, setup

package_name = 'doghead_motion'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dev',
    maintainer_email='dev@example.com',
    description='Kinematics and motion trajectory generation for doghead arm',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_node = doghead_motion.motion_node:main',
            'trajectory_player = doghead_motion.trajectory_player_node:main',
        ],
    },
)
