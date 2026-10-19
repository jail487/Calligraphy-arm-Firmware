from setuptools import find_packages, setup

package_name = 'doghead_hil'

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
    description='HIL testing and telemetry validation for doghead arm',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'telemetry_validator = doghead_hil.telemetry_validator:main',
            'scenario_runner = doghead_hil.scenario_runner:main',
            'tracking_report = doghead_hil.tracking_report:main',
        ],
    },
)
