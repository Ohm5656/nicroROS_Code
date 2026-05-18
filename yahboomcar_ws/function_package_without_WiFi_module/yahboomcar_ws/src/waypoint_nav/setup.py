from setuptools import find_packages, setup

package_name = 'waypoint_nav'

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
    maintainer='natdanai',
    maintainer_email='natdanai@todo.todo',
    description='Waypoint collection and waypoint running for Nav2',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'collect_points = waypoint_nav.collect_points:main',
            'run_waypoints = waypoint_nav.run_waypoints:main',
            'check_points = waypoint_nav.check_points:main',
        ],
    },
)
