from setuptools import find_packages, setup

package_name = 'drone_swarm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/swarm_launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/swarm_world.sdf']),
        ('share/' + package_name + '/scripts', ['scripts/launch_swarm_sitl.sh']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ostap',
    maintainer_email='ostaphutsal04@gmail.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
    "test": ["pytest"],
    },
    entry_points={
        'console_scripts': [
            "drone_swarm_manager = drone_swarm.drone_swarm_manager:main",
            'swarm_coordinator = drone_swarm.swarm_coordinator:main',
            'drone_controller = drone_swarm.drone_controller:main',
            'target_selector = drone_swarm.target_selector:main'
        ],
    },
)
