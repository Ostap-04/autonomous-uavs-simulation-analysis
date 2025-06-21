from setuptools import find_packages, setup
import os

package_name = 'drone_swarm'

def package_files(directory):
    return [os.path.join(dp, f) for dp, dn, filenames in os.walk(directory) for f in filenames]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', package_files('launch')),
        ('share/' + package_name + '/worlds', package_files('worlds')),
        ('share/' + package_name + '/scripts', package_files('scripts')),
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
            # 'target_selector = drone_swarm.target_selector:main',
            'square_formation_controller = drone_swarm.square_formation_controller:main',
            'target_attack_controller = drone_swarm.target_attack_controller:main',
            'target_selector = drone_swarm.target_selector:main',
            'algorithm_coordinator = drone_swarm.algorithm_coordinator:main',
        ],
    },
)
