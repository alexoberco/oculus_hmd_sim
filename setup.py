from setuptools import find_packages, setup
import glob

package_name = 'oculus_hmd_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf',   glob.glob('urdf/*.urdf')),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
        ('share/' + package_name + '/worlds', glob.glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='HMD yaw-pitch-roll rig in Gazebo Harmonic with ros2_control.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hmd_pose_to_gz_servos = oculus_hmd_sim.hmd_pose_to_gz_servos:main',
        ],
    },
)
