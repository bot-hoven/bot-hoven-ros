from setuptools import find_packages, setup

package_name = 'description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/bothoven.urdf.xacro']),
        ('share/' + package_name + '/launch', ['launch/gazebo.launch.py']),
        ('share/' + package_name + '/launch', ['launch/rsp.launch.py']),
        ('share/' + package_name + '/description', ['description/bothoven_core.xacro']),
        ('share/' + package_name + '/description', ['description/gazebo_control.xacro']),
        ('share/' + package_name + '/description', ['description/common.xacro']),
        ('share/' + package_name + '/worlds', ['worlds/bothoven_world.sdf']),
        ('share/' + package_name + '/resource', ['resource/rviz2_config.rviz']),
        ('share/' + package_name + '/models/bothoven', ['models/bothoven/model.config']),
        ('share/' + package_name + '/models/bothoven', ['models/bothoven/model.sdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='tonyzaky.s@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
