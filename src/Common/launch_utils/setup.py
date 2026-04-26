from setuptools import find_packages, setup

package_name = 'launch_utils'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amap',
    maintainer_email='kukwonko@gmail.com',
    description='Common ROS2 launch helpers (GPU PRIME offload auto-detect, etc.)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
