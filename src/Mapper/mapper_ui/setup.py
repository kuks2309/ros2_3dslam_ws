from setuptools import setup
import os
from glob import glob

package_name = 'mapper_ui'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'ui'), glob('ui/*.ui')),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'mapper_ui_node = mapper_ui.mapper_ui_node:main',
        ],
    },
)
