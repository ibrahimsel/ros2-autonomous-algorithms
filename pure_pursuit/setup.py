from setuptools import setup
from glob import glob
import os
package_name = 'pure_pursuit'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ibrahimsel',
    maintainer_email='ibrahim.sel@eteration.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker_publisher = pure_pursuit.marker_publisher:main',
            'pure_pursuit = pure_pursuit.pure_pursuit:main',
            'waypoint_logger = pure_pursuit.waypoint_logger:main'
        ],
    },
)
