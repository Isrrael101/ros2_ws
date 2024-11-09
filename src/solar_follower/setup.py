from setuptools import setup
import os
from glob import glob

package_name = 'solar_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Agregar las carpetas de configuración
        (os.path.join('share', package_name, 'config'),
         glob('config/*')),
        (os.path.join('share', package_name, 'description'),
         glob('description/*')),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*')),
        (os.path.join('share', package_name, 'worlds'),
         glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Solar panel follower package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'solar_tracker = solar_follower.solar_tracker_node:main'
        ],
    },
)