from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sdf_to_gif'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'numpy', 'pillow'],
    zip_safe=True,
    maintainer='ajf',
    maintainer_email='ali.jafari.fesh@gmail.com',
    description='Convert SDF world files to animated GIFs using Ignition Gazebo',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sdf_to_gif_node = sdf_to_gif.sdf_to_gif_node:main',
        ],
    },
)
