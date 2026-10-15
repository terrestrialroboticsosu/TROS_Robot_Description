import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tros_sldworks_pkg'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*')),
    ('share/' + package_name + '/urdf', glob('urdf/*')),
    ('share/' + package_name + '/world', glob('world/*')),
    ('share/' + package_name + '/config', glob('config/*')),
    ('share/' + package_name + '/rviz', glob('rviz/*')),
    ('share/' + package_name + '/meshes', glob('meshes/*')),
]

# Recursively crawl the models directory and add all nested files
for root, dirs, files in os.walk('models'):
    if files:
        install_dir = os.path.join('share', package_name, root)
        file_paths = [os.path.join(root, f) for f in files]
        data_files.append((install_dir, file_paths))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='apostolos-ubuntu-pc',
    maintainer_email='apostolos-ubuntu-pc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)