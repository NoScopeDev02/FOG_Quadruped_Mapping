from setuptools import find_packages, setup

package_name = 'legged_height_map'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='harshal',
    maintainer_email='harshghadge51@gmail.com',
    description='FOG Mapping Algorithm',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mapping_node = legged_height_map.mapping_node:main',
            'pcd_saver_node = legged_height_map.pcd_saver_node:main'
        ],
    },
)