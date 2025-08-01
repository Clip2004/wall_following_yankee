from setuptools import find_packages, setup

package_name = 'wall_following_yankee'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', [
        'launch/wall_following_yankee.launch.py',
    ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='clip2004',
    maintainer_email='felipe.mercado59@eia.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dist_finder_yankee = wall_following_yankee.dist_finder_yankee:main',
            'control_yankee = wall_following_yankee.control_yankee:main',
            'AEBSystem = wall_following_yankee.AEBSystem:main',
        ],
    },
)
