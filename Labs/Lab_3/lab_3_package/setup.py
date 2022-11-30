from setuptools import setup

package_name = 'lab_3_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='riyad0autobots',
    maintainer_email='rr3khan@uwaterloo.ca',
    description='Lab 3 package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_sub_node = lab_3_package.mp_sub:main',
            'drive_testing = lab_3_package.drive_robot:main',
            'robot_controller = lab_3_package.robotController:main'
        ],
    },
)
