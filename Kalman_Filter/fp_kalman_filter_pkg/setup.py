from setuptools import setup

package_name = 'fp_kalman_filter_pkg'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'fp_bagrun = fp_kalman_filter_pkg.fp_bagrun:main'
        ],
    },
)
