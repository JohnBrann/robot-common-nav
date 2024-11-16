from setuptools import find_packages, setup
import glob

package_name = 'robot_common_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch')),
        ('share/' + package_name + '/maps', glob.glob('maps/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='csrobot',
    maintainer_email='john.brann4315@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_navigator = robot_common_nav.waypoint_navigator:main',
            'waypoint_client = robot_common_nav.waypoint_client:main',
        ],
    },
)

