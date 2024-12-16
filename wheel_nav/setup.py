from setuptools import find_packages, setup
import glob

package_name = 'wheel_nav'

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
            'waypoint_navigator = wheel_nav.waypoint_navigator:main',
            'waypoint_client = wheel_nav.waypoint_client:main',
            'simple_nav_bt = wheel_nav.simple_nav_bt:main',
            'patrol_a_bt = wheel_nav.patrol_a_bt:main',
            'rl_state = wheel_nav.rl_state:main',
            'rl_step = wheel_nav.rl_step_service:main',
            'reward = wheel_nav.reward:main',
            'agent = wheel_nav.agent:main',
        ],
    },
)

