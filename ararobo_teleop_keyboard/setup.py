from setuptools import find_packages, setup

package_name = 'ararobo_teleop_keyboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lambda',
    maintainer_email='m24366@g.metro-cit.ac.jp',
    description='Keyboard teleoperation for ararobo omni robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = ararobo_teleop_keyboard.teleop_keyboard:main'
        ],
    },
)