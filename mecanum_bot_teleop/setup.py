from setuptools import setup

package_name = 'mecanum_bot_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name,
        'mecanum_bot_controller'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yadu',
    maintainer_email='yadunund@gmail.com',
    description='A package to teleoperate the mecanum bot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'teleop_node = mecanum_bot_teleop.teleop_node:main' 
        ],
    },
)
