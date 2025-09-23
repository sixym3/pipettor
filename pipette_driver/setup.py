from setuptools import find_packages, setup

package_name = 'pipette_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eric',
    maintainer_email='xiao.yuan0217@gmail.com',
    description='Pipette driver with ros2_control hardware interface',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pipette_driver_node = pipette_driver.pipette_driver_node:main',
            'serial_terminal = pipette_driver.serial_terminal:cli_main',
            'joint_state_bridge = pipette_driver.joint_state_bridge:main'
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
)
