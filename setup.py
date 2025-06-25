from setuptools import find_packages, setup

package_name = 'motor_driver'

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
    maintainer='rmfrescue',  # 任意で変更
    maintainer_email='rmfrescue@example.com',  # 任意で変更
    description='ROS2 Humble motor driver for MD10C R3 on Raspberry Pi',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver = motor_driver.motor_driver_node:main',
            'motor_pwm_toggle = motor_driver.motor_pwm_toggle:main',
            'motor_pwm_gradient = motor_driver.motor_pwm_gradient:main',
            'motor_pwm_button_control = motor_driver.motor_pwm_button_control:main',
            'multi_motor_control = motor_driver.multi_motor_control:main',
        ],
    },
)
