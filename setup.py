from setuptools import find_packages, setup

package_name = 'whale_ctl'

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
            'motor_driver =  whale_ctl.motor_driver_node:main',
            'motor_pwm_toggle = whale_ctl.motor_pwm_toggle:main',
            'motor_pwm_gradient = whale_ctl.motor_pwm_gradient:main',
            'motor_pwm_button_control = whale_ctl.motor_pwm_button_control:main',
            'multi_motor_control = whale_ctl.multi_motor_control:main',
            'actuator_driver = whale_ctl.actuator_driver:main',
            'a1_motor_test = whale_ctl.a1_motor_test:main',
        ],
    },
)
