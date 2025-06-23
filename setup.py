from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'whale_ctl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name, 'img'), glob('img/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hibiki',
    maintainer_email='144889491+MAMMARU-class@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "cam_fb = whale_ctl.cam_fb:main",
        ],
    },
)
