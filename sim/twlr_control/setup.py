from glob import glob
from setuptools import setup

package_name = 'twlr_control'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jungwonseok',
    maintainer_email='jhkx28@gmail.com',
    description='TWLR balance controller',
    license='MIT',
    entry_points={'console_scripts': [
        'balance_node = twlr_control.balance_node:main',
    ]},
)
