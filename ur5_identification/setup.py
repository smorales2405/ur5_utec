from setuptools import setup
import os
from glob import glob

package_name = 'ur5_identification'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('lib', package_name), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sergio Morales',
    maintainer_email='smorales@utec.edu.pe',
    description='Offline joint friction identification for the UR5e (FASE 2)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_identification = ur5_identification.run_identification:main',
        ],
    },
)
