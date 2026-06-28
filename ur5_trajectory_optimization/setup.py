from setuptools import setup
import os
from glob import glob

package_name = 'ur5_trajectory_optimization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sergio Morales',
    maintainer_email='smorales@utec.edu.pe',
    description='Multi-objective trajectory optimization for UR5 (CU3)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'run_optimization   = ur5_trajectory_optimization.run_optimization:main',
            'run_singleobjective = ur5_trajectory_optimization.run_singleobjective:main',
            'export_trajectory  = ur5_trajectory_optimization.export_selected_trajectory:main',
            'eval_baseline_cu2  = ur5_trajectory_optimization.eval_baseline_cu2:main',
        ],
    },
)
