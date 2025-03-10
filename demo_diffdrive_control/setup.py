from setuptools import find_packages, setup

package_name = 'demo_diffdrive_control'

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
    maintainer='root',
    maintainer_email='Alen.Turnwald@efs-auto.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo_control_node = demo_diffdrive_control.robot_controller:main',
            'simple_control_node = demo_diffdrive_control.simple_controller:main',
        ],
    },
)
