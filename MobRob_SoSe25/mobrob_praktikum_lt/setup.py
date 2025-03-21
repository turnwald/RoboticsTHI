from setuptools import find_packages, setup

package_name = 'mobrob_praktikum_lt'

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
    maintainer='turnwald',
    maintainer_email='alen.turnwald@thi.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_world = mobrob_praktikum_lt.hello_world:main',
            'hello_world_ros = mobrob_praktikum_lt.hello_world_ros:main',
            'mein_talker = mobrob_praktikum_lt.talker:main',
            'mein_talker_oop = mobrob_praktikum_lt.talker_oop:main',
            'mein_listener = mobrob_praktikum_lt.listener:main',
            'mein_listener_oop = mobrob_praktikum_lt.listener_oop:main'
        ],
    },
)
