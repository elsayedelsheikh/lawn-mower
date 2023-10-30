from setuptools import find_packages, setup

package_name = 'fws_base_controller'

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
    maintainer='sayed',
    maintainer_email='sayed@todo.todo',
    description='Controller for 4WS Mobile Robot driving',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = fws_base_controller.controller:main'
        ],
    },
)
