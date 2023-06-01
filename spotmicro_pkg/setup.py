from setuptools import setup

package_name = 'spotmicro_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='park',
    maintainer_email='park@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'communication = spotmicro_pkg.communication.main:main',
            'motion_control = spotmicro_pkg.motion_control.motion_control:main',
            'servo_control = spotmicro_pkg.motion_control.servo_controller:main',
            'motion_simulation = spotmicro_pkg.motion_simulation.main:main',
        ],
    },
)
