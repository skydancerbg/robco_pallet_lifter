from setuptools import find_packages, setup

package_name = 'robco_pallet_lifter'

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
    maintainer='robco',
    maintainer_email='robco@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lifter_server = robco_pallet_lifter.lifter_server:main',
            'lifter_joy_control = robco_pallet_lifter.lifter_joy_control:main',
            'lifter_topic_control = robco_pallet_lifter.lifter_topic_control:main',

        ],
    },
)
