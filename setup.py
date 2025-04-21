from setuptools import find_packages, setup

package_name = 'raspike_ros_imu_publisher'

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
    maintainer='KatsumiOuwa',
    maintainer_email='d2p.yggdrasill@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'raspike_ros_imu_publisher = raspike_ros_imu_publisher.raspike_ros_imu_publisher:main'
        ],
    },
)
