from setuptools import setup

package_name = 'pymavlink_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joel',
    maintainer_email='tu@email.com',
    description='Paquete ROS2 Python',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manage_unity = pymavlink_ros.manage_unity:main'
        ],
    },
)

