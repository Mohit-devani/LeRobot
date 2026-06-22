from setuptools import find_packages, setup

package_name = 'rviz_objects'

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
    maintainer='mohit-devani',
    maintainer_email='robotics7202@gmail.com',
    description='RViz object marker package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cube_marker = rviz_objects.cube_marker:main',
        ],
    },
)
