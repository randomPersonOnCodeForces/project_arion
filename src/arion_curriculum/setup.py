from setuptools import find_packages, setup

package_name = 'arion_curriculum'

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
    maintainer='rdlee',
    maintainer_email='richarddleehk@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'drive_square = arion_curriculum.drive_square:main',
            'stop_at_wall = arion_curriculum.stop_at_wall:main',
        ],
    },
)
