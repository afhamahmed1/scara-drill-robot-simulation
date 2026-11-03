from setuptools import setup

package_name = 'scara_drill'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Afham Ahmed',
    maintainer_email='afham12ahmed@gmail.com',
    description='Kinematics and nodes that move the SCARA arm and drill holes.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_to = scara_drill.move_to:main',
        ],
    },
)
