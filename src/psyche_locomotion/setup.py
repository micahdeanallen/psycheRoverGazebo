from setuptools import setup

package_name = 'psyche_locomotion'

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
    maintainer='Psyche Rover Team',
    maintainer_email='todo@asu.edu',
    description='Leg control and gait for the Psyche hexapod rover.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'stand_node = psyche_locomotion.stand_node:main',
            'gait_node = psyche_locomotion.gait_node:main',
        ],
    },
)
