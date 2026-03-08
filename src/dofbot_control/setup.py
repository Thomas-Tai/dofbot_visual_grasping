from setuptools import find_packages, setup

package_name = 'dofbot_control'

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
    maintainer='nv-sky',
    maintainer_email='thomastai.uni@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test_motion = dofbot_control.test_motion:main',
            'test_cartesian = dofbot_control.test_cartesian:main',
        ],
    },
)
