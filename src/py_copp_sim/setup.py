from setuptools import setup

package_name = 'py_copp_sim'

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
    maintainer='dyxcvi',
    maintainer_email='dy.xcvi@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = py_copp_sim.N1:main',
            'path_generator = py_copp_sim.N2:main'
        ],
    },
)
