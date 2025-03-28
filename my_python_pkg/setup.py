from setuptools import find_packages, setup

package_name = 'my_python_pkg'

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
    maintainer='mj',
    maintainer_email='marcus.j.hsieh@gmail.com',
    description='Python version: Talker and listener node',
    license='MIT',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # add a new entry point for each executable in the package
            # e.g. run main() in my_python_pkg/talker.py
            'talker = my_python_pkg.talker:main', 
            'listener = my_python_pkg.listener:main',
        ],
    },
)
