from setuptools import setup, find_packages

setup(
    name='motor_control',                 # good practice, not used by catkin
    version='0.1.0',
    packages=find_packages('src'),        # == ['motor_control', 'motor_control.drivers', …]
    package_dir={'': 'src'},              # root of our packages is src/
    install_requires=[],                  # pure ROS deps stay in package.xml
)