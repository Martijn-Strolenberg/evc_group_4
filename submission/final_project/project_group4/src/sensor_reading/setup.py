from setuptools import setup, find_packages

setup(
    name='sensor_reading',                 # good practice, not used by catkin
    version='0.1.0',
    packages=find_packages('src'),        # == ['sensor_reading', 'sensor_reading.drivers', …]
    package_dir={'': 'src'},              # root of our packages is src/
    install_requires=[],                  # pure ROS deps stay in package.xml
)