from setuptools import setup
import glob

package_name = 'sm130_interrogator_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob.glob('launch/*.launch.*')),
        ('share/' + package_name + "/config", glob.glob('config/*.yaml')), # default parameter files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dimitri Lezcano',
    maintainer_email='dlezcan1@jhu.edu',
    description='Micron Optics sm130 Interrogatory python package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sm130_interrogator = sm130_interrogator_py.sm130_interrogator:main',
            'sm130_demo = sm130_interrogator_py.sm130_interrogator_demo:main',
        ],
    },
)
