from setuptools import setup
import os
from glob import glob

package_name = 'obstacle_avoidance'

setup(
    name=package_name,
    version='0.0.0',
    # no need for `packages=[]` since we are using py_modules
    py_modules=[
        'Depth_process',
        'obsavoid',
    ],
    package_dir={
        '': 'src',  # <-- this tells setuptools where the py_modules are
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/obstacle_avoidance']),
        ('share/' + package_name, ['package.xml']),
        # Include all files in launch folder recursively
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/**/*.py', recursive=True) + glob('launch/**/*.xml', recursive=True)),
        ],
    install_requires=[
        'setuptools',
        'numpy',
        'scikit-image',
        'opencv-python',
        'scikit-image',
    ],
    zip_safe=True,
    maintainer='KiheonKwak',
    maintainer_email='kiheonkwak@kookmin.ac.kr',
    description='Obstacle avoidance nodes for autonomous driving',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Depth_processe = Depth_process:main',
            'obsavoid = obsavoid:main',
        ],
    },
)


