from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['lcastor_navigation'],
    package_dir={'': 'srv'}
)
setup(**d)