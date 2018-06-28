from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
		packages=['riptide_utilities'],
		package_dir={'': 'src/riptide_utilities'},
		)
setup(**setup_args)