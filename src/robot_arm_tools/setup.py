from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
setup_args = generate_distutils_setup(
    packages=['robot_arm_tools'],
    package_dir={'': 'src'},
    install_requires=["urchin"]
)
setup(**setup_args)