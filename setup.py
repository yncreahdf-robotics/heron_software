#!/usr/bin/env python3
# coding: utf-8


from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=["tutorial_package"],
    package_dir={"": "src"},
)
setup(**setup_args)


if __name__ == "__main__":
    try:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        pass
