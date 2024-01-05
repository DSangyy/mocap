from setuptools import setup
from Cython.Build import cythonize

setup(
    ext_modules=cythonize("mathfunc/quatmath.pyx")
)

setup(
    name='mocap',
    version='0.0.1',
    packages=['mocap'],
    url='',
    license='',
    author='sang',
    author_email='',
    description=''
)
