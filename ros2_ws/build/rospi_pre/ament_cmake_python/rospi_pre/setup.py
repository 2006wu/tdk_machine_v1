from setuptools import find_packages
from setuptools import setup

setup(
    name='rospi_pre',
    version='0.0.0',
    packages=find_packages(
        include=('rospi_pre', 'rospi_pre.*')),
)
