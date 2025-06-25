from setuptools import setup, find_packages, Extension
from setuptools.command.build_ext import build_ext
import sys
import os
import shutil

class SwigBuildExt(build_ext):
    def build_extensions(self):
        if not shutil.which("swig"):
            sys.stderr.write("Error: SWIG is required to build this package. Install it and retry.\n")
            sys.exit(1)

        wrapper = "bst_python_sdk/swig_parser_wrap.cxx"
        interface = "bst_python_sdk/swig_parser.i"
        swig_cmd = f"swig -python -c++ -o {wrapper} {interface}"
        print(f"SWIG build: {swig_cmd}")
        if os.system(swig_cmd) != 0:
            sys.stderr.write("SWIG compilation failed!\n")
            sys.exit(1)

        super().build_extensions()

# Define the SWIG extension
parser_module = Extension(
    "bst_python_sdk._swig_parser",
    sources=[
        "bst_python_sdk/swig_parser_wrap.cxx",
        "bst_python_sdk/swig_parser.cpp"],
    extra_compile_args=["-std=c++11"])

with open("README.md", "r") as readme:
    long_description = readme.read()

setup_args = dict(
    name="BSTPythonSDK",
    version="3.22.0.dev16",
    author="Black Swift Technologies",
    author_email="ben.busby@blackswifttech.com",
    description="BST Flight Management SDK",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/bstaero/sdk",
    packages=find_packages(),
    install_requires=["numpy", "scipy", "h5netcdf", "lxml"],
    ext_modules=[parser_module],
    cmdclass={"build_ext": SwigBuildExt},
    py_modules=["swig_parser"],
    setup_requires=["setuptools", "wheel", "swig"],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: GNU General Public License v2 (GPLv2)",
        "Operating System :: OS Independent",
    ],
)

setup(**setup_args)
