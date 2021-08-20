import setuptools

with open("README.md", "r") as readme:
    long_description = readme.read()

setuptools.setup(
    name="BSTPythonSDK",
    version="3.16.0.dev4",
    author="Black Swift Technologies",
    author_email="ben.busby@blackswifttech.com",
    description="BST Flight Management SDK",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/bstaero/sdk",
    packages=setuptools.find_packages(),
    classifiers=(
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: GNU General Public License v2 (GPLv2)",
        "Operating System :: OS Independent",
    )
)
