from glob import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = "mofpy"

# build a list of the data files
data_files = []
data_files.append(("share/ament_index/resource_index/packages", ["resource/" + package_name]))
data_files.append(("share/" + package_name, ["package.xml"]))


def package_files(directory, data_files):
    for path, directories, filenames in os.walk(directory):
        for filename in filenames:
            data_files.append(
                ("share/" + package_name + "/" + path, glob(path + "/**/*.*", recursive=True))
            )
    return data_files


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kazuya Oguma",
    maintainer_email="kzy.basect@gmail.com",
    description="Mofpy is a ROS node that lets you to generically trigger actions from joypad inputs.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["mofpy_node = mofpy.mofpy_node:main"],
    },
)
