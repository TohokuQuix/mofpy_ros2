from setuptools import find_packages
from setuptools import setup

package_name = "moveit_py_configs_utils"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kazuya Oguma",
    maintainer_email="kzy.basect@gmail.com",
    description="Python library for loading MoveIt config parameters in launch files",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
