from setuptools import find_packages, setup

package_name = "path_storage"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["launch/path_storage.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wavem",
    maintainer_email="ys.kwon@wavem.net",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "path_storage_manager = path_storage.main:main",
        ],
    },
)
