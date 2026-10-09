from setuptools import find_packages, setup

package_name = "so_arm_calibration"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jafar Uruc",
    maintainer_email="jafar.uruc@gmail.com",
    description="Joint zero calibration for the SO-ARM101",
    license="BSD",
    entry_points={
        "console_scripts": [
            "calibrate_zeros = so_arm_calibration.calibrate_zeros:main",
            "apply_zeros = so_arm_calibration.apply_zeros:main",
        ],
    },
)
