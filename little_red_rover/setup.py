from setuptools import find_packages, setup

package_name = "little_red_rover"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    install_requires=["setuptools"],
    maintainer="Michael Crum",
    maintainer_email="michael@michael-crum.com",
    description="Drivers for the little red rover robotics platform",
    license="LGPL-2.1",
)
