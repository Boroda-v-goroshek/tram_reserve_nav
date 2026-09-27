from setuptools import find_packages, setup


package_name = "hackathon_solution_checker"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hackathon maintainers",
    maintainer_email="maintainer@example.com",
    description="Online metrics for localization hackathon solutions.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "metrics = hackathon_solution_checker.metrics:main",
        ],
    },
)
