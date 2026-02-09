from setuptools import setup


package_name = "violin_robot_drivers"


setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="todo",
    maintainer_email="todo@example.com",
    description="Mock and real hardware drivers (v0 uses mock).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "driver_mock_node = violin_robot_drivers.driver_mock_node:main",
        ],
    },
)

