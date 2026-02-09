from setuptools import setup


package_name = "violin_robot_control"


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
    description="Tactile guard (v0 gates command_raw -> command and raises e-stop).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "tactile_guard_node = violin_robot_control.tactile_guard_node:main",
        ],
    },
)

