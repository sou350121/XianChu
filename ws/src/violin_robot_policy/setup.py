from setuptools import setup


package_name = "violin_robot_policy"


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
    description="Policy stub (v0 maps music_spec to command_raw JSON).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "policy_stub_node = violin_robot_policy.policy_stub_node:main",
        ],
    },
)

