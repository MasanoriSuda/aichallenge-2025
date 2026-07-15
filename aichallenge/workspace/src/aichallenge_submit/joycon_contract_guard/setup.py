from setuptools import find_packages, setup


package_name = "joycon_contract_guard"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Automotive AI Challenge Team",
    maintainer_email="maintainer@example.com",
    description="Participant-side guard for the contracted AWSIM Joy-Con boundary.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joycon_contract_guard_node = joycon_contract_guard.node:main",
        ],
    },
)
