from setuptools import setup


setup(
    name="xuegeros-agent-control",
    version="0.1.0",
    packages=["agent_control"],
    package_dir={"agent_control": "."},
    package_data={"agent_control": ["skills.json"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/agent_control"]),
        ("share/agent_control", ["package.xml"]),
        ("share/agent_control", ["skills.json"]),
        ("share/agent_control/config", ["config/agent_mapping.yaml"]),
        ("share/agent_control/launch", ["launch/agent_mapping.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "mapping_controller = agent_control.ros_mapping_controller:main",
            "robotd = agent_control.ros_cerebellum:main",
            "agent_plugin = agent_control.plugin_server:main",
        ]
    },
)
