def pytest_launch_collect_makemodule(path, parent, entrypoint):  # noqa
    marks = getattr(entrypoint, "pytestmark", [])
    if marks and any(m.name == "rostest" for m in marks):
        from launch_testing_ros.pytest.hooks import LaunchROSTestModule

        return LaunchROSTestModule.from_parent(parent=parent, fspath=path)
    return None


def pytest_configure(config):  # noqa
    config.addinivalue_line(
        "markers", "rostest: mark a generate_test_description function as a ROS launch test entrypoint"
    )
