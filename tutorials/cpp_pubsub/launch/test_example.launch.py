import unittest

import launch
import launch.actions
import launch_testing
import launch_testing.actions
from launch_ros.actions import Node


def generate_test_description():
    """Creates launchfile description for the test

    Returns
    -------
        LaunchDescription: launchfile description

    """
    cpp_pubsub_test = Node(package="cpp_pubsub", executable="test_example", output="screen")

    return launch.LaunchDescription(
        [cpp_pubsub_test, launch_testing.util.KeepAliveProc(), launch_testing.actions.ReadyToTest()]
    ), locals()


class PubSubTest(unittest.TestCase):
    """Test class"""

    def test_termination(self, cpp_pubsub_test, proc_info):
        """Asserts the simulation finishes correctly"""
        proc_info.assertWaitForShutdown(process=cpp_pubsub_test, timeout=200)


@launch_testing.post_shutdown_test()
class PubSubTestAfterShutdown(unittest.TestCase):
    """Test class"""

    def test_exit_code(self, cpp_pubsub_test, proc_info):
        """Asserts the simulation finishes with the correct exit code"""
        launch_testing.asserts.assertExitCodes(proc_info, [launch_testing.asserts.EXIT_OK], cpp_pubsub_test)
