import unittest

from other_programs.sirius_launcher.process_manager import ProcessManager


class TestProcessManager(unittest.TestCase):
    def test_detects_ros2_bag_record_command(self):
        self.assertTrue(
            ProcessManager._is_rosbag_record_command(
                ["/opt/ros/jazzy/bin/ros2", "bag", "record", "-s", "mcap"]
            )
        )

    def test_does_not_treat_bag_info_as_recording(self):
        self.assertFalse(
            ProcessManager._is_rosbag_record_command(
                ["/opt/ros/jazzy/bin/ros2", "bag", "info", "/tmp/bag"]
            )
        )

    def test_detects_offline_recorder_scripts(self):
        self.assertTrue(
            ProcessManager._is_offline_recorder_script(
                ["bash", "/workspace/bash/startup_bash/record_rosbag_offline.sh"]
            )
        )
        self.assertTrue(
            ProcessManager._is_offline_recorder_script(
                ["bash", "/workspace/bash/startup_bash/record_rosbag_offline_real.sh"]
            )
        )


if __name__ == "__main__":
    unittest.main()
