import os
import sys
from pathlib import Path
import unittest

# Ensure the project root is on sys.path so imports work
sys.path.insert(0, str(Path.cwd()))
sys.path.insert(0, str(Path.cwd().joinpath('other_programs', 'sirius_launcher')))

from PySide6.QtWidgets import QApplication
from PySide6.QtGui import QMouseEvent
from PySide6.QtCore import Qt, QPoint

from other_programs.sirius_launcher.sirius_launcher import SiriusLauncher


class TestTabSelection(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ["QT_QPA_PLATFORM"] = "offscreen"
        cls.app = QApplication([])

    def test_select_tab_on_click_running(self):
        window = SiriusLauncher()
        layout, group_widget = window.add_group('TestGroup', tab_name='センサー・ハードウェア')
        window.add_button(layout, 'Test', 'echo "Test"', 'desc', group_widget)
        btn = window.buttons[-1]
        # Force the process_manager to appear running
        btn.process_manager.is_running = lambda: True

        # Simulate mouse press on the launch button (child widget event filter should catch it)
        event = QMouseEvent(QMouseEvent.MouseButtonPress, QPoint(1, 1), Qt.LeftButton, Qt.LeftButton, Qt.NoModifier)
        btn.launch_btn.event(event)

        self.assertEqual(window.tab_widget.currentIndex(), btn.tab_index)


if __name__ == '__main__':
    unittest.main()
