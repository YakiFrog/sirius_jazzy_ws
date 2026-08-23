import os
import sys
from pathlib import Path
import unittest

# Ensure the project root is on sys.path so imports work
sys.path.insert(0, str(Path.cwd()))
sys.path.insert(0, str(Path.cwd().joinpath('other_programs', 'sirius_launcher')))

from PySide6.QtWidgets import QApplication, QLabel
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

    def test_offline_mapping_has_own_tab_and_instructions(self):
        window = SiriusLauncher()
        tab_names = [
            window.tab_widget.tabText(index)
            for index in range(window.tab_widget.count())
        ]
        self.assertIn('オフライン・マッピング', tab_names)

        offline_buttons = {
            'record_offline_sim',
            'run_offline_mapping',
            'sam3_docker_gpu',
        }
        offline_index = tab_names.index('オフライン・マッピング')
        for button_name in offline_buttons:
            self.assertEqual(window.button_map[button_name].tab_index, offline_index)

        offline_tab = window.tab_widget.widget(offline_index)
        description_labels = offline_tab.findChildren(QLabel, "groupDescription")
        descriptions = "\n".join(label.text() for label in description_labels)
        self.assertIn('slamtoolbox', descriptions)
        self.assertIn('bag内の補正済みTF', descriptions)

        offline_preset = next(
            items
            for name, items in window.presets
            if name.startswith('オフラインマッピング録画セット')
        )
        self.assertLess(
            offline_preset.index('slamtoolbox'),
            offline_preset.index('record_offline_sim'),
        )


if __name__ == '__main__':
    unittest.main()
