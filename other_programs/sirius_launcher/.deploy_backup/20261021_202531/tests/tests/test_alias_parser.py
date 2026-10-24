import tempfile
import unittest
from pathlib import Path

from other_programs.sirius_launcher.alias_parser import parse_bash_aliases


class TestAliasParser(unittest.TestCase):
    def test_group_descriptions_are_optional_and_joined(self):
        source = """\
# GROUP: Offline
# GROUP_DESC: First line
# GROUP_DESC: Second line
# Start recorder
alias record_test='echo record'
"""
        with tempfile.TemporaryDirectory() as directory:
            alias_file = Path(directory) / "aliases.sh"
            alias_file.write_text(source, encoding="utf-8")

            groups, presets = parse_bash_aliases(alias_file)
            self.assertEqual(groups["Offline"][0][0], "record_test")
            self.assertEqual(presets, [])

            groups, presets, descriptions = parse_bash_aliases(
                alias_file, include_group_descriptions=True
            )
            self.assertEqual(descriptions["Offline"], "First line\nSecond line")
            self.assertEqual(groups["Offline"][0][2], "Start recorder")


if __name__ == "__main__":
    unittest.main()
