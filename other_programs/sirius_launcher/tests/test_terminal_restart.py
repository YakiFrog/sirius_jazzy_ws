from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_terminal_program_list_has_restart_buttons():
    script = (ROOT / 'static/console.js').read_text(encoding='utf-8')
    assert 'terminal-program-restart' in script
    assert "restartIt(name,restart)" in script
    assert "restart.title=name+' を再起動'" in script


def test_terminal_pane_header_has_restart_for_programs():
    script = (ROOT / 'static/terminal.js').read_text(encoding='utf-8')
    assert 'terminal-pane-restart' in script
    assert 'restartIt(alias(node.name),restart)' in script
    assert 'isProgram(node.name)' in script


def test_restart_buttons_have_theme_styles():
    css = (ROOT / 'static/terminal.css').read_text(encoding='utf-8')
    assert '.terminal-program-row' in css
    assert '.terminal-program-restart' in css
    assert '.terminal-pane-restart' in css
