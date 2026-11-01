from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_face_and_ear_have_distinct_icons_and_grouped_buttons():
    face = (ROOT / 'static' / 'face.js').read_text(encoding='utf-8')
    ear = (ROOT / 'static' / 'ear.js').read_text(encoding='utf-8')
    css = (ROOT / 'static' / 'face.css').read_text(encoding='utf-8')

    assert 'FACE_ICON' in face
    assert 'face-card-actions' in face
    assert 'SIRIUS FACE / EAR' in face
    assert 'EAR_ICON' in ear
    assert "faceButton.innerHTML=FACE_ICON+'<span>顔の操作</span>'" in ear
    assert "cardButton.innerHTML=EAR_ICON+'<span>耳の操作</span>'" in ear
    assert '.face-card-actions{' in css


def test_desktop_face_and_ear_are_shown_side_by_side():
    ear = (ROOT / 'static' / 'ear.js').read_text(encoding='utf-8')
    css = (ROOT / 'static' / 'ear.css').read_text(encoding='utf-8')

    assert "matchMedia('(min-width:980px)')" in ear
    assert 'ear-side-by-side' in ear
    assert 'section.hidden=both?false' in ear
    assert '.face-dialog.ear-side-by-side' in css
    assert 'grid-template-columns:minmax(0,1.45fr) minmax(300px,.9fr)' in css
    assert '.face-dialog.ear-side-by-side .face-widgets' in css
    assert '.face-dialog.ear-side-by-side .ear-device-tabs{display:none}' in css


def test_face_action_instructions_support_llm_and_direct_modes():
    face = (ROOT / 'static' / 'face.js').read_text(encoding='utf-8')
    css = (ROOT / 'static' / 'face.css').read_text(encoding='utf-8')

    assert 'face-action-section' in face
    assert 'data-action-mode="llm"' in face
    assert 'data-action-mode="nav"' in face
    assert 'face-action-form' in face
    assert 'ACTION_PRESETS' in face
    assert 'sendInstruction' in face
    assert 'face-action-feedback' in face
    assert 'actionFeedback' in face
    assert "action:'instruction'" not in face  # posted via send() payload
    assert "send('instruction'" in face
    assert '.face-action-mode{' in css
    assert '.face-action-presets{' in css
    assert '.face-action-feedback{' in css


def test_face_widgets_are_collapsible_and_persisted():
    face = (ROOT / 'static' / 'face.js').read_text(encoding='utf-8')
    ear = (ROOT / 'static' / 'ear.js').read_text(encoding='utf-8')
    css = (ROOT / 'static' / 'face.css').read_text(encoding='utf-8')

    assert 'function initCollapse' in face
    assert "sirius-face-collapse-" in face
    assert "window.siriusFaceCollapse=initCollapse" in face
    assert "initCollapse(document.querySelector('.face-status-area')" in face
    assert "initCollapse(document.querySelector('.face-action-section')" in face
    assert "siriusFaceCollapse(pane,'ear','耳LED')" in ear
    assert '.face-collapse-toggle' in css
    assert '.face-collapse-body[hidden]{display:none}' in css


def test_face_widgets_have_six_dot_drag_handles_and_saved_order():
    face = (ROOT / 'static' / 'face.js').read_text(encoding='utf-8')
    css = (ROOT / 'static' / 'face.css').read_text(encoding='utf-8')

    assert 'face-drag-handle' in face
    assert 'sirius-face-widget-order' in face
    assert "document.elementFromPoint" in face
    assert 'face-widgets > .face-section' in face
    assert '.face-drag-handle' in css
    assert 'touch-action:none' in css
    assert '.face-dragging-active' in css
