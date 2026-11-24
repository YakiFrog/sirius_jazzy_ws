'use strict';

(() => {
  const layerNames = {map: 'PGM', semantic: 'Road Semantic', texture: 'Texture Image',
    costmap: 'Cost Map', inflation: 'Inflation (Nav2)', drawing: 'Drawing',
    path: 'Path', waypoints: 'Waypoints', landmarks: 'Landmarks'};
  const layerMarkup = Object.entries(layerNames).map(([key, name]) => `
    <div class="me-layer-row">
      <label><input type="checkbox" data-layer="${key}" checked> ${name}</label>
      <input type="range" min="0" max="100" step="1" value="100" data-opacity="${key}"
             aria-label="${name} opacity" aria-valuetext="100%">
      <output data-opacity-value="${key}">100%</output>
    </div>`).join('');
  const tools = {
    select: ['Select', '<path d="m4 3 6 17 2.5-6 6.5-2.5L4 3Z"/>'],
    pan: ['Pan', '<path d="M8 12V5a1.5 1.5 0 0 1 3 0v6-8a1.5 1.5 0 0 1 3 0v8-6a1.5 1.5 0 0 1 3 0v7-4a1.5 1.5 0 0 1 3 0v7a6 6 0 0 1-6 6h-2c-2 0-3-1-4-2l-4-5a1.5 1.5 0 0 1 2-2l2 2"/>'],
    pen: ['Pen', '<path d="m4 16-1 5 5-1L20 8l-4-4L4 16Zm10-10 4 4"/>'],
    eraser: ['Eraser', '<path d="m3 14 8-10a2 2 0 0 1 3 0l6 5a2 2 0 0 1 0 3l-7 8H8l-5-4a2 2 0 0 1 0-2Zm4-4 9 7m-3 3h8"/>'],
    cost: ['Cost Pen', '<path d="m4 16-1 5 5-1L20 8l-4-4L4 16Zm10-10 4 4"/><path d="M14 13h5m-2.5-2.5v5"/>'],
    'cost-eraser': ['Cost Eraser', '<path d="m3 14 8-10a2 2 0 0 1 3 0l6 5a2 2 0 0 1 0 3l-7 8H8l-5-4a2 2 0 0 1 0-2Zm4-4 9 7m-3 3h8"/><path d="M15 4 20 9"/>'],
    add: ['Waypoint', '<path d="M10 21s-6-5.4-6-11a6 6 0 0 1 12 0c0 5.6-6 11-6 11Z"/><circle cx="10" cy="10" r="2"/><path d="M16 16h5m-2-2 2 2-2 2"/>'],
    landmark: ['Landmark', '<path d="M5 21V4m0 1c6-5 8 4 14 0v9c-6 4-8-5-14 0"/>'],
  };
  const toolShortcuts = {select: 'V', pan: 'H', pen: 'P', eraser: 'E', cost: 'C', 'cost-eraser': 'X',
    add: 'W', landmark: 'L'};
  const toolMarkup = Object.entries(tools).map(([mode, [label, icon]]) => {
    const shortcut = toolShortcuts[mode];
    return `<button type="button" class="hbtn me-tool" id="me-${mode}" data-tool="${mode}" title="${label} (${shortcut})" aria-keyshortcuts="${shortcut}" aria-pressed="${mode === 'select'}"><kbd class="me-key" aria-hidden="true">${shortcut}</kbd><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">${icon}</svg><span>${label}</span></button>`;
  }).join('');
  const page = document.createElement('section');
  page.id = 'page-map-editor';
  page.className = 'page';
  page.setAttribute('aria-label', 'Map Editor');
  page.innerHTML = `
    <header class="me-header">
      <div class="me-title"><h1>Map Editor</h1><div class="me-context"><span id="me-map-label">No map</span><span id="me-semantic-label" hidden></span><span id="me-route-label">No waypoints</span></div></div>
      <div class="me-actions">
        <button type="button" class="hbtn" id="me-open-map">Open Map</button>
        <button type="button" class="hbtn" id="me-open-route">Open Waypoints</button>
        <button type="button" class="hbtn" id="me-new-route">New Route</button>
        <button type="button" class="primary-button" id="me-export">Export</button>
      </div>
    </header>
    <p class="me-status" id="me-status" role="status"></p>
    <div class="me-layout">
      <section class="me-map-panel" aria-label="Map canvas">
        <div class="me-tools" role="group" aria-label="Map tools">
          <div class="me-tool-group">${toolMarkup}</div>
          <div class="me-tool-group me-view-tools">
            <button type="button" class="hbtn" id="me-undo" aria-label="Undo" aria-keyshortcuts="Control+Z Meta+Z" title="Undo (Ctrl+Z)">↶</button>
            <button type="button" class="hbtn" id="me-redo" aria-label="Redo" aria-keyshortcuts="Control+Shift+Z Meta+Shift+Z" title="Redo (Ctrl+Shift+Z)">↷</button>
            <button type="button" class="hbtn" id="me-zoom-out" aria-label="Zoom out" aria-keyshortcuts="-" title="Zoom out (-)">−</button>
            <button type="button" class="hbtn" id="me-zoom-in" aria-label="Zoom in" aria-keyshortcuts="+" title="Zoom in (+)">+</button>
            <button type="button" class="hbtn" id="me-native" title="One screen pixel per PGM cell (0)" aria-keyshortcuts="0">100%</button>
            <button type="button" class="hbtn" id="me-fit" title="Fit map (F)" aria-keyshortcuts="F">Fit</button>
            <button type="button" class="hbtn" id="me-grid" aria-pressed="false" aria-keyshortcuts="G" title="Toggle grid (G)">Grid</button>
          </div>
        </div>
        <label id="me-brush-controls" class="me-brush" hidden>Brush <input id="me-brush" type="range" min="1" max="128" value="5"> <output id="me-brush-size">5 px</output></label>
        <div class="me-viewport">
          <canvas id="me-canvas" tabindex="0" aria-label="Occupancy map and waypoints"></canvas>
          <p id="me-empty">Open a PGM map or its ROS YAML.</p>
          <span id="me-waypoint-mode" class="me-waypoint-mode" role="status" hidden></span>
          <div class="me-map-caption" id="me-caption" hidden>
            <span id="me-scale"></span><span id="me-resolution"></span>
          </div>
        </div>
        <div class="me-map-footer"><span id="me-coordinates"></span><span id="me-tool-hint"></span></div>
        <p class="me-shortcuts" aria-label="Keyboard shortcuts"><span><kbd>V</kbd> Select</span><span><kbd>H</kbd> Pan</span><span><kbd>P</kbd> Pen</span><span><kbd>E</kbd> Eraser</span><span><kbd>C</kbd> Cost Pen</span><span><kbd>X</kbd> Cost Eraser</span><span><kbd>W</kbd> Waypoint</span><span><kbd>L</kbd> Landmark</span><span><kbd>Ctrl+Z</kbd> Undo</span><span><kbd>Ctrl+Shift+Z</kbd> Redo</span><span><kbd>F</kbd> Fit</span><span><kbd>G</kbd> Grid</span><span><kbd>0</kbd> Native</span><span><kbd>+</kbd><kbd>−</kbd> Zoom</span></p>
      </section>
      <aside class="me-sidebar" aria-label="Editor panels">
        <details class="me-card me-disclosure" id="me-layer-panel">
          <summary>Layers</summary>
          <div class="me-layers">${layerMarkup}</div>
          <div class="me-actions me-layer-actions">
            <button type="button" class="hbtn" id="me-open-semantic">Open Semantic Map</button>
            <button type="button" class="hbtn" id="me-remove-semantic" hidden>Remove Semantic Map</button>
          </div>
          <ul id="me-semantic-legend" class="me-semantic-legend" aria-label="Road semantic classes" hidden></ul>
        </details>
        <details class="me-card me-disclosure" id="me-costmap-panel" hidden>
          <summary>Nav2 Costmap</summary>
          <p class="me-hint">Nav2 inflation_layer-compatible cost grid. 0 = free, 100 = lethal.</p>
          <div class="me-fields me-cost-settings">
            <label><span>Inflation radius (m)</span><input id="me-inflation-radius" type="number" min="0" max="10" step="0.05" value="0.75"></label>
            <label><span>Cost scaling</span><input id="me-inflation-scaling" type="number" min="0" max="50" step="0.5" value="3"></label>
            <label><span>Inscribed radius (m)</span><input id="me-inflation-inscribed" type="number" min="0" max="10" step="0.05" value="0.35"></label>
            <label><span>Occupied threshold</span><input id="me-inflation-occupied" type="number" min="0" max="1" step="0.05" value="0.65"></label>
            <label><span>Combine with inflation</span><select id="me-cost-combine"><option value="max">Max (recommended)</option><option value="add">Add</option></select></label>
          </div>
          <div class="me-cost-paint">
            <label><span>Paint class</span><select id="me-cost-class-select"></select></label>
            <label><span>Paint value</span><input id="me-cost-value" type="number" min="0" max="100" step="1" value="50"></label>
            <span class="me-legend-swatch" id="me-cost-swatch" aria-hidden="true"></span>
          </div>
          <div class="me-cost-table" id="me-cost-classes" aria-label="Semantic class costs"></div>
          <div class="me-actions me-layer-actions">
            <button type="button" class="hbtn" id="me-cost-reset">Reset to road map defaults</button>
            <button type="button" class="hbtn" id="me-cost-clear">Clear Cost Edits</button>
          </div>
          <p class="me-hint" id="me-cost-status" role="status"></p>
        </details>
        <details class="me-card me-disclosure" id="me-waypoints-panel" open>
          <summary>Waypoints <span id="me-count">0</span></summary>
          <div id="me-list" class="me-list"></div>
          <div class="me-actions">
            <button type="button" class="hbtn" id="me-up">Move Up</button>
            <button type="button" class="hbtn" id="me-down">Move Down</button>
            <button type="button" class="hbtn" id="me-generate-path" aria-pressed="false">Generate Path</button>
          </div>
        </details>
        <section class="me-card" id="me-inspector" hidden>
          <div class="me-inspector-heading">
            <header class="me-card-head"><h2 id="me-inspector-title"></h2><span id="me-edit-state">Read only</span></header>
            <div class="me-actions" id="me-edit-controls" hidden>
              <button type="button" class="primary-button" id="me-apply-edit">Apply Changes</button>
              <button type="button" class="hbtn" id="me-cancel-edit">Cancel</button>
            </div>
          </div>
          <div id="me-summary"></div>
          <button type="button" class="hbtn" id="me-edit-waypoint">Edit Waypoint</button>
          <div id="me-edit-fields" hidden>
            <div class="me-fields" id="me-pose"></div>
            <div id="me-arrival-editor">
              <h3>Arrival Actions</h3>
              <div class="me-fields" id="me-actions"></div>
              <div id="me-pose-actions"></div>
              <button type="button" class="hbtn" id="me-delete">Delete Waypoint</button>
            </div>
          </div>
        </section>
        <details class="me-card me-disclosure" id="me-landmark-panel" hidden>
          <summary>Landmarks <span id="me-landmark-count"></span></summary>
          <div id="me-landmark-list" class="me-list"></div>
          <button type="button" class="hbtn" id="me-landmarks-import">Import Landmarks</button>
        </details>
        <details class="me-card me-disclosure" id="me-format-panel" open>
          <summary>Format Editor</summary>
          <textarea id="me-format" spellcheck="false" aria-label="Waypoint format YAML"></textarea>
          <div class="me-actions">
            <button type="button" class="hbtn" id="me-format-import">Import</button>
            <button type="button" class="hbtn" id="me-format-export">Export Format</button>
            <button type="button" class="primary-button" id="me-format-apply">Update Format</button>
          </div>
          <p class="me-status" id="me-format-status" role="status"></p>
        </details>
      </aside>
    </div>
  `;
  document.querySelector('.workspace').append(page);

  const nav = document.createElement('button');
  nav.type = 'button';
  nav.className = 'nav-item';
  nav.dataset.page = 'map-editor';
  nav.title = 'Map Editor';
  nav.innerHTML = '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7" aria-hidden="true"><path d="m3 6 6-3 6 3 6-3v7M3 6v15l6-3 4 2M9 3v15m6-12v5"/><path d="m14 17 5-5 2 2-5 5-3 1z"/></svg><span>Map Editor</span>';
  nav.onclick = () => setPage('map-editor');
  document.querySelector('.nav-item[data-page="files"]').before(nav);
  if (PAGE === 'map-editor') setPage(PAGE);

  const el = id => document.getElementById(id);
  const canvas = el('me-canvas');
  const context = canvas.getContext('2d');
  const layerBuffer = document.createElement('canvas');
  const layerContext = layerBuffer.getContext('2d');
  const defaultFormat = {version: '1.0', format: {
    number: 'int', x: 'float', y: 'float', angle_radians: 'float', rotate: 'float', stop: 'bool',
    wait_time: 'float', change_map: 'string', threshold: 'float', initial_pose: 'pose',
    speech: 'string', person_area: 'bool', manual: 'bool',
  }};
  const poseKeys = new Set(['number', 'x', 'y', 'angle_radians', 'angle_degrees']);
  const actionDefaults = {rotate: 0, stop: false, wait_time: 0, change_map: '',
    threshold: -1, initial_pose: null, speech: '', person_area: false, manual: false};
  const costDefaults = {inflation_radius: 0.75, cost_scaling: 3.0, inscribed_radius: 0.35,
    occupied_thresh: 0.65, negate: 0};
  const state = {
    rasters: {map: null, semantic: null, texture: null, costmap: null, inflation: null},
    get map() { return (this.rasters.map || this.rasters.semantic)?.map; },
    route: null, path: '', saved: '', selected: -1, waypointPage: 0,
    mode: 'select', grid: false, undo: [], redo: [], view: null, drag: null,
    draft: null, draftMode: 'all', strokes: [], savedStrokes: [], landmarks: [], savedLandmarks: [],
    format: defaultFormat, classLabels: [], classCosts: {}, costStrokes: [], costToken: '',
    costRequest: 0, costTimer: 0, combine: 'max', inflation: {...costDefaults},
    generatedPath: null, generatedKey: '', showGeneratedPath: false,
    costPaint: {id: null, value: 50, color: [255, 0, 0]},
    layers: Object.fromEntries(Object.keys(layerNames).map(name => [name, {visible: true, opacity: 1}])),
  };
  state.layers.inflation.visible = false;
  const layerVisible = name => state.layers[name].visible && state.layers[name].opacity > 0;

  function node(tag, content, className) {
    const result = document.createElement(tag);
    if (content) result.textContent = content;
    if (className) result.className = className;
    return result;
  }

  function status(message, error = false) {
    el('me-status').textContent = message;
    el('me-status').classList.toggle('error', error);
  }

  async function get(endpoint, path) {
    const suffix = path === undefined ? '' : '?path=' + encodeURIComponent(path);
    const response = await fetch(endpoint + suffix, {cache: 'no-store', signal: AbortSignal.timeout(10000)});
    const payload = await response.json();
    if (!response.ok) throw Error(payload.error === 'auth' ? 'Sign in to continue.' : payload.error || 'Could not open the file.');
    return payload;
  }

  async function post(endpoint, body) {
    const response = await fetch('/api/map-editor/' + endpoint, {
      method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body),
    });
    const payload = await response.json();
    if (!response.ok) throw Error(payload.error || 'The request failed.');
    return payload;
  }

  const rows = () => state.route?.waypoints || [];
  const current = () => rows()[state.selected];
  const routeKey = () => JSON.stringify(rows().map((row, index) => {
    const item = index === state.selected && state.draft ? state.draft : row;
    return [item.number, item.x, item.y, item.angle_radians];
  }));
  const degrees = radians => Number((radians * 180 / Math.PI).toFixed(7));
  const radians = value => value * Math.PI / 180;
  const basename = name => name.split('/').pop();
  const snapshot = () => ({
    route: structuredClone(state.route), selected: state.selected, waypointPage: state.waypointPage,
    strokes: state.strokes, landmarks: state.landmarks, costStrokes: state.costStrokes,
    classCosts: structuredClone(state.classCosts), inflation: {...state.inflation}, combine: state.combine,
  });
  const routeDirty = () => Boolean(state.route && JSON.stringify(state.route) !== state.saved);
  const changed = (current, saved) => JSON.stringify(current) !== JSON.stringify(saved);
  const drawingDirty = () => changed(state.strokes, state.savedStrokes);
  const landmarksDirty = () => changed(state.landmarks, state.savedLandmarks);
  const draftDirty = () => state.draft && changed(state.draft, current());
  const replaceAllowed = () => !(routeDirty() || draftDirty()) || confirm('Discard unsaved waypoint edits?');

  function remember(before) {
    state.undo.push(before);
    if (state.undo.length > 40) state.undo.shift();
    state.redo.length = 0;
  }

  function edit(change, cost = false) {
    const previous = snapshot();
    change();
    remember(previous);
    render();
    if (cost) scheduleCostmap(0);
  }

  function historyStep(undo) {
    const from = undo ? state.undo : state.redo;
    const to = undo ? state.redo : state.undo;
    if (!from.length || !leaveWaypointEdit()) return;
    to.push(snapshot());
    Object.assign(state, from.pop());
    render();
    if (state.rasters.semantic) scheduleCostmap(0);
  }

  function setRoute(document, path) {
    state.route = document;
    state.path = path;
    state.saved = JSON.stringify(document);
    state.selected = -1;
    state.waypointPage = 0;
    state.draft = null;
    state.undo.length = 0;
    state.redo.length = 0;
    setMode('select');
    render();
  }

  function field(container, label, value, change, options = {}) {
    const wrapper = node('label');
    wrapper.append(node('span', label));
    const input = node(options.type === 'bool' ? 'select' : 'input');
    if (options.type === 'bool') {
      for (const [key, title] of [['', 'Unset'], ['true', 'true'], ['false', 'false']]) {
        const option = node('option', title);
        option.value = key;
        input.append(option);
      }
    } else input.type = options.type || 'text';
    input.value = value ?? '';
    if (input.type === 'number') input.step = options.integer ? '1' : 'any';
    if (options.key) input.dataset.field = options.key;
    input.onchange = () => {
      try {
        if (input.value === '' && options.optional) change(undefined);
        else if (options.type === 'bool') change(input.value === 'true');
        else if (input.type === 'number') {
          const number = input.valueAsNumber;
          if (!Number.isFinite(number) || (options.integer && !Number.isSafeInteger(number))) {
            throw Error(label + (options.integer ? ' requires an integer.' : ' requires a number.'));
          }
          change(number);
        } else change(options.json ? JSON.parse(input.value) : input.value);
        status('');
      } catch (error) {
        input.value = value ?? '';
        status(error.message, true);
      }
    };
    input.disabled = Boolean(options.disabled);
    wrapper.append(input);
    container.append(wrapper);
  }

  function renderInspector() {
    const waypoint = current();
    el('me-inspector').hidden = !waypoint;
    if (!waypoint) return;
    el('me-inspector-title').textContent = 'Waypoint ' + waypoint.number;
    const summary = el('me-summary');
    const draft = state.draft;
    const mode = state.draftMode;
    summary.hidden = Boolean(draft);
    el('me-edit-state').textContent = draft ? {all: 'Editing', position: 'Position', heading: 'Heading · Position locked'}[mode] : 'Read only';
    el('me-edit-fields').hidden = !draft;
    el('me-edit-controls').hidden = !draft;
    el('me-edit-waypoint').hidden = Boolean(draft);
    if (!draft) {
      summary.replaceChildren();
      const details = node('dl', '', 'me-summary-list');
      details.append(node('dt', 'Coordinates'), node('dd', waypoint.x.toFixed(3) + ', ' + waypoint.y.toFixed(3) + ' m'),
        node('dt', 'Heading'), node('dd', waypoint.angle_radians.toFixed(3) + ' rad'));
      summary.append(details, node('h3', 'Arrival Actions'));
      const actions = node('dl', '', 'me-summary-list');
      for (const [key, value] of Object.entries(waypoint)) {
        if (poseKeys.has(key) || value === null || value === actionDefaults[key] || value === '') continue;
        actions.append(node('dt', key), node('dd', typeof value === 'object' ? JSON.stringify(value) : String(value)));
      }
      summary.append(actions.childElementCount ? actions : node('p', 'No configured actions.', 'me-hint'));
      return;
    }
    const pose = el('me-pose');
    pose.replaceChildren();
    const bind = (key, value) => {
      if (value === undefined) delete draft[key];
      else draft[key] = value;
      if (key === 'change_map') renderInspector();
      draw();
    };
    field(pose, 'X (m)', draft.x, value => bind('x', value), {type: 'number', key: 'x', disabled: mode === 'heading'});
    field(pose, 'Y (m)', draft.y, value => bind('y', value), {type: 'number', key: 'y', disabled: mode === 'heading'});
    field(pose, 'Heading (°)', degrees(draft.angle_radians), value => bind('angle_radians', radians(value)),
      {type: 'number', key: 'angle_radians', disabled: mode === 'position'});
    el('me-arrival-editor').hidden = mode !== 'all';
    if (mode !== 'all') return;

    const actions = el('me-actions');
    actions.replaceChildren();
    const format = Object.entries(state.format.format);
    for (const [key, type] of format) {
      if (poseKeys.has(key) || key === 'initial_pose') continue;
      const numeric = type === 'int' || type === 'float';
      const value = draft[key];
      field(actions, key, type === 'pose' && value != null ? JSON.stringify(value) : value, item => bind(key, item),
        {type: numeric ? 'number' : type === 'bool' ? 'bool' : 'text', key, optional: true, integer: type === 'int',
          json: type === 'pose', disabled: key === 'change_map' && draft.initial_pose != null});
    }
    const structured = el('me-pose-actions');
    structured.replaceChildren();
    if (state.format.format.initial_pose !== 'pose') return;
    const initial = draft.initial_pose;
    const checkbox = (id, label, checked, disabled, change) => {
      const wrapper = node('label', '', 'me-checkbox');
      const input = node('input');
      input.type = 'checkbox';
      input.id = id;
      input.checked = checked;
      input.disabled = disabled;
      input.onchange = () => { change(input.checked); renderInspector(); };
      wrapper.append(input, label);
      structured.append(wrapper);
    };
    checkbox('me-initial', 'initial_pose', Boolean(initial), Boolean(draft.change_map), enabled => {
      if (enabled) draft.initial_pose = {x: draft.x, y: draft.y, yaw: draft.angle_radians};
      else delete draft.initial_pose;
    });
    if (!initial) return;
    const keepHeading = initial.yaw_mode === 'current';
    checkbox('me-keep-heading', 'Keep current heading', keepHeading, false, enabled => {
      if (enabled) initial.yaw_mode = 'current';
      else {
        initial.yaw_mode = 'fixed';
        initial.yaw ??= draft.angle_radians;
      }
    });
    const initialPose = node('div', '', 'me-fields');
    structured.append(initialPose);
    field(initialPose, 'Initial X (m)', initial.x, value => { initial.x = value; }, {type: 'number', key: 'initial-x'});
    field(initialPose, 'Initial Y (m)', initial.y, value => { initial.y = value; }, {type: 'number', key: 'initial-y'});
    if (!keepHeading) field(initialPose, 'Initial heading (°)', degrees(initial.yaw),
      value => { initial.yaw = radians(value); }, {type: 'number', key: 'initial-yaw'});
    const copy = node('button', 'Use Waypoint Pose', 'hbtn');
    copy.type = 'button';
    copy.onclick = () => {
      initial.x = draft.x;
      initial.y = draft.y;
      if (!keepHeading) initial.yaw = draft.angle_radians;
      renderInspector();
    };
    structured.append(copy);
  }

  function leaveWaypointEdit() {
    if (draftDirty() && !confirm('Discard changes to the selected waypoint?')) return false;
    state.draft = null;
    return true;
  }

  function select(index, center = false) {
    if (index !== state.selected && !leaveWaypointEdit()) return false;
    state.selected = index;
    if (center && current() && state.view) {
      const {x, y} = state.draft || current();
      const point = worldToPixel(x, y);
      if (insidePlacement(point)) {
        state.view.x = canvas.clientWidth / 2 - point.x * state.view.scale;
        state.view.y = canvas.clientHeight / 2 - point.y * state.view.scale;
      }
    }
    refreshWaypointSelection();
    scrollSelectedWaypoint();
    return true;
  }

  function refreshWaypointSelection() {
    for (const button of el('me-list').querySelectorAll('.me-waypoint[data-index]')) {
      button.setAttribute('aria-pressed', String(Number(button.dataset.index) === state.selected));
    }
    el('me-up').disabled = Boolean(state.draft) || state.selected < 1;
    el('me-down').disabled = Boolean(state.draft) || state.selected < 0 || state.selected >= rows().length - 1;
    renderInspector();
    renderToolHint();
    draw();
  }

  function beginWaypointEdit(mode) {
    if (!current() || state.drag) return;
    if (mode !== 'all' && state.draft && state.draftMode === 'all' && draftDirty()) {
      status('Apply or cancel the current waypoint edits first.', true);
      return;
    }
    state.draft ??= structuredClone(current());
    if (mode === 'heading' && state.draftMode === 'position') {
      state.draft.x = current().x;
      state.draft.y = current().y;
    }
    state.draftMode = mode;
    setMode('select');
    refreshWaypointSelection();
  }

  const followingWaypoint = () => state.mode === 'select' && state.draft && state.draftMode !== 'all';
  function clickSequence(previous, event, target) {
    const touch = event.pointerType === 'touch' || event.sourceCapabilities?.firesTouchEvents;
    let count = event.detail;
    if (target !== previous?.target) count = Math.min(count, 1);
    else if (touch && previous.touch && count === 1 && event.timeStamp - previous.time <= 400 &&
        Math.hypot(event.clientX - previous.x, event.clientY - previous.y) <= 14) {
      count = previous.count + 1;
    }
    return {target, count, touch, time: event.timeStamp, x: event.clientX, y: event.clientY};
  }

  let lastListClick = null;
  const isMultiClick = detail => detail === 2 || detail === 3;
  const clickEditMode = detail => detail === 3 ? 'heading' : 'position';
  function scrollSelectedWaypoint() {
    if (state.selected < 0) return;
    const button = el('me-list').querySelector('.me-waypoint[data-index="' + state.selected + '"]');
    button?.scrollIntoView({block: 'nearest'});
  }

  function renderWaypointList() {
    const list = el('me-list');
    const scrollTop = list.scrollTop;
    const content = document.createDocumentFragment();
    const waypoints = rows();
    const count = waypoints.length;
    const start = 0;
    const end = count;
    for (let index = start; index < end; index++) {
      const waypoint = waypoints[index];
      const row = node('div', '', 'me-waypoint-row');
      row.draggable = !state.draft;
      row.dataset.dragIndex = index;
      row.addEventListener('dragstart', event => {
        if (state.draft) return event.preventDefault();
        event.dataTransfer.effectAllowed = 'move';
        event.dataTransfer.setData('text/plain', String(index));
        row.classList.add('dragging');
      });
      row.addEventListener('dragend', () => row.classList.remove('dragging'));
      row.addEventListener('dragover', event => {
        if (state.draft) return;
        event.preventDefault();
        event.dataTransfer.dropEffect = 'move';
        row.classList.add('drop-target');
      });
      row.addEventListener('dragleave', () => row.classList.remove('drop-target'));
      row.addEventListener('drop', event => {
        event.preventDefault();
        row.classList.remove('drop-target');
        const from = Number(event.dataTransfer.getData('text/plain'));
        if (Number.isSafeInteger(from)) reorderWaypoint(from, index);
      });
      const handle = node('span', '⠿', 'me-waypoint-handle');
      handle.title = 'Drag to reorder';
      handle.setAttribute('aria-hidden', 'true');
      const button = node('button', '', 'me-waypoint');
      button.type = 'button';
      button.dataset.index = index;
      button.setAttribute('aria-pressed', String(state.selected === index));
      button.append(node('strong', String(waypoint.number).padStart(2, '0')),
        node('span', '(' + waypoint.x.toFixed(2) + ', ' + waypoint.y.toFixed(2) + ')'),
        node('span', waypoint.angle_radians.toFixed(2) + ' rad', 'muted'));
      button.title = 'Double-click to move · Triple-click to rotate';
      button.onclick = event => {
        lastListClick = clickSequence(lastListClick, event, button);
        if (select(index, true) && isMultiClick(lastListClick.count)) {
          beginWaypointEdit(clickEditMode(lastListClick.count));
        }
      };
      const remove = node('button', '', 'hbtn me-waypoint-delete');
      remove.type = 'button';
      remove.title = 'Delete waypoint ' + waypoint.number;
      remove.setAttribute('aria-label', remove.title);
      remove.innerHTML = '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M4 7h16M9 7V4h6v3m3 0-1 13H7L6 7m4 4v5m4-5v5"/></svg>';
      remove.onclick = () => {
        if (removeWaypoint(index)) {
          list.querySelector('[data-index="' + Math.min(index, rows().length - 1) + '"]')?.focus({preventScroll: true});
        }
      };
      row.append(handle, button, remove);
      content.append(row);
    }
    if (!count) content.append(node('p', 'No waypoints.', 'me-hint'));
    list.replaceChildren(content);
    list.scrollTop = scrollTop;
    el('me-count').textContent = count;
  }

  function render() {
    renderWaypointList();
    renderMapLayers();
    el('me-route-label').textContent = state.route ? (state.path ? basename(state.path) : 'New Route') + (routeDirty() ? ' · Unsaved' : '') : 'No waypoints';
    el('me-undo').disabled = !state.undo.length;
    el('me-redo').disabled = !state.redo.length;
    el('me-export').disabled = !state.route && !state.map;
    if (state.generatedPath && state.generatedKey !== routeKey()) {
      state.generatedPath = null;
      state.showGeneratedPath = false;
    }
    const generate = el('me-generate-path');
    generate.disabled = rows().length < 2 || !state.costToken;
    generate.setAttribute('aria-pressed', String(state.showGeneratedPath));
    el('me-caption').hidden = !state.map;
    el('me-landmark-panel').hidden = !state.map;
    for (const mode of ['pan', 'add', 'landmark']) el('me-' + mode).disabled = !state.map;
    for (const mode of ['pen', 'eraser']) el('me-' + mode).disabled = !state.map?.editable;
    for (const mode of ['cost', 'cost-eraser']) el('me-' + mode).disabled = !state.rasters.costmap;
    for (const id of ['fit', 'native', 'grid', 'zoom-in', 'zoom-out']) el('me-' + id).disabled = !state.map;
    renderLandmarks();
    refreshWaypointSelection();
  }

  function worldToPixel(x, y, map = state.map) {
    const {origin: [ox, oy, yaw], resolution, height} = map;
    const dx = x - ox, dy = y - oy;
    const c = Math.cos(yaw), s = Math.sin(yaw);
    return {x: (c * dx + s * dy) / resolution, y: height - (-s * dx + c * dy) / resolution};
  }

  function pixelToWorld(x, y, map = state.map) {
    const {origin: [ox, oy, yaw], resolution, height} = map;
    const dx = x * resolution, dy = (height - y) * resolution;
    const c = Math.cos(yaw), s = Math.sin(yaw);
    return {x: ox + c * dx - s * dy, y: oy + s * dx + c * dy};
  }

  function screenToPixel({x, y}) {
    return {x: (x - state.view.x) / state.view.scale, y: (y - state.view.y) / state.view.scale};
  }

  function screenToMapPixel(point, map) {
    if (!map || map === state.map) return screenToPixel(point);
    const base = screenToPixel(point);
    const world = pixelToWorld(base.x, base.y);
    return worldToPixel(world.x, world.y, map);
  }

  function costMap() {
    return state.rasters.costmap?.map || state.rasters.semantic?.map;
  }

  function screenPoint(waypoint) {
    const pixel = worldToPixel(waypoint.x, waypoint.y);
    return {x: state.view.x + pixel.x * state.view.scale, y: state.view.y + pixel.y * state.view.scale};
  }

  function insideMap({x, y}, map = state.map) {
    return x >= 0 && y >= 0 && x < map.width && y < map.height;
  }

  function insidePlacement(pixel) {
    if (insideMap(pixel)) return true;
    const semantic = state.rasters.semantic?.map;
    if (!semantic || semantic === state.map) return false;
    const {x, y} = pixelToWorld(pixel.x, pixel.y);
    return insideMap(worldToPixel(x, y, semantic), semantic);
  }

  function rasterBounds() {
    const corners = Object.values(state.rasters).filter(Boolean).flatMap(({map}) =>
      [[0, 0], [map.width, 0], [0, map.height], [map.width, map.height]].map(([x, y]) => {
        if (map === state.map) return {x, y};
        const world = pixelToWorld(x, y, map);
        return worldToPixel(world.x, world.y);
      }));
    return {left: Math.min(...corners.map(point => point.x)), right: Math.max(...corners.map(point => point.x)),
      top: Math.min(...corners.map(point => point.y)), bottom: Math.max(...corners.map(point => point.y))};
  }

  function fit() {
    if (!state.map || !canvas.clientWidth || !canvas.clientHeight) return;
    const bounds = rasterBounds();
    const scale = fitScale(canvas.clientWidth, canvas.clientHeight, bounds);
    if (scale <= 0) return;
    state.view = {x: (canvas.clientWidth - (bounds.left + bounds.right) * scale) / 2,
      y: (canvas.clientHeight - (bounds.top + bounds.bottom) * scale) / 2, scale, fit: scale};
    draw();
  }

  function fitScale(width, height, bounds = rasterBounds()) {
    return Math.min((width - 24) / (bounds.right - bounds.left), (height - 24) / (bounds.bottom - bounds.top));
  }

  function paintLayer(name, width, height, ratio, paint, isolate = false) {
    const opacity = state.layers[name].opacity;
    if (!isolate && opacity === 1) return paint(context);
    layerContext.setTransform(ratio, 0, 0, ratio, 0, 0);
    layerContext.clearRect(0, 0, width, height);
    const result = paint(layerContext);
    context.globalAlpha = opacity;
    context.drawImage(layerBuffer, 0, 0, width, height);
    context.globalAlpha = 1;
    return result;
  }

  function rasterTransform(map) {
    if (map === state.map) return {...state.view, angle: 0};
    const origin = pixelToWorld(0, 0, map);
    const {x, y} = screenPoint(origin);
    return {x, y, angle: state.map.origin[2] - map.origin[2],
      scale: state.view.scale * map.resolution / state.map.resolution};
  }

  let tileFrame = 0;
  function drawRaster(ctx, name, width, height, ratio) {
    const raster = state.rasters[name];
    const {map, image, tiles} = raster;
    const {x, y, scale, angle} = rasterTransform(map);
    ctx.save();
    ctx.translate(x, y);
    if (angle) ctx.rotate(angle);
    ctx.scale(scale, scale);
    ctx.imageSmoothingEnabled = !['semantic', 'costmap', 'inflation'].includes(name) && scale < 1;
    ctx.drawImage(image, 0, 0, map.width, map.height);
    const fullSize = image.naturalWidth === map.width && image.naturalHeight === map.height;
    if (fullSize || scale < 0.25 || scale * ratio <= image.naturalWidth / map.width) {
      ctx.restore();
      return fullSize ? 'Native PGM' : 'Preview';
    }
    // Invert the view transform to find the raster cells intersecting the screen.
    const c = Math.cos(angle), s = Math.sin(angle);
    const corners = [[0, 0], [width, 0], [0, height], [width, height]].map(([sx, sy]) => [
      (c * (sx - x) + s * (sy - y)) / scale, (-s * (sx - x) + c * (sy - y)) / scale,
    ]);
    const size = map.tile_size;
    const cols = corners.map(([col]) => col / size), rows = corners.map(([, row]) => row / size);
    const fromCol = Math.max(0, Math.floor(Math.min(...cols)));
    const fromRow = Math.max(0, Math.floor(Math.min(...rows)));
    const toCol = Math.min(Math.ceil(map.width / size), Math.ceil(Math.max(...cols)));
    const toRow = Math.min(Math.ceil(map.height / size), Math.ceil(Math.max(...rows)));
    const visible = new Set();
    let pending = false, failed = false;
    const snap = (value, offset) => angle ? value : (Math.round((offset + value * scale) * ratio) / ratio - offset) / scale;
    ctx.imageSmoothingEnabled = false;
    for (let row = fromRow; row < toRow; row++) {
      for (let col = fromCol; col < toCol; col++) {
        const key = col + ',' + row;
        visible.add(key);
        let tile = tiles.get(key);
        if (!tile) {
          const bitmap = new Image();
          tile = {bitmap, ready: false, failed: false};
          tiles.set(key, tile);
          const finish = ok => {
            tile.ready = ok;
            tile.failed = !ok;
            if (state.rasters[name] !== raster || tiles.get(key) !== tile || tileFrame) return;
            tileFrame = requestAnimationFrame(() => { tileFrame = 0; draw(); });
          };
          bitmap.onload = () => finish(true);
          bitmap.onerror = () => finish(false);
          let source;
          if (name === 'costmap' || name === 'inflation') {
            source = '/api/map-editor/costmap-tile?' + new URLSearchParams({
              token: state.costToken, kind: name, col, row,
            });
          } else {
            const endpoint = name === 'texture' ? '/api/map-editor/texture-tile' : '/api/map-editor/tile';
            source = endpoint + '?' + new URLSearchParams({
              path: map.semantic?.palette_path || map.image, revision: map.image_revision, col, row,
            });
          }
          bitmap.src = source;
        }
        if (!tile.ready) {
          pending ||= !tile.failed;
          failed ||= tile.failed;
          continue;
        }
        const left = snap(col * size, x), top = snap(row * size, y);
        const right = snap(col * size + tile.bitmap.naturalWidth, x);
        const bottom = snap(row * size + tile.bitmap.naturalHeight, y);
        ctx.clearRect(left, top, right - left, bottom - top);
        ctx.drawImage(tile.bitmap, left, top, right - left, bottom - top);
      }
    }
    for (const key of tiles.keys()) {
      if (tiles.size <= Math.max(48, visible.size)) break;
      if (!visible.has(key)) tiles.delete(key);
    }
    ctx.restore();
    return failed ? 'Detail unavailable' : pending ? 'Loading detail…' : 'Native PGM';
  }

  function drawCostDraft(context) {
    const stroke = state.drag?.stroke;
    const raster = state.rasters.costmap;
    if (!stroke || !raster) return;
    const {x, y, scale, angle} = rasterTransform(raster.map);
    context.save();
    context.translate(x, y);
    if (angle) context.rotate(angle);
    context.scale(scale, scale);
    const color = stroke.color || [255, 0, 0];
    const alpha = stroke.tool === 'cost-eraser' ? 0.85 : Math.max(0.25, (stroke.value || 0) / 100);
    context.strokeStyle = context.fillStyle = 'rgba(' + color.join(',') + ',' + alpha + ')';
    context.lineWidth = stroke.size;
    context.lineCap = context.lineJoin = 'round';
    const points = stroke.points;
    context.beginPath();
    const [startX, startY] = points[0];
    context.moveTo(startX + 0.5, startY + 0.5);
    for (const [px, py] of points) context.lineTo(px + 0.5, py + 0.5);
    context.stroke();
    context.beginPath();
    context.arc(startX + 0.5, startY + 0.5, stroke.size / 2, 0, Math.PI * 2);
    context.fill();
    context.restore();
  }

  function drawStrokes(context) {
    const {x, y, scale} = state.view;
    const map = state.map;
    context.save();
    context.translate(x, y);
    context.scale(scale, scale);
    context.beginPath();
    context.rect(0, 0, map.width, map.height);
    context.clip();
    const draft = state.drag?.stroke;
    for (const {tool, size, points} of draft ? [...state.strokes, draft] : state.strokes) {
      const light = tool === 'pen' ? map.occupied : 255 - map.occupied;
      context.strokeStyle = context.fillStyle = light === 0 ? '#000' : '#fff';
      context.lineWidth = size;
      context.lineCap = context.lineJoin = 'round';
      context.beginPath();
      const [startX, startY] = points[0];
      context.moveTo(startX + 0.5, startY + 0.5);
      for (const [px, py] of points) context.lineTo(px + 0.5, py + 0.5);
      context.stroke();
      context.beginPath();
      context.arc(startX + 0.5, startY + 0.5, size / 2, 0, Math.PI * 2);
      context.fill();
    }
    context.restore();
  }

  function draw() {
    const width = canvas.clientWidth, height = canvas.clientHeight;
    if (!width || !height) return;
    const ratio = Math.min(devicePixelRatio || 1, 2);
    const bufferWidth = Math.round(width * ratio), bufferHeight = Math.round(height * ratio);
    if (canvas.width !== bufferWidth || canvas.height !== bufferHeight) {
      canvas.width = bufferWidth;
      canvas.height = bufferHeight;
    }
    if (layerBuffer.width !== bufferWidth || layerBuffer.height !== bufferHeight) {
      layerBuffer.width = bufferWidth;
      layerBuffer.height = bufferHeight;
    }
    context.setTransform(ratio, 0, 0, ratio, 0, 0);
    context.imageSmoothingEnabled = false;
    context.clearRect(0, 0, width, height);
    if (!state.map || !state.view) return;
    const {scale} = state.view;
    const drawLayer = (name, content, isolate = false) =>
      paintLayer(name, width, height, ratio, content, isolate);
    const rasters = ['map', 'texture', 'semantic', 'costmap', 'inflation']
      .filter(name => state.rasters[name] && layerVisible(name));
    const displays = rasters.map(name => drawLayer(name, ctx => drawRaster(ctx, name, width, height, ratio), true));
    const display = rasters.length ? rasters.map((name, index) => layerNames[name] + ': ' + displays[index]).join(' · ') : 'Map hidden';
    if (state.rasters.map && layerVisible('drawing')) drawLayer('drawing', drawStrokes);
    if (state.rasters.costmap && state.drag?.stroke?.tool?.startsWith('cost') &&
        state.layers.costmap.visible) drawLayer('costmap', drawCostDraft);
    el('me-resolution').textContent = display + ' · ' + Number(state.map.resolution.toPrecision(3)) + ' m / cell';
    el('me-native').textContent = Math.round(scale * 100) + '%';
    const metresAcross = 85 * state.map.resolution / scale;
    const unit = 10 ** Math.floor(Math.log10(metresAcross));
    const bar = ([5, 2, 1].find(value => value * unit <= metresAcross) || 1) * unit;
    el('me-scale').textContent = Number(bar.toPrecision(2)) + ' m';
    el('me-scale').style.setProperty('--bar-width', bar * scale / state.map.resolution + 'px');
    if (state.grid) drawGrid(width, height);

    const waypoints = state.draft ? rows().map((item, index) => index === state.selected ? state.draft : item) : rows();
    const points = waypoints.map(screenPoint);
    const pathPoints = (state.showGeneratedPath && state.generatedPath?.length
      ? state.generatedPath : waypoints).map(screenPoint);
    if (layerVisible('path')) drawLayer('path', ctx => {
      ctx.beginPath();
      pathPoints.forEach((point, index) => {
        if (index === 0) ctx.moveTo(point.x, point.y);
        else ctx.lineTo(point.x, point.y);
      });
      ctx.strokeStyle = state.showGeneratedPath ? '#169b62' : '#166bdb';
      ctx.lineWidth = state.showGeneratedPath ? 2.5 : 2;
      ctx.stroke();
    });
    const labels = [];
    const selectedPoint = points[state.selected];
    if (selectedPoint) labels.push(selectedPoint);
    function drawWaypoint(context, point, index, selected = false) {
      if (point.x < -30 || point.x > width + 30 || point.y < -30 || point.y > height + 30) return;
      const showLabel = selected || !labels.some(label =>
        Math.abs(point.x - label.x) < 42 && Math.abs(point.y - label.y) < 20);
      context.save();
      context.translate(point.x, point.y);
      context.fillStyle = selected ? '#c65917' : '#075cb9';
      context.strokeStyle = '#fff';
      context.lineWidth = 1.5;
      context.beginPath();
      context.arc(0, 0, selected ? 7 : 4, 0, 2 * Math.PI);
      context.fill();
      context.stroke();
      if (showLabel) {
        labels.push(point);
        context.font = 'bold 11px sans-serif';
        context.fillText(String(waypoints[index].number), 9, -8);
        context.rotate(state.map.origin[2] - waypoints[index].angle_radians);
        context.beginPath();
        context.moveTo(7, 0);
        context.lineTo(19, 0);
        context.lineTo(14, -4);
        context.moveTo(19, 0);
        context.lineTo(14, 4);
        context.strokeStyle = context.fillStyle;
        context.lineWidth = 2;
        context.stroke();
      }
      context.restore();
    }
    if (layerVisible('waypoints')) drawLayer('waypoints', ctx => {
      points.forEach((point, index) => {
        if (index !== state.selected) drawWaypoint(ctx, point, index);
      });
      if (selectedPoint) drawWaypoint(ctx, selectedPoint, state.selected, true);
    });
    if (layerVisible('landmarks')) drawLayer('landmarks', drawLandmarks);
    const costMode = ['cost', 'cost-eraser'].includes(state.mode);
    if (state.brushCursor && ((layerVisible('drawing') && ['pen', 'eraser'].includes(state.mode)) ||
        (costMode && state.rasters.costmap && layerVisible('costmap')))) {
      const costMap = state.rasters.costmap?.map || state.rasters.semantic?.map;
      const costScale = costMap && costMap !== state.map
        ? scale * costMap.resolution / state.map.resolution : scale;
      context.beginPath();
      context.arc(state.brushCursor.x, state.brushCursor.y,
        Math.max(2, Number(el('me-brush').value) * costScale / 2), 0, Math.PI * 2);
      context.strokeStyle = '#fff';
      context.lineWidth = 3;
      context.stroke();
      context.strokeStyle = costMode && state.mode === 'cost'
        ? 'rgb(' + state.costPaint.color.join(',') + ')' : '#314c84';
      context.lineWidth = 1;
      context.stroke();
    }
  }

  function drawGrid(width, height) {
    const view = state.view, map = state.map;
    const spacing = view.scale / map.resolution;
    const step = spacing * 10 ** Math.ceil(Math.log10(32 / spacing));
    const left = Math.max(0, view.x), top = Math.max(0, view.y);
    const right = Math.min(width, view.x + map.width * view.scale);
    const bottom = Math.min(height, view.y + map.height * view.scale);
    context.beginPath();
    for (let x = view.x + Math.ceil((left - view.x) / step) * step; x < right; x += step) {
      context.moveTo(x, top);
      context.lineTo(x, bottom);
    }
    const originY = view.y + map.height * view.scale;
    for (let y = originY + Math.ceil((top - originY) / step) * step; y < bottom; y += step) {
      context.moveTo(left, y);
      context.lineTo(right, y);
    }
    context.lineWidth = 1;
    context.strokeStyle = '#26486655';
    context.stroke();
  }

  function zoom(factor, anchor = {x: canvas.clientWidth / 2, y: canvas.clientHeight / 2}) {
    if (!state.view) return;
    const point = screenToPixel(anchor);
    const view = state.view;
    view.scale = Math.min(Math.max(view.scale * factor, Math.min(view.fit / 2, 1)), Math.max(12, view.fit * 80));
    view.x = anchor.x - point.x * view.scale;
    view.y = anchor.y - point.y * view.scale;
    draw();
  }

  function setMode(mode) {
    if (state.drag) return;
    state.mode = mode;
    canvas.dataset.mode = mode;
    state.brushCursor = null;
    const brush = ['pen', 'eraser', 'cost', 'cost-eraser'].includes(mode);
    el('me-brush-controls').hidden = !brush;
    for (const button of page.querySelectorAll('[data-tool]')) {
      button.setAttribute('aria-pressed', String(button.dataset.tool === mode));
    }
    if (['pen', 'eraser'].includes(mode) && !state.layers.drawing.visible) {
      state.layers.drawing.visible = true;
      page.querySelector('[data-layer="drawing"]').checked = true;
    }
    if (['cost', 'cost-eraser'].includes(mode) && !state.layers.costmap.visible) {
      state.layers.costmap.visible = true;
      page.querySelector('[data-layer="costmap"]').checked = true;
    }
    renderToolHint();
    draw();
  }

  function renderToolHint() {
    const mode = state.mode;
    const draftMode = state.draft && state.draftMode;
    el('me-tool-hint').textContent = {
      select: 'Double-click to move · Triple-click to rotate',
      pan: 'Drag to pan.',
      add: 'Drag to place a waypoint and set its heading.',
      pen: 'Draw occupied cells.',
      eraser: 'Paint free cells.',
      cost: 'Paint semantic cost (0–100).',
      'cost-eraser': 'Erase semantic cost.',
      landmark: 'Drag to place a landmark and set its heading.',
    }[mode];
    const indicator = el('me-waypoint-mode');
    indicator.hidden = !state.map || mode !== 'select' || !draftMode || draftMode === 'all';
    canvas.toggleAttribute('data-follow-waypoint', !indicator.hidden);
    if (!indicator.hidden) {
      indicator.textContent = draftMode === 'heading'
        ? 'Heading · Move the pointer to rotate · Click to confirm'
        : 'Position · Move the pointer · Click to confirm';
    }
  }

  function eventPoint(event) {
    const box = canvas.getBoundingClientRect();
    return {x: event.clientX - box.left, y: event.clientY - box.top};
  }

  function followWaypoint(point) {
    const pixel = screenToPixel(point);
    if (!insidePlacement(pixel)) return false;
    const waypoint = state.draft;
    const pose = el('me-pose');
    if (state.draftMode === 'position') {
      Object.assign(waypoint, pixelToWorld(pixel.x, pixel.y));
      pose.querySelector('[data-field=x]').value = waypoint.x;
      pose.querySelector('[data-field=y]').value = waypoint.y;
    } else {
      const marker = screenPoint(waypoint);
      if (Math.hypot(point.x - marker.x, point.y - marker.y) >= 1) {
        waypoint.angle_radians = Math.atan2(marker.y - point.y, point.x - marker.x) + state.map.origin[2];
        pose.querySelector('[data-field=angle_radians]').value = degrees(waypoint.angle_radians);
      }
    }
    return true;
  }

  canvas.addEventListener('wheel', event => {
    if (!state.view) return;
    event.preventDefault();
    zoom(event.deltaY < 0 ? 1.22 : 1 / 1.22, eventPoint(event));
  }, {passive: false});

  let canvasClickWaypoint = -1;
  let lastCanvasClick = null;

  canvas.addEventListener('pointerdown', event => {
    if (!state.view || state.drag || event.button !== 0) return;
    canvasClickWaypoint = -1;
    if (followingWaypoint()) {
      canvas.focus({preventScroll: true});
      return;
    }
    const point = eventPoint(event);
    const pixel = screenToPixel(point);
    let type = 'pan', index = -1;
    if (['add', 'landmark', 'pen', 'eraser', 'cost', 'cost-eraser'].includes(state.mode)) {
      const costMode = state.mode === 'cost' || state.mode === 'cost-eraser';
      const paintMap = costMode ? costMap() : state.map;
      const paintPixel = costMode ? screenToMapPixel(point, paintMap) : pixel;
      const brush = ['pen', 'eraser', 'cost', 'cost-eraser'].includes(state.mode);
      if (!paintMap || (brush ? !insideMap(paintPixel, paintMap) : !insidePlacement(pixel))) return;
      if (state.mode === 'add' && !leaveWaypointEdit()) return;
      type = state.mode;
    } else if (state.mode === 'select' && layerVisible('waypoints')) {
      let distance = event.pointerType === 'touch' ? 23 : 12;
      rows().forEach((item, candidate) => {
        const waypoint = candidate === state.selected && state.draft ? state.draft : item;
        const marker = screenPoint(waypoint);
        const separation = Math.hypot(point.x - marker.x, point.y - marker.y);
        if (separation < distance) { distance = separation; index = candidate; }
      });
      if (index >= 0) {
        if (!select(index)) return;
        canvasClickWaypoint = index;
        canvas.focus({preventScroll: true});
        if (!state.draft) return;
        type = event.shiftKey ? 'heading' : 'move';
      }
    }
    const marker = index >= 0 ? screenPoint(state.draft) : point;
    const drag = {id: event.pointerId, type, point, marker, view: {...state.view}, moved: false};
    state.drag = drag;
    if (state.draft) drag.draftBefore = structuredClone(state.draft);
    if (type === 'add') {
      drag.before = snapshot();
      state.route ??= {format_version: state.format.version, waypoints: []};
      const number = rows().reduce((max, row) => Math.max(max, row.number), 0) + 1;
      rows().push({number, ...pixelToWorld(pixel.x, pixel.y), angle_radians: 0});
      state.selected = rows().length - 1;
    } else if (type === 'landmark') {
      const names = new Set(state.landmarks.map(item => item.name));
      let number = state.landmarks.length + 1;
      while (names.has('Landmark ' + number)) number++;
      drag.landmark = {name: 'Landmark ' + number, ...pixelToWorld(pixel.x, pixel.y), yaw: 0, aliases: []};
    } else if (['pen', 'eraser', 'cost', 'cost-eraser'].includes(type)) {
      const costMode = type === 'cost' || type === 'cost-eraser';
      const paintMap = costMode ? costMap() : state.map;
      const paintPixel = costMode ? screenToMapPixel(point, paintMap) : pixel;
      drag.stroke = {
        tool: type, size: Number(el('me-brush').value),
        points: [[Math.floor(paintPixel.x), Math.floor(paintPixel.y)]],
      };
      if (costMode) {
        drag.stroke.value = type === 'cost-eraser' ? 0 : state.costPaint.value;
        drag.stroke.color = type === 'cost-eraser' ? [0, 0, 0] : state.costPaint.color.slice();
      }
    }
    canvas.focus({preventScroll: true});
    canvas.setPointerCapture(event.pointerId);
    draw();
  });

  canvas.addEventListener('pointermove', event => {
    if (!state.view) return;
    const point = eventPoint(event), pixel = screenToPixel(point);
    const {x, y} = pixelToWorld(pixel.x, pixel.y);
    el('me-coordinates').textContent = 'X ' + x.toFixed(3) + ' m · Y ' + y.toFixed(3) + ' m';
    if (followingWaypoint()) {
      if (followWaypoint(point)) draw();
      return;
    }
    const drag = state.drag;
    const costMode = state.mode === 'cost' || state.mode === 'cost-eraser';
    if (['pen', 'eraser', 'cost', 'cost-eraser'].includes(state.mode)) {
      const paintMap = costMode ? costMap() : state.map;
      const paintPixel = costMode ? screenToMapPixel(point, paintMap) : pixel;
      state.brushCursor = paintMap && insideMap(paintPixel, paintMap) ? point : null;
      if (drag?.stroke && drag.id === event.pointerId) {
        const points = drag.stroke.points;
        const next = [Math.max(0, Math.min(paintMap.width - 1, Math.floor(paintPixel.x))),
          Math.max(0, Math.min(paintMap.height - 1, Math.floor(paintPixel.y)))];
        const [lastX, lastY] = points.at(-1);
        if (next[0] !== lastX || next[1] !== lastY) points.push(next);
      }
      draw();
      return;
    }
    if (!drag || drag.id !== event.pointerId) return;
    const dx = point.x - drag.point.x, dy = point.y - drag.point.y;
    drag.moved ||= Math.hypot(dx, dy) > 3;
    if (!drag.moved) return;
    canvasClickWaypoint = -1;
    if (drag.type === 'pan') {
      state.view.x = drag.view.x + dx;
      state.view.y = drag.view.y + dy;
    } else if (drag.type === 'move') {
      const destination = screenToPixel({x: drag.marker.x + dx, y: drag.marker.y + dy});
      if (insidePlacement(destination)) Object.assign(state.draft, pixelToWorld(destination.x, destination.y));
    } else {
      const angle = Math.atan2(drag.marker.y - point.y, point.x - drag.marker.x) + state.map.origin[2];
      if (drag.landmark) drag.landmark.yaw = angle;
      else (state.draft || current()).angle_radians = angle;
    }
    draw();
  });

  function finishDrag(event, cancel = false) {
    const drag = state.drag;
    if (!drag || drag.id !== event.pointerId) return;
    state.drag = null;
    if (cancel) canvasClickWaypoint = -1;
    if (drag.type === 'pan') {
      if (cancel) state.view = drag.view;
      draw();
      return;
    }
    if (drag.stroke) {
      const costStroke = drag.stroke.tool.startsWith('cost');
      if (!cancel) edit(() => {
        if (costStroke) state.costStrokes = [...state.costStrokes, drag.stroke];
        else state.strokes = [...state.strokes, drag.stroke];
      }, costStroke);
      else draw();
      return;
    }
    if (drag.landmark) {
      if (!cancel) {
        edit(() => { state.landmarks = [...state.landmarks, drag.landmark]; });
        el('me-landmark-panel').open = true;
      } else draw();
      return;
    }
    if (drag.type === 'add') {
      if (cancel) Object.assign(state, drag.before);
      else {
        remember(drag.before);
      }
      render();
      scrollSelectedWaypoint();
      return;
    }
    if (cancel) state.draft = drag.draftBefore;
    refreshWaypointSelection();
  }

  canvas.addEventListener('pointerup', event => finishDrag(event));
  canvas.addEventListener('pointercancel', event => finishDrag(event, true));
  canvas.addEventListener('lostpointercapture', event => finishDrag(event, true));
  canvas.addEventListener('click', event => {
    if (!state.view || state.mode !== 'select' || event.detail === 0) return;
    const following = followingWaypoint();
    const index = following ? state.selected : canvasClickWaypoint;
    canvasClickWaypoint = -1;
    const previous = lastCanvasClick;
    lastCanvasClick = clickSequence(previous, event, index);
    if (following) {
      // The third click of the original gesture switches to heading editing.
      if (state.draftMode === 'position' && previous?.count === 2 && lastCanvasClick.count === 3) {
        beginWaypointEdit('heading');
      } else if (followWaypoint(eventPoint(event))) {
        applyWaypointEdit();
      }
    } else if (index >= 0 && index === state.selected && isMultiClick(lastCanvasClick.count)) {
      beginWaypointEdit(clickEditMode(lastCanvasClick.count));
    }
  });
  canvas.addEventListener('pointerleave', () => {
    if (!state.brushCursor) return;
    state.brushCursor = null;
    draw();
  });
  canvas.addEventListener('keydown', event => {
    if (!(event.ctrlKey || event.metaKey) || event.key.toLowerCase() !== 'z') return;
    event.preventDefault();
    historyStep(!event.shiftKey);
  });
  let canvasWidth = 0, canvasHeight = 0;
  new ResizeObserver(() => {
    const width = canvas.clientWidth, height = canvas.clientHeight;
    if (!width || !height) return;
    if (!state.view || state.view.scale === state.view.fit) fit();
    else {
      if (canvasWidth && canvasHeight) {
        state.view.x += (width - canvasWidth) / 2;
        state.view.y += (height - canvasHeight) / 2;
        state.view.fit = fitScale(width, height);
      }
      draw();
    }
    canvasWidth = width;
    canvasHeight = height;
  }).observe(canvas);

  function moveOrder(offset) {
    const destination = state.selected + offset;
    if (state.draft || !current() || destination < 0 || destination >= rows().length) return;
    edit(() => {
      [rows()[state.selected], rows()[destination]] = [rows()[destination], rows()[state.selected]];
      state.selected = destination;
    });
    scrollSelectedWaypoint();
  }

  function reorderWaypoint(from, to) {
    if (state.draft || !Number.isSafeInteger(from) || from === to ||
        from < 0 || to < 0 || from >= rows().length || to >= rows().length) return;
    edit(() => {
      const [item] = rows().splice(from, 1);
      rows().splice(to, 0, item);
      state.selected = to;
    });
    scrollSelectedWaypoint();
  }

  function removeWaypoint(index) {
    const target = rows()[index];
    if (!target || !confirm('Delete waypoint ' + target.number + '?')) return false;
    if (state.selected === index) state.draft = null;
    edit(() => {
      rows().splice(index, 1);
      if (state.selected > index) state.selected--;
      else if (state.selected === index) state.selected = Math.min(index, rows().length - 1);
    });
    return true;
  }

  function drawLandmarks(context) {
    const draft = state.drag?.landmark;
    for (const mark of draft ? [...state.landmarks, draft] : state.landmarks) {
      const point = screenPoint(mark);
      context.save();
      context.translate(point.x, point.y);
      context.fillStyle = '#7c36b0';
      context.strokeStyle = '#fff';
      context.lineWidth = 1.5;
      context.beginPath();
      context.moveTo(0, -8);
      context.lineTo(7, 0);
      context.lineTo(0, 8);
      context.lineTo(-7, 0);
      context.closePath();
      context.fill();
      context.stroke();
      context.font = 'bold 11px sans-serif';
      context.fillText(mark.name, 10, -8);
      context.rotate(state.map.origin[2] - mark.yaw);
      context.strokeStyle = context.fillStyle;
      context.beginPath();
      context.moveTo(7, 0);
      context.lineTo(17, 0);
      context.stroke();
      context.restore();
    }
  }

  function renderLandmarks() {
    el('me-landmark-count').textContent = String(state.landmarks.length);
    const list = el('me-landmark-list');
    const scroll = list.scrollTop;
    const content = document.createDocumentFragment();
    state.landmarks.forEach((mark, index) => {
      const row = node('div', '', 'me-landmark');
      const rename = node('button', mark.name, 'me-waypoint');
      rename.type = 'button';
      rename.title = 'Rename landmark';
      rename.onclick = () => {
        const name = prompt('Landmark name', mark.name)?.trim();
        if (!name || name === mark.name) return;
        if (name.length > 120 || state.landmarks.some(item => item.name === name)) {
          status('Enter a unique landmark name of at most 120 characters.', true);
          return;
        }
        edit(() => { state.landmarks = state.landmarks.map((item, i) => i === index ? {...item, name} : item); });
      };
      const remove = node('button', '×', 'hbtn');
      remove.type = 'button';
      remove.setAttribute('aria-label', 'Delete ' + mark.name);
      remove.onclick = () => edit(() => { state.landmarks = state.landmarks.filter((_, i) => i !== index); });
      row.append(rename, remove);
      content.append(row);
    });
    if (!state.landmarks.length) content.append(node('p', 'No landmarks.', 'me-hint'));
    list.replaceChildren(content);
    list.scrollTop = scroll;
  }

  function landmarksDocument() {
    const {image, resolution, origin} = state.map;
    return {format_version: '1.0', map: {image: basename(image), resolution, origin}, landmarks: state.landmarks};
  }

  function importLandmarks(text) {
    if (!state.map) throw Error('Open the landmark map first.');
    const data = JSON.parse(text);
    if (!Array.isArray(data?.landmarks) || data.landmarks.length > 1000) throw Error('Expected a landmarks array (up to 1,000).');
    if (data.map && (data.map.resolution != null && Math.abs(data.map.resolution - state.map.resolution) > 1e-6 ||
        data.map.origin != null && (!Array.isArray(data.map.origin) || data.map.origin.length !== 3 ||
          !data.map.origin.every((value, index) => Number.isFinite(value) && Math.abs(value - state.map.origin[index]) < 1e-6)))) {
      throw Error('The landmark origin or resolution differs from the open map.');
    }
    const names = new Set();
    const landmarks = data.landmarks.map((item, index) => {
      const name = (typeof item?.name === 'string' ? item.name.trim() : '') || 'Landmark ' + (index + 1);
      const yaw = item?.yaw ?? item?.angle ?? 0;
      const aliases = item?.aliases ?? [];
      if (name.length > 120 || names.has(name) || ![item?.x, item?.y, yaw].every(Number.isFinite) ||
          !Array.isArray(aliases) || !aliases.every(alias => typeof alias === 'string')) {
        throw Error('Invalid landmark ' + (index + 1) + '.');
      }
      names.add(name);
      return {...item, name, yaw, aliases};
    });
    edit(() => { state.landmarks = landmarks; });
    state.savedLandmarks = landmarks;
    status('Imported ' + landmarks.length + ' landmarks.');
  }

  const fileInput = node('input');
  fileInput.type = 'file';
  fileInput.hidden = true;
  document.body.append(fileInput);
  function importFile(accept, maxBytes, load, report = status) {
    fileInput.value = '';
    fileInput.accept = accept;
    fileInput.onchange = async () => {
      const file = fileInput.files[0];
      if (!file) return;
      try {
        if (file.size > maxBytes) throw Error('Import file is too large.');
        await load(await file.text());
      } catch (error) {
        report(error.message, true);
      }
    };
    fileInput.click();
  }

  function download(name, text, type) {
    const link = node('a');
    const url = URL.createObjectURL(new Blob([text], {type}));
    link.href = url;
    link.download = name;
    document.body.append(link);
    link.click();
    link.remove();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  }

  let activeFormatYaml = "version: '1.0'\nformat:\n" +
    Object.entries(defaultFormat.format).map(([key, type]) => '  ' + key + ': ' + type).join('\n') + '\n';
  let formatOverride = false;
  el('me-format').value = activeFormatYaml;
  let formatRequest = 0;
  function formatStatus(message, error = false) {
    const box = el('me-format-status');
    box.textContent = message;
    box.classList.toggle('error', error);
  }
  async function applyFormat(text, announce = true) {
    const request = ++formatRequest;
    try {
      const result = await post('format', {yaml: text});
      if (request !== formatRequest) return;
      state.format = result.document;
      formatOverride = true;
      el('me-format').value = activeFormatYaml = result.yaml;
      try { localStorage.setItem('sirius-waypoint-format', activeFormatYaml); } catch (error) { /* storage is optional */ }
      renderInspector();
      if (announce) formatStatus('Format updated.');
    } catch (error) {
      if (request === formatRequest) formatStatus(error.message, true);
    }
  }

  const picker = node('dialog', '', 'me-dialog');
  picker.setAttribute('aria-label', 'Choose a map or waypoint file');
  document.body.append(picker);
  let pickerRequest = 0, mapRequest = 0, routeRequest = 0;
  let rootPromise;
  function getRoots() {
    rootPromise ??= get('/api/map-editor/roots').catch(error => { rootPromise = null; throw error; });
    return rootPromise;
  }

  function closeButton(dialog) {
    const button = node('button', 'Close', 'hbtn');
    button.type = 'button';
    button.onclick = () => dialog.close();
    return button;
  }

  async function openMap(path, requireSemantic = false) {
    const request = ++mapRequest;
    status('Loading map…');
    try {
      const map = await get('/api/map-editor/map', path);
      if (request !== mapRequest) return;
      if (requireSemantic && !map.semantic) throw Error('Choose an indexed PGM with its road semantic JSON.');
      const image = new Image();
      const preview = map.semantic ? '/api/map-editor/semantic-preview' : '/api/files/preview';
      image.src = preview + '?' + new URLSearchParams({
        path: map.semantic?.palette_path || map.image, revision: map.image_revision,
      });
      try {
        await image.decode();
      } catch (error) {
        throw Error(map.semantic ? 'Could not render the indexed PGM with its JSON palette.' : 'Could not display this PGM.');
      }
      if (request !== mapRequest) return;
      const layer = map.semantic ? 'semantic' : 'map';
      const replacesBase = layer === 'map' && state.rasters.map && state.rasters.map.map.revision !== map.revision;
      if (replacesBase) {
        if ((drawingDirty() || landmarksDirty()) && !confirm('Discard unsaved drawing and landmarks?')) {
          status('');
          return;
        }
        state.strokes = state.savedStrokes = [];
        state.landmarks = state.savedLandmarks = [];
        state.undo.length = state.redo.length = 0;
      }
      state.rasters[layer] = {map, image, tiles: new Map()};
      state.layers[layer].visible = true;
      page.querySelector('[data-layer="' + layer + '"]').checked = true;
      let warning = '';
      if (layer === 'semantic') {
        state.rasters.texture = null;
        initCostmap(map);
        if (map.texture) {
          const textureMap = {...map, path: map.texture.path, image: map.texture.path,
            image_revision: map.texture.revision, revision: map.texture.revision,
            semantic: null, editable: false, width: map.texture.width, height: map.texture.height};
          const texture = new Image();
          texture.src = '/api/map-editor/texture-preview?' + new URLSearchParams({
            path: map.semantic.palette_path, revision: map.revision,
          });
          try {
            await texture.decode();
            if (request !== mapRequest) return;
            state.rasters.texture = {map: textureMap, image: texture, tiles: new Map()};
            state.layers.texture.visible = true;
            page.querySelector('[data-layer="texture"]').checked = true;
          } catch (error) {
            warning = 'The texture image could not be loaded. Continuing with the indexed PGM.';
          }
        } else if (map.texture_error) {
          warning = map.texture_error;
        }
      } else {
        state.rasters.texture = null;
      }
      state.view = null;
      if (layer === 'map') setMode('select');
      else el('me-layer-panel').open = true;
      fit();
      render();
      status(warning, Boolean(warning));
    } catch (error) {
      if (request === mapRequest) status(error.message || 'Could not display this PGM.', true);
    }
  }

  const reservedClasses = new Set(['unknown', 'wall', 'floor']);
  function costStatus(message, error = false) {
    const box = el('me-cost-status');
    box.textContent = message;
    box.classList.toggle('error', error);
  }

  function costClass(id) {
    return state.classLabels.find(item => String(item.id) === String(id));
  }

  function syncCostPaint() {
    const swatch = el('me-cost-swatch');
    swatch.style.backgroundColor = 'rgb(' + state.costPaint.color.join(',') + ')';
    const value = el('me-cost-value');
    if (document.activeElement !== value) value.value = state.costPaint.value;
    const picker = el('me-cost-class-select');
    if (document.activeElement !== picker) picker.value = state.costPaint.id == null ? '' : String(state.costPaint.id);
  }

  function renderCostClasses() {
    const table = el('me-cost-classes');
    table.replaceChildren();
    const rows = state.classLabels.filter(item => !reservedClasses.has(String(item.name).toLowerCase()));
    if (!rows.length) {
      table.append(node('p', 'This road map has no editable semantic classes.', 'me-hint'));
    } else {
      const head = node('div', '', 'me-cost-row me-cost-head');
      head.append(node('span', 'Class'), node('span', 'Global'), node('span', 'Local'));
      table.append(head);
      for (const item of rows) {
        const row = node('div', '', 'me-cost-row');
        const label = node('label');
        const swatch = node('span', '', 'me-legend-swatch');
        swatch.style.backgroundColor = 'rgb(' + item.rgb.join(',') + ')';
        label.append(swatch, node('span', item.name));
        row.append(label);
        for (const key of ['global_cost', 'local_cost']) {
          const input = node('input');
          input.type = 'number';
          input.min = '0';
          input.max = '100';
          input.step = '1';
          input.value = state.classCosts[item.id]?.[key] ?? item[key];
          input.setAttribute('aria-label', item.name + ' ' + key.replace('_', ' '));
          input.onchange = () => {
            const value = Math.max(0, Math.min(100, Math.round(Number(input.value))));
            if (!Number.isFinite(value)) {
              input.value = state.classCosts[item.id]?.[key] ?? item[key];
              return;
            }
            input.value = value;
            state.classCosts[item.id] ??= {global_cost: item.global_cost, local_cost: item.local_cost};
            state.classCosts[item.id][key] = value;
            if (state.costPaint.id === item.id) {
              state.costPaint.value = Math.max(state.classCosts[item.id].global_cost,
                state.classCosts[item.id].local_cost);
              syncCostPaint();
            }
            scheduleCostmap();
          };
          row.append(input);
        }
        table.append(row);
      }
    }
    const picker = el('me-cost-class-select');
    picker.replaceChildren(node('option', 'Custom value'));
    picker.firstChild.value = '';
    for (const item of rows) {
      const option = node('option', item.name + (maxCost(item) ? ' · ' + maxCost(item) : ''));
      option.value = String(item.id);
      picker.append(option);
    }
    picker.onchange = () => {
      if (picker.value === '') {
        state.costPaint.id = null;
        syncCostPaint();
        return;
      }
      const item = costClass(Number(picker.value));
      state.costPaint.id = item.id;
      state.costPaint.value = maxCost(item);
      state.costPaint.color = item.rgb.slice();
      syncCostPaint();
    };
    syncCostPaint();
  }

  function maxCost(item) {
    const costs = state.classCosts[item.id];
    return Math.max(costs?.global_cost ?? item.global_cost, costs?.local_cost ?? item.local_cost);
  }

  function resetCostmapControls() {
    for (const [id, key] of [['me-inflation-radius', 'inflation_radius'],
      ['me-inflation-scaling', 'cost_scaling'], ['me-inflation-inscribed', 'inscribed_radius'],
      ['me-inflation-occupied', 'occupied_thresh']]) {
      el(id).value = state.inflation[key];
    }
    el('me-cost-combine').value = state.combine;
    renderCostClasses();
  }

  function initCostmap(map) {
    state.costRequest++;
    state.costToken = '';
    state.rasters.costmap = null;
    state.rasters.inflation = null;
    state.costStrokes = [];
    state.classLabels = map.semantic?.labels || [];
    state.classCosts = Object.fromEntries(state.classLabels.map(item => [item.id, {
      global_cost: item.global_cost, local_cost: item.local_cost,
    }]));
    state.inflation = {...costDefaults};
    state.combine = 'max';
    const first = state.classLabels.find(item => !reservedClasses.has(String(item.name).toLowerCase()) &&
      Math.max(item.global_cost, item.local_cost) > 0);
    state.costPaint = first
      ? {id: first.id, value: Math.max(first.global_cost, first.local_cost), color: first.rgb.slice()}
      : {id: null, value: 50, color: [255, 80, 40]};
    state.layers.costmap.visible = true;
    state.layers.inflation.visible = false;
    page.querySelector('[data-layer="costmap"]').checked = true;
    page.querySelector('[data-layer="inflation"]').checked = false;
    resetCostmapControls();
    prepareCostmap();
  }

  function clearCostmap() {
    state.costRequest++;
    clearTimeout(state.costTimer);
    state.costToken = '';
    state.rasters.costmap = null;
    state.rasters.inflation = null;
    state.classLabels = [];
    state.classCosts = {};
    state.costStrokes = [];
    costStatus('');
  }

  function scheduleCostmap(delay = 220) {
    if (!state.rasters.semantic) return;
    clearTimeout(state.costTimer);
    state.costTimer = setTimeout(() => prepareCostmap(), delay);
  }

  async function loadCostmapRaster(kind, map, token) {
    const image = new Image();
    image.src = '/api/map-editor/costmap-preview?' + new URLSearchParams({token, kind});
    try {
      await image.decode();
    } catch (error) {
      throw Error('Could not render the ' + kind + ' layer.');
    }
    const rasterMap = {...map, path: token, image: token, image_revision: token,
      revision: token, semantic: null, editable: false};
    return {map: rasterMap, image, tiles: new Map()};
  }

  async function prepareCostmap() {
    const semantic = state.rasters.semantic?.map;
    if (!semantic) return;
    const request = ++state.costRequest;
    state.generatedPath = null;
    state.showGeneratedPath = false;
    costStatus('Preparing Nav2 cost map…');
    try {
      const base = state.rasters.map?.map;
      const result = await post('costmap', {
        path: semantic.semantic.palette_path, revision: semantic.revision,
        base: base && base !== semantic ? {path: base.path, revision: base.revision} : null,
        labels: state.classCosts, combine: state.combine, inflation: state.inflation,
        strokes: state.costStrokes,
      });
      if (request !== state.costRequest || !state.rasters.semantic) return;
      const costmap = await loadCostmapRaster('costmap', semantic, result.token);
      if (request !== state.costRequest) return;
      const inflation = await loadCostmapRaster('inflation', semantic, result.token);
      if (request !== state.costRequest) return;
      state.costToken = result.token;
      state.rasters.costmap = costmap;
      state.rasters.inflation = inflation;
      const stats = result.stats;
      costStatus(stats.cost_cells.toLocaleString() + ' cost cells · ' +
        stats.lethal_cells.toLocaleString() + ' lethal · inflation max ' + stats.inflation_max);
      renderMapLayers();
      render();
      draw();
    } catch (error) {
      if (request === state.costRequest) costStatus(error.message, true);
    }
  }

  function renderMapLayers() {
    const base = state.rasters.map?.map;
    const semantic = state.rasters.semantic?.map;
    const texture = state.rasters.texture?.map;
    const label = map => basename(map.path) + (map.semantic ? ' · Road Semantic' : '') +
      ' · ' + map.width + ' × ' + map.height + ' px';
    el('me-empty').hidden = Boolean(state.map);
    el('me-map-label').textContent = state.map ? label(state.map) + (texture ? ' · Texture Image' : '') : 'No map';
    el('me-semantic-label').hidden = !(base && semantic);
    if (base && semantic) el('me-semantic-label').textContent = label(semantic);
    el('me-remove-semantic').hidden = !semantic;
    el('me-costmap-panel').hidden = !semantic;
    for (const [key, available] of [['map', base], ['semantic', semantic], ['texture', texture],
      ['costmap', state.rasters.costmap], ['inflation', state.rasters.inflation]]) {
      page.querySelector('[data-layer="' + key + '"]').disabled = !available;
      page.querySelector('[data-opacity="' + key + '"]').disabled = !available;
    }
    const legend = el('me-semantic-legend');
    const items = (semantic?.semantic.legend || []).map(({name, rgb}) => {
      const item = node('li');
      const swatch = node('span', '', 'me-legend-swatch');
      swatch.style.backgroundColor = 'rgb(' + rgb.join(',') + ')';
      swatch.setAttribute('aria-hidden', 'true');
      item.append(swatch, name.replaceAll('_', ' '));
      return item;
    });
    legend.replaceChildren(...items);
    legend.hidden = !items.length;
  }

  async function openRoute(path) {
    if (!replaceAllowed()) return;
    const request = ++routeRequest;
    const previous = JSON.stringify(state.route);
    const previousDraft = JSON.stringify(state.draft);
    status('Loading waypoints…');
    try {
      const {document, path: resolved} = await get('/api/map-editor/waypoints', path);
      if (request !== routeRequest || ((JSON.stringify(state.route) !== previous ||
          JSON.stringify(state.draft) !== previousDraft) && !replaceAllowed())) return;
      setRoute(document, resolved);
      status('');
    } catch (error) {
      if (request === routeRequest) status(error.message, true);
    }
  }

  async function pick(kind, path) {
    const request = ++pickerRequest;
    const isMap = kind !== 'route';
    const heading = node('h2', {map: 'Open Map', semantic: 'Open Semantic Map', route: 'Open Waypoints'}[kind]);
    const location = node('p', 'Loading…', 'muted');
    const controls = node('div', '', 'me-actions');
    const search = node('input');
    search.type = 'search';
    search.placeholder = 'Filter by name';
    search.setAttribute('aria-label', 'Filter by filename');
    const list = node('div', '', 'me-picker-list');
    picker.replaceChildren(heading, location, controls, search, list, closeButton(picker));
    if (!picker.open) picker.showModal();
    try {
      const roots = await getRoots();
      const allowed = isMap ? [roots.maps, roots.archive] : [roots.waypoints];
      const within = candidate => allowed.some(root => candidate === root || candidate?.startsWith(root + '/'));
      const data = await get('/api/files/list', path ?? (kind === 'semantic' && roots.semantic ? roots.semantic : allowed[0]));
      if (request !== pickerRequest || !picker.open) return;
      if (!within(data.path)) throw Error('Choose a folder under Map & Waypoint.');
      location.textContent = data.path;
      function addPickerButton(label, action) {
        const button = node('button', label, 'hbtn');
        button.type = 'button';
        button.onclick = action;
        controls.append(button);
      }
      if (isMap) {
        addPickerButton('Maps', () => pick(kind, roots.maps));
        if (roots.semantic) addPickerButton('Road Semantic Maps', () => pick(kind, roots.semantic));
        addPickerButton('Archive', () => pick(kind, roots.archive));
      }
      if (within(data.parent)) addPickerButton('Parent Folder', () => pick(kind, data.parent));
      const names = new Set(data.entries.filter(item => !item.directory).map(item => item.name.toLowerCase()));
      const items = data.entries.filter(item => {
        if (item.directory || /\.ya?ml$/i.test(item.name)) return true;
        if (!isMap) return false;
        return /\.pgm$/i.test(item.name) ||
          (/\.json$/i.test(item.name) && names.has(item.name.toLowerCase().replace(/\.json$/, '.pgm')));
      });
      function showItems() {
        const filter = search.value.trim().toLocaleLowerCase();
        const content = document.createDocumentFragment();
        for (const item of items) {
          if (!item.name.toLocaleLowerCase().includes(filter)) continue;
          const button = node('button', (item.directory ? '▰ ' : '') + item.name, 'me-picker-item');
          button.type = 'button';
          button.onclick = () => {
            if (item.directory) return pick(kind, item.path);
            picker.close();
            if (isMap) openMap(item.path, kind === 'semantic');
            else openRoute(item.path);
          };
          content.append(button);
        }
        if (!content.childNodes.length) content.append(node('p', 'No matching files.', 'muted'));
        list.replaceChildren(content);
      }
      search.oninput = showItems;
      showItems();
    } catch (error) {
      if (request === pickerRequest) location.textContent = error.message;
    }
  }

  const save = node('dialog', '', 'me-dialog');
  save.setAttribute('aria-label', 'Export map editor files');
  const form = node('form');
  form.innerHTML = `
    <h2>Export</h2>
    <label class="me-save-name">File type<select name="kind">
      <option value="route">Waypoints YAML</option>
      <option value="map">Edited PGM + map YAML</option>
      <option value="cost">Cost Map PGM + YAML (Nav2 raw)</option>
      <option value="landmarks">Landmarks JSON</option>
    </select></label>
    <p class="muted" id="me-export-location"></p>
    <label class="me-save-name">Name<input name="name" required maxlength="240"></label>
    <p class="me-status error" role="alert"></p>
    <div class="me-actions"><button type="submit" class="primary-button">Export Copy</button></div>`;
  const saveClose = closeButton(save);
  form.querySelector('.me-actions').prepend(saveClose);
  save.append(form);
  document.body.append(save);
  let saving = false;
  save.addEventListener('cancel', event => { if (saving) event.preventDefault(); });
  function setExportName() {
    const kind = form.elements.kind.value;
    const mapName = state.map ? basename(state.map.path).replace(/\.ya?ml$/i, '') : 'map';
    const routeName = state.path ? basename(state.path).replace(/\.ya?ml$/i, '') : 'waypoints';
    form.elements.name.value = {route: routeName + '-edit.yaml', map: mapName + '-edit',
      cost: mapName + '-costmap', landmarks: mapName + '.landmarks.json'}[kind];
    el('me-export-location').textContent = {
      route: 'A new file in Map & Waypoint / waypoints.',
      map: 'A new folder in Map & Waypoint / maps.',
      cost: 'A Nav2 raw-mode cost map folder in Map & Waypoint / maps.',
      landmarks: 'Download the AMR_TOOLKIT landmarks file.',
    }[kind];
    form.querySelector('[role=alert]').textContent = '';
  }
  form.elements.kind.onchange = setExportName;

  function exportRouteDocument() {
    const document = structuredClone(state.route);
    if (formatOverride) document.format_version = state.format.version;
    document.waypoints = document.waypoints.map(row => {
      if (Object.hasOwn(state.format.format, 'angle_degrees') || Object.hasOwn(row, 'angle_degrees')) {
        row.angle_degrees = degrees(row.angle_radians);
      }
      const fields = Object.keys(state.format.format).filter(key => Object.hasOwn(row, key));
      return {...Object.fromEntries(fields.map(key => [key, row[key]])), ...row};
    });
    return document;
  }

  form.onsubmit = async event => {
    event.preventDefault();
    if (saving) return;
    const submit = form.querySelector('button[type=submit]');
    const errorBox = form.querySelector('[role=alert]');
    const name = form.elements.name.value.trim();
    const kind = form.elements.kind.value;
    if (!name) {
      errorBox.textContent = 'Enter a name.';
      return;
    }
    saving = submit.disabled = saveClose.disabled = form.elements.kind.disabled = true;
    errorBox.textContent = '';
    try {
      let filename;
      if (kind === 'route') {
        if (draftDirty()) throw Error('Close this dialog and apply the waypoint changes first.');
        const document = exportRouteDocument();
        const result = await post('waypoints', {name: /\.ya?ml$/i.test(name) ? name : name + '.yaml', document});
        state.route = document;
        state.saved = JSON.stringify(document);
        state.path = result.path;
        filename = result.name;
      } else if (kind === 'map') {
        const {path, revision} = state.map;
        const strokes = state.strokes;
        const result = await post('map', {name, path, revision, strokes});
        state.savedStrokes = strokes;
        filename = result.name + ' (PGM + YAML)';
      } else if (kind === 'cost') {
        if (!state.costToken) throw Error('Wait for the Nav2 cost map preview to finish first.');
        const result = await post('costmap-export', {name, token: state.costToken});
        filename = result.name + ' (PGM + YAML · Nav2 raw)';
      } else {
        filename = /\.json$/i.test(name) ? name : name + '.json';
        download(filename, JSON.stringify(landmarksDocument(), null, 2) + '\n', 'application/json');
        state.savedLandmarks = state.landmarks;
      }
      save.close();
      render();
      status('Exported ' + filename + '.');
    } catch (error) {
      errorBox.textContent = error.message;
    } finally {
      saving = submit.disabled = saveClose.disabled = form.elements.kind.disabled = false;
    }
  };

  el('me-open-map').onclick = () => pick('map');
  el('me-open-semantic').onclick = () => pick('semantic');
  el('me-remove-semantic').onclick = () => {
    mapRequest++;
    clearCostmap();
    state.rasters.semantic = null;
    state.rasters.texture = null;
    state.view = null;
    status('');
    fit();
    render();
  };
  el('me-open-route').onclick = () => pick('route');
  el('me-new-route').onclick = () => {
    if (replaceAllowed()) {
      routeRequest++;
      setRoute({format_version: state.format.version, waypoints: []}, '');
      status('New route created.');
    }
  };
  for (const mode of Object.keys(tools)) el('me-' + mode).onclick = () => setMode(mode);
  el('me-brush').oninput = event => {
    el('me-brush-size').textContent = event.target.value + ' px';
    draw();
  };
  for (const input of page.querySelectorAll('[data-layer]')) {
    input.onchange = () => {
      state.layers[input.dataset.layer].visible = input.checked;
      draw();
    };
  }
  for (const input of page.querySelectorAll('[data-opacity]')) {
    input.oninput = () => {
      const name = input.dataset.opacity;
      state.layers[name].opacity = Number(input.value) / 100;
      const percent = input.value + '%';
      input.setAttribute('aria-valuetext', percent);
      page.querySelector('[data-opacity-value="' + name + '"]').textContent = percent;
      draw();
    };
  }
  for (const [id, key] of [['me-inflation-radius', 'inflation_radius'],
    ['me-inflation-scaling', 'cost_scaling'], ['me-inflation-inscribed', 'inscribed_radius'],
    ['me-inflation-occupied', 'occupied_thresh']]) {
    el(id).onchange = () => {
      const value = Number(el(id).value);
      if (!Number.isFinite(value) || value < 0) {
        el(id).value = state.inflation[key];
        return;
      }
      state.inflation[key] = value;
      el(id).value = value;
      scheduleCostmap();
    };
  }
  el('me-cost-combine').onchange = () => {
    state.combine = el('me-cost-combine').value;
    scheduleCostmap();
  };
  el('me-cost-value').onchange = () => {
    const value = Math.max(0, Math.min(100, Math.round(Number(el('me-cost-value').value))));
    if (!Number.isFinite(value)) {
      el('me-cost-value').value = state.costPaint.value;
      return;
    }
    state.costPaint.id = null;
    state.costPaint.value = value;
    state.costPaint.color = [255, 80, 40];
    el('me-cost-value').value = value;
    el('me-cost-class-select').value = '';
    syncCostPaint();
  };
  el('me-cost-reset').onclick = () => {
    edit(() => {
      state.classCosts = Object.fromEntries(state.classLabels.map(item => [item.id, {
        global_cost: item.global_cost, local_cost: item.local_cost,
      }]));
      state.inflation = {...costDefaults};
      state.combine = 'max';
      const first = state.classLabels.find(item => !reservedClasses.has(String(item.name).toLowerCase()) &&
        Math.max(item.global_cost, item.local_cost) > 0);
      state.costPaint = first
        ? {id: first.id, value: Math.max(first.global_cost, first.local_cost), color: first.rgb.slice()}
        : {id: null, value: 50, color: [255, 80, 40]};
    }, true);
    resetCostmapControls();
    costStatus('Reset to road map defaults.');
  };
  el('me-cost-clear').onclick = () => {
    if (!state.costStrokes.length || !confirm('Clear all manual cost edits?')) return;
    edit(() => { state.costStrokes = []; }, true);
    costStatus('Manual cost edits cleared.');
  };
  el('me-fit').onclick = fit;
  el('me-native').onclick = () => { if (state.view) zoom(1 / state.view.scale); };
  el('me-grid').onclick = event => {
    state.grid = !state.grid;
    event.currentTarget.setAttribute('aria-pressed', String(state.grid));
    draw();
  };
  el('me-zoom-in').onclick = () => zoom(1.35);
  el('me-zoom-out').onclick = () => zoom(1 / 1.35);
  el('me-undo').onclick = () => historyStep(true);
  el('me-redo').onclick = () => historyStep(false);
  el('me-up').onclick = () => moveOrder(-1);
  el('me-down').onclick = () => moveOrder(1);
  el('me-generate-path').onclick = async () => {
    if (state.showGeneratedPath) {
      state.showGeneratedPath = false;
      render();
      draw();
      status('Generated path hidden.');
      return;
    }
    if (!state.costToken || rows().length < 2) return;
    const button = el('me-generate-path');
    button.disabled = true;
    status('Planning a cost-aware path…');
    try {
      const key = routeKey();
      const result = await post('costmap-path', {
        token: state.costToken, combine: state.combine,
        points: rows().map(row => [row.x, row.y]),
      });
      state.generatedPath = result.path;
      state.generatedKey = key;
      state.showGeneratedPath = true;
      render();
      draw();
      status('Generated path: ' + result.planned_segments + ' of ' + result.segments +
        ' segments planned on the cost map.');
    } catch (error) {
      status(error.message, true);
    } finally {
      render();
    }
  };
  el('me-edit-waypoint').onclick = () => {
    if (!current() || state.draft) return;
    beginWaypointEdit('all');
    el('me-edit-controls').scrollIntoView({block: 'nearest', inline: 'nearest'});
    el('me-apply-edit').focus({preventScroll: true});
  };
  function applyWaypointEdit() {
    const draft = state.draft;
    if (!draft) return;
    state.draft = null;
    if (changed(draft, current())) edit(() => { rows()[state.selected] = draft; });
    else render();
  }
  el('me-apply-edit').onclick = () => {
    applyWaypointEdit();
    el('me-edit-waypoint').focus({preventScroll: true});
  };
  function cancelWaypointEdit() {
    state.draft = null;
    render();
    el('me-edit-waypoint').focus({preventScroll: true});
  }
  el('me-cancel-edit').onclick = cancelWaypointEdit;
  page.addEventListener('keydown', event => {
    if (event.key !== 'Escape' || event.isComposing || event.defaultPrevented || !state.draft || state.drag) return;
    event.preventDefault();
    cancelWaypointEdit();
  });
  document.addEventListener('keydown', event => {
    if (event.defaultPrevented || event.isComposing || event.repeat || event.altKey) return;
    if (!(window.isPageVisible ? isPageVisible('map-editor') : document.body.dataset.page === 'map-editor')) return;
    if (document.querySelector('dialog[open], .modal.open, .log.open')) return;
    const target = event.target;
    if (target instanceof Element && target.closest('input, textarea, select, [contenteditable="true"], [contenteditable=""]')) return;
    const modifier = event.ctrlKey || event.metaKey;
    const key = event.key.toLowerCase();
    if (modifier && key === 'z') {
      event.preventDefault();
      historyStep(!event.shiftKey);
      return;
    }
    if (modifier && (key === 'y' || (event.shiftKey && key === 'z'))) {
      event.preventDefault();
      historyStep(false);
      return;
    }
    if (modifier || !state.map || state.drag) return;
    const mode = {v: 'select', h: 'pan', p: 'pen', e: 'eraser', c: 'cost', x: 'cost-eraser',
      w: 'add', l: 'landmark'}[key];
    if (mode) {
      const button = el('me-' + mode);
      if (!button || button.disabled) return;
      event.preventDefault();
      setMode(mode);
      canvas.focus({preventScroll: true});
      return;
    }
    if (key === 'g') {
      event.preventDefault();
      el('me-grid').click();
    } else if (key === 'f') {
      event.preventDefault();
      fit();
    } else if (key === '0') {
      event.preventDefault();
      if (state.view) zoom(1 / state.view.scale);
    } else if (key === '+' || key === '=') {
      event.preventDefault();
      zoom(1.35);
    } else if (key === '-' || key === '_') {
      event.preventDefault();
      zoom(1 / 1.35);
    }
  });
  el('me-delete').onclick = () => removeWaypoint(state.selected);
  el('me-export').onclick = () => {
    if ((!state.route && !state.map) || saving) return;
    const types = form.elements.kind;
    types.options[0].disabled = !state.route;
    types.options[1].disabled = !state.map?.editable;
    types.options[2].disabled = !state.costToken;
    types.options[3].disabled = !state.map;
    types.value = state.map?.editable && (drawingDirty() || !state.route) ? 'map' : state.route ? 'route' : 'landmarks';
    setExportName();
    save.showModal();
    form.elements.name.select();
  };
  el('me-landmarks-import').onclick = () => {
    if (landmarksDirty() && !confirm('Replace unsaved landmarks?')) return;
    importFile('.json,application/json', 512 * 1024, importLandmarks);
  };
  el('me-format-import').onclick = () => importFile('.yaml,.yml', 32 * 1024, applyFormat, formatStatus);
  el('me-format-apply').onclick = () => applyFormat(el('me-format').value);
  el('me-format-export').onclick = () => {
    if (el('me-format').value !== activeFormatYaml) return formatStatus('Update the format before exporting.', true);
    download('waypoint-format.yaml', activeFormatYaml, 'application/yaml');
    formatStatus('Format exported.');
  };
  window.openMapInEditor = path => {
    setPage('map-editor');
    openMap(path);
  };
  window.addEventListener('beforeunload', event => {
    if (!(routeDirty() || draftDirty() || drawingDirty() || landmarksDirty() || state.costStrokes.length ||
        el('me-format').value !== activeFormatYaml)) return;
    event.preventDefault();
    event.returnValue = '';
  });
  try {
    const storedFormat = localStorage.getItem('sirius-waypoint-format');
    if (storedFormat) applyFormat(storedFormat, false);
  } catch (error) { /* storage is optional */ }
  setMode('select');
  render();
})();
