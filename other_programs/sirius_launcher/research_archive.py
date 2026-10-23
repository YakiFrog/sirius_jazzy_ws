"""Read-only research catalogue and explicitly registered result directories."""
import json
import mimetypes
from pathlib import Path
import re
from urllib.parse import quote, unquote, urlsplit

from flask import Blueprint, abort, jsonify, make_response, redirect, send_file

ALLOWED = {'.html', '.js', '.css', '.json', '.png', '.jpg', '.jpeg', '.webp',
           '.gif', '.svg', '.bin', '.csv', '.yaml', '.yml', '.pgm', '.ply', '.md', '.txt', '.py'}
ATTR = re.compile(r'''\b(href|src)\s*=\s*(["'])(.*?)\2''', re.IGNORECASE)
BACK = ('<nav class="research-backbar" style="position:sticky;top:0;z-index:100;'
        'padding:10px 16px;margin-bottom:12px;border-radius:8px;background:#142e35;'
        'font:14px system-ui;box-shadow:0 2px 12px #0002">'
        '<a href="/research/" style="color:#e5f5ee;text-decoration:none">← 研究の進展</a></nav>')


class ResearchArchive:
    def __init__(self, catalogue=None, workspace=None, home=None):
        self.workspace = Path(workspace or Path(__file__).resolve().parents[2]).resolve()
        self.home = Path(home or Path.home()).resolve()
        self.catalogue = Path(catalogue or Path(__file__).with_name('static')/'research/catalogue.json')

    def load(self):
        config = json.loads(self.catalogue.read_text())
        roots = {key: Path(value.format(workspace=self.workspace, home=self.home)).resolve()
                 for key, value in config['bundles'].items()}
        entries = [dict(row) for row in config['entries']]
        registered = {str(roots[row['bundle']]) for row in entries}
        # Only completed top-level outputs are discovered. Never walk the large
        # inference caches, recording directories or the entire workspace.
        maps = self.workspace/'maps_waypoints/maps/road_semantic'
        if maps.is_dir():
            for folder in maps.iterdir():
                if not re.fullmatch(r'(theta|zed)_\d{8}_\d{6}(?:_\d+)?', folder.name):
                    continue
                if not folder.resolve().is_relative_to(maps.resolve()) or str(folder.resolve()) in registered:
                    continue
                try:
                    meta = json.loads((folder/'manifest.json').read_text())
                    origin = json.loads((folder/'source_bag.json').read_text())
                    if meta['status'] != 'complete' or not (folder/'completed.json').exists():
                        continue
                    if not (folder/'images.html').exists():
                        continue
                    key = 'auto-'+folder.name
                    roots[key] = folder.resolve()
                    entries.append(dict(id=key, date=origin['created_at'][:10], category='road',
                        title=('THETA' if folder.name.startswith('theta') else 'ZED')+'の路面地図',
                        summary='BAGから生成した路面テクスチャとセマンティック地図。',
                        status='生成済み', bundle=key, page='images.html',
                        image='indexed.png' if (folder/'indexed.png').exists() else 'overlay.jpg',
                        recording=origin.get('source_bag_name', ''),
                        metrics=[dict(label='反映フレーム', value=f"{meta['frame_count']:,}")],
                        findings=['生成完了が記録された結果を自動追加しています。'],
                        limits=['認識や地図精度の実験評価は、個別の記録で確認してください。'],
                        evidence=[dict(label='生成記録', bundle=key, path='manifest.json')]))
                except (OSError, ValueError, KeyError, TypeError):
                    continue
        return config, roots, entries

    @staticmethod
    def asset(roots, bundle, name):
        if bundle not in roots:
            raise FileNotFoundError(bundle)
        root = roots[bundle]
        path = (root/name).resolve()
        if not path.is_relative_to(root) or not path.is_file() or path.suffix.lower() not in ALLOWED:
            raise FileNotFoundError(name)
        return path

    @staticmethod
    def url(bundle, name):
        return '/research/assets/'+quote(bundle, safe='')+'/'+quote(name, safe='/')

    def public(self):
        config, roots, entries = self.load()
        rows = []
        for item in entries:
            row = {k: v for k, v in item.items() if k not in {'bundle', 'page', 'image', 'evidence'}}
            try:
                self.asset(roots, item['bundle'], item['page'])
                row.update(available=True, url=self.url(item['bundle'], item['page']))
            except (OSError, ValueError):
                row.update(available=False, url=None)
            row['image'] = item.get('thumbnail')
            if not row['image']:
                try:
                    self.asset(roots, item['bundle'], item['image'])
                    row['image'] = self.url(item['bundle'], item['image'])
                except (OSError, KeyError, ValueError):
                    row['image'] = None
            row['evidence'] = []
            for evidence in item.get('evidence', []):
                try:
                    self.asset(roots, evidence['bundle'], evidence['path'])
                    row['evidence'].append(dict(label=evidence['label'],
                        url=self.url(evidence['bundle'], evidence['path'])))
                except (OSError, ValueError):
                    continue
            rows.append(row)
        rows.sort(key=lambda row: (row['date'], row.get('featured', False)), reverse=True)
        return dict(title='Sirius Research', categories=config['categories'], entries=rows)

    def rewrite_html(self, path, text, config, roots):
        def rewrite(match):
            raw = match[3]
            url = urlsplit(raw)
            if url.scheme or url.netloc or raw.startswith(('#', 'data:')):
                return match[0]
            candidate = None
            for prefix, bundle in config.get('aliases', {}).items():
                if url.path.startswith(prefix):
                    candidate = roots[bundle]/unquote(url.path[len(prefix):])
                    break
            if candidate is None and not url.path.startswith('/'):
                candidate = path.parent/unquote(url.path)
            if candidate is not None:
                candidate = candidate.resolve()
                for bundle, root in roots.items():
                    if candidate.is_relative_to(root) and candidate.is_file():
                        target = self.url(bundle, str(candidate.relative_to(root)))
                        if url.query:
                            target += '?'+url.query
                        if url.fragment:
                            target += '#'+url.fragment
                        return match[1]+'='+match[2]+target+match[2]
            return match[0]
        text = ATTR.sub(rewrite, text)
        position = re.search(r'<body\b[^>]*>', text, re.IGNORECASE)
        if position:
            return text[:position.end()]+BACK+text[position.end():]
        position = re.search(r'<h1\b', text, re.IGNORECASE)
        return text[:position.start()]+BACK+text[position.start():] if position else BACK+text


def register_research_archive(app, auth_ok, archive=None):
    archive = archive or ResearchArchive()
    bp = Blueprint('research_archive', __name__)

    @bp.get('/research')
    def research_redirect():
        return redirect('/research/', code=302)

    @bp.get('/research/')
    def research_home():
        if not auth_ok():
            return redirect('/console')
        response = send_file(Path(__file__).with_name('static')/'research/index.html')
        response.headers['Cache-Control'] = 'no-store'
        return response

    @bp.get('/api/research')
    def research_catalogue():
        if not auth_ok():
            return jsonify(error='auth'), 401
        response = jsonify(archive.public())
        response.headers['Cache-Control'] = 'no-store'
        return response

    @bp.get('/research/assets/<bundle>/<path:name>')
    def research_asset(bundle, name):
        if not auth_ok():
            return jsonify(error='auth'), 401
        try:
            config, roots, _ = archive.load()
            path = archive.asset(roots, bundle, name)
        except (OSError, ValueError):
            abort(404)
        if path.suffix.lower() == '.html':
            response = make_response(archive.rewrite_html(path, path.read_text(), config, roots))
            response.mimetype = 'text/html'
            response.headers['Cache-Control'] = 'no-store'
            return response
        # Reproduction scripts linked by the reports are readable source only.
        mime = 'text/plain' if path.suffix.lower() == '.py' else mimetypes.guess_type(path.name)[0]
        response = send_file(path, conditional=True, mimetype=mime)
        response.headers['X-Content-Type-Options'] = 'nosniff'
        return response

    app.register_blueprint(bp)
