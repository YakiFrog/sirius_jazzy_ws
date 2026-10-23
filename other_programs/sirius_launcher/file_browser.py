"""Host files, non-overwriting moves, recoverable deletion and bookmarks."""
import io
import hashlib
import configparser
import ctypes
import errno
from datetime import datetime
import re
import uuid
from urllib.parse import quote, unquote
import json
import os
from pathlib import Path
import stat
import tempfile
import threading
import warnings

from flask import Blueprint, jsonify, request, send_file

TEXT_LIMIT = 512 * 1024
IMAGE_LIMIT = 32 * 1024 * 1024
IMAGE_EXTENSIONS = {'.pgm', '.png', '.jpg', '.jpeg', '.bmp', '.webp', '.tif', '.tiff'}
TEXT_EXTENSIONS = {'.yaml', '.yml', '.json', '.txt', '.md', '.csv', '.py', '.sh', '.xml', '.log', '.ini', '.cfg', '.toml', '.rviz', '.pgm.info'}


class HostFiles:
    def __init__(self, home=None, workspace=None, favorites_file=None):
        self.home = Path(home or Path.home()).resolve()
        self.workspace = Path(workspace or self.home/'sirius_jazzy_ws').resolve()
        self.roots = [self.home]
        self.bookmarks = [dict(label='Map & Waypoint', path=str(self.workspace/'maps_waypoints')),
                          dict(label='ROSBAG', path=str(self.home/'rosbag2_data')),
                          dict(label='Workspace', path=str(self.workspace)),
                          dict(label='ホーム', path=str(self.home))]
        self.favorites_file = Path(favorites_file or self.home/'.config/sirius_launcher/file_favorites.json')
        self.lock = threading.RLock()
        self.trash = self.home/'.local/share/Trash'

    def resolve(self, value):
        if not isinstance(value, str) or '\0' in value:
            raise ValueError('パスが不正です')
        path = Path(value or self.bookmarks[0]['path']).expanduser()
        if not path.is_absolute():
            path = self.home/path
        path = path.resolve(strict=True)
        if not any(path.is_relative_to(root) for root in self.roots):
            raise PermissionError('ホームフォルダの外は参照できません')
        return path

    @staticmethod
    def kind(path):
        return 'image' if path.suffix.lower() in IMAGE_EXTENSIONS else 'text' if path.suffix.lower() in TEXT_EXTENSIONS or path.name in {'README', 'LICENSE', 'Dockerfile'} else 'binary'

    def listing(self, value, hidden=False):
        path = self.resolve(value)
        if not path.is_dir():
            raise ValueError('フォルダを指定してください')
        entries, truncated = [], False
        # No recursive scans: large bags and workspaces remain cheap to browse.
        with os.scandir(path) as children:
            for entry in children:
                if not hidden and entry.name.startswith('.'):
                    continue
                if len(entries) >= 5000:
                    truncated = True
                    break
                try:
                    item = self.resolve(str(path/entry.name))
                    info = item.stat()
                    if not (stat.S_ISDIR(info.st_mode) or stat.S_ISREG(info.st_mode)):
                        continue
                    directory = item.is_dir()
                    entries.append(dict(name=entry.name, path=str(path/entry.name), directory=directory,
                                        symlink=entry.is_symlink(), size=None if directory else info.st_size,
                                        modified=info.st_mtime, kind='directory' if directory else self.kind(item)))
                except (OSError, ValueError):
                    continue
        entries.sort(key=lambda e: (not e['directory'], e['name'].casefold()))
        parent = str(path.parent) if path != self.home and any(path.parent.is_relative_to(root) for root in self.roots) else None
        return dict(path=str(path), parent=parent, entries=entries, truncated=truncated,
                    bookmarks=self.bookmarks, favorites=self.favorites())

    def regular_file(self, value):
        path = self.resolve(value)
        if not stat.S_ISREG(path.stat().st_mode):
            raise ValueError('通常ファイルを指定してください')
        return path

    def text(self, value):
        path = self.regular_file(value)
        if self.kind(path) != 'text':
            raise ValueError('この形式はダウンロードして確認してください')
        with path.open('rb') as stream:
            data = stream.read(TEXT_LIMIT+1)
            info = os.fstat(stream.fileno())
        if b'\0' in data:
            raise ValueError('バイナリファイルはテキスト表示できません')
        encoding = 'utf-8-sig' if data.startswith(b'\xef\xbb\xbf') else 'utf-8'
        try:
            text = data[:TEXT_LIMIT].decode(encoding)
            valid_utf8 = True
        except UnicodeDecodeError:
            text, valid_utf8 = data[:TEXT_LIMIT].decode(encoding, errors='replace'), False
        editable = len(data)<=TEXT_LIMIT and valid_utf8 and os.access(path, os.W_OK)
        return dict(name=path.name, path=str(path), text=text, truncated=len(data)>TEXT_LIMIT,
                    size=info.st_size, editable=editable,
                    revision=self.revision(data, info) if editable else None,
                    edit_note='' if editable else '編集できるのは512 KB以下の、書き込み可能なUTF-8テキストです')

    @staticmethod
    def revision(data, info):
        identity = f'{info.st_dev}:{info.st_ino}:{info.st_mtime_ns}:{info.st_ctime_ns}:{info.st_mode}:'
        return hashlib.sha256(identity.encode()+data).hexdigest()

    def save_text(self, body):
        if not isinstance(body, dict) or not isinstance(body.get('text'), str):
            raise ValueError('保存するテキストを指定してください')
        text, revision = body['text'], body.get('revision')
        if not isinstance(revision, str) or not re.fullmatch('[0-9a-f]{64}', revision):
            raise ValueError('ファイルを開き直してから保存してください')
        if '\0' in text or len(text.encode('utf-8'))>TEXT_LIMIT:
            raise ValueError('テキストは512 KB以下で、NUL文字を含めずに保存してください')
        with self.lock:
            path = self.regular_file(body.get('path'))
            current = self.text(str(path))
            if not current['editable']:
                raise ValueError(current['edit_note'])
            if current['revision'] != revision:
                raise FileExistsError('ほかで変更されています。編集内容を残したまま、最新のファイルと比較してください')
            if path.suffix.lower() == '.json':
                def reject_constant(value):
                    raise ValueError('JSONでは '+value+' を使用できません')
                try:
                    json.loads(text, parse_constant=reject_constant)
                except json.JSONDecodeError as error:
                    raise ValueError(f'JSONの形式が不正です（{error.lineno}行 {error.colno}列）：{error.msg}') from None
                except RecursionError:
                    raise ValueError('JSONの階層が深すぎます') from None
            original = path.read_bytes()
            info = path.stat()
            if self.revision(original, info) != revision:
                raise FileExistsError('保存前にファイルが変更されました。最新のファイルと比較してください')
            # Keep the existing encoding marker and Windows line endings.
            if b'\r\n' in original and b'\n' not in original.replace(b'\r\n', b''):
                text = text.replace('\r\n', '\n').replace('\n', '\r\n')
            data = text.encode('utf-8')
            if original.startswith(b'\xef\xbb\xbf'):
                data = b'\xef\xbb\xbf'+data
            if len(data)>TEXT_LIMIT:
                raise ValueError('保存後のファイルサイズは512 KB以下にしてください')
            backups = self.home/'.local/share/sirius_launcher/file_backups'
            backups.mkdir(parents=True, exist_ok=True, mode=0o700)
            backups = self.resolve(str(backups))
            tail = path.name.encode('utf-8')[-120:].decode('utf-8', errors='ignore')
            backup = backups/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:12]+'_'+tail)
            temporary = None
            try:
                with tempfile.NamedTemporaryFile('wb', dir=path.parent, delete=False) as stream:
                    temporary = stream.name
                    stream.write(data)
                    stream.flush()
                    os.fchmod(stream.fileno(), stat.S_IMODE(info.st_mode)&0o777)
                    os.fsync(stream.fileno())
                latest = self.text(body['path'])
                if latest['path'] != str(path) or latest['revision'] != revision:
                    raise FileExistsError('保存前にファイルが変更されました。最新のファイルと比較してください')
                with os.fdopen(os.open(backup, os.O_WRONLY|os.O_CREAT|os.O_EXCL, 0o600), 'wb') as stream:
                    stream.write(original)
                    stream.flush()
                    os.fsync(stream.fileno())
                latest = self.text(body['path'])
                if latest['path'] != str(path) or latest['revision'] != revision:
                    raise FileExistsError('保存前にファイルが変更されました。最新のファイルと比較してください')
                os.replace(temporary, path)
                temporary = None
            finally:
                if temporary:
                    Path(temporary).unlink(missing_ok=True)
            saved = self.text(str(path))
            return dict(ok=True, path=str(path), revision=saved['revision'],
                        size=saved['size'], backup_path=str(backup))

    def image(self, value):
        from PIL import Image, ImageOps
        path = self.regular_file(value)
        if path.suffix.lower() not in IMAGE_EXTENSIONS or path.stat().st_size > IMAGE_LIMIT:
            raise ValueError('画像形式、またはサイズ上限（32 MB）を確認してください')
        with warnings.catch_warnings():
            warnings.simplefilter('error', Image.DecompressionBombWarning)
            with Image.open(path) as image:
                if image.width*image.height > 40_000_000:
                    raise ValueError('プレビュー画像は4,000万画素までです')
                original_size = image.size
                image.thumbnail((1800,1800))
                if image.mode in ('I', 'I;16', 'I;16B', 'F'):
                    import numpy as np
                    a = np.asarray(image, dtype=float)
                    lo, hi = float(a.min()), float(a.max())
                    image = Image.fromarray(np.clip((a-lo)*255/max(hi-lo,1),0,255).astype('uint8'))
                image = ImageOps.exif_transpose(image).convert('RGB')
                result = io.BytesIO()
                image.save(result, format='PNG')
        result.seek(0)
        return result, original_size

    def favorites(self):
        with self.lock:
            try:
                rows = json.loads(self.favorites_file.read_text())
                return [r for r in rows if isinstance(r, dict) and isinstance(r.get('path'), str) and isinstance(r.get('label'), str)][:100]
            except (OSError, ValueError):
                return []

    def favorite(self, value, remove=False):
        if remove:
            if not isinstance(value, str):
                raise ValueError('パスが不正です')
            path = value
        else:
            path = self.resolve(value)
            if not path.is_dir():
                raise ValueError('お気に入りにはフォルダを指定してください')
        with self.lock:
            rows = [row for row in self.favorites() if row['path'] != str(path)]
            if not remove:
                if len(rows) >= 100:
                    raise ValueError('お気に入りは100件までです')
                rows.append(dict(path=str(path), label=path.name or str(path)))
            self.write_favorites(rows)
            return rows


    @staticmethod
    def filename(value):
        if (not isinstance(value, str) or value in ('', '.', '..') or
                any(c in value for c in ('/', '\\', '\0')) or
                any(ord(c)<32 for c in value) or len(value.encode('utf-8'))>255):
            raise ValueError('名前は255バイト以内で、/ や制御文字を含めずに指定してください')
        return value

    def source(self, value):
        # Keep the last component lexical: manipulating a symlink affects the
        # link itself, never the file or folder it points to.
        if not isinstance(value, str) or not value:
            raise ValueError('対象を指定してください')
        raw=Path(value).expanduser()
        if not raw.is_absolute():
            raw=self.home/raw
        path=self.resolve(str(raw.parent))/self.filename(raw.name)
        if path==self.home or path.is_relative_to(self.trash) or self.trash.is_relative_to(path):
            raise PermissionError('ホームやゴミ箱の管理フォルダは操作できません')
        mode=path.lstat().st_mode
        if not (stat.S_ISREG(mode) or stat.S_ISDIR(mode) or stat.S_ISLNK(mode)):
            raise ValueError('通常のファイル・フォルダを選択してください')
        return path

    @staticmethod
    def rename_exclusive(source, target):
        if source==target:
            return
        libc=ctypes.CDLL(None, use_errno=True)
        rename=libc.renameat2
        rename.argtypes=[ctypes.c_int,ctypes.c_char_p,ctypes.c_int,ctypes.c_char_p,ctypes.c_uint]
        rename.restype=ctypes.c_int
        if rename(-100,os.fsencode(source),-100,os.fsencode(target),1):
            code=ctypes.get_errno()
            if code in (errno.EEXIST,errno.ENOTEMPTY):
                raise FileExistsError('同名のファイル・フォルダがあるため、上書きせず中止しました')
            if code==errno.EXDEV:
                raise ValueError('別ディスク間の移動はこの画面では実行できません。元データは保持されています')
            raise OSError(code,os.strerror(code))

    def remap_favorites(self, old, new):
        rows=self.favorites()
        changed=False
        for row in rows:
            path=Path(row['path'])
            if path==old or path.is_relative_to(old):
                row['path']=str(new/path.relative_to(old))
                if path==old:
                    row['label']=new.name
                changed=True
        if changed:
            self.write_favorites(list({row['path']:row for row in rows}.values()))

    def write_favorites(self, rows):
        self.favorites_file.parent.mkdir(parents=True,exist_ok=True)
        temporary=None
        try:
            with tempfile.NamedTemporaryFile('w',dir=self.favorites_file.parent,delete=False) as stream:
                temporary=stream.name
                json.dump(rows,stream,ensure_ascii=False,indent=2)
            os.replace(temporary,self.favorites_file)
        finally:
            if temporary and os.path.exists(temporary):
                os.unlink(temporary)

    def mutate(self, body):
        action=body.get('action')
        with self.lock:
            if action=='move_many':
                return self.move_many(body.get('paths'),body.get('destination'))
            if action=='mkdir':
                parent=self.resolve(body.get('parent'))
                if not parent.is_dir():
                    raise ValueError('作成先フォルダを指定してください')
                target=parent/self.filename(body.get('name'))
                if target.is_relative_to(self.trash):
                    raise PermissionError('ゴミ箱の管理フォルダ内には作成できません')
                try:
                    target.mkdir()  # exclusive: never merge or overwrite an existing item
                except FileExistsError:
                    raise FileExistsError('同名のファイル・フォルダがあります') from None
                return dict(path=str(target),parent=str(parent))
            if action not in ('rename','move','trash'):
                raise ValueError('操作が不正です')
            source=self.source(body.get('path'))
            if action=='trash':
                if not self.trash.resolve().is_relative_to(self.home):
                    raise PermissionError('ゴミ箱の保存先がホームの外にあります')
                token=uuid.uuid4().hex
                (self.trash/'files').mkdir(parents=True,exist_ok=True)
                (self.trash/'info').mkdir(exist_ok=True)
                info=self.trash/'info'/(token+'.trashinfo')
                text='[Trash Info]\nPath='+quote(str(source),safe='/')+'\nDeletionDate='+datetime.now().isoformat(timespec='seconds')+'\nX-SiriusLauncher=true\n'
                with info.open('x') as stream:
                    stream.write(text)
                try:
                    self.rename_exclusive(source,self.trash/'files'/token)
                except Exception:
                    info.unlink(missing_ok=True)
                    raise
                return dict(path=str(source),parent=str(source.parent),trash_id=token)
            if action=='rename':
                target=source.parent/self.filename(body.get('name'))
            else:
                destination=self.resolve(body.get('destination'))
                if not destination.is_dir():
                    raise ValueError('移動先フォルダを指定してください')
                target=destination/source.name
            if target.is_relative_to(self.trash):
                raise PermissionError('削除はゴミ箱へ移動の操作を使ってください')
            if not source.is_symlink() and source.is_dir() and target.is_relative_to(source) and target!=source:
                raise ValueError('フォルダ自身や、その中には移動できません')
            self.rename_exclusive(source,target)
            warning=''
            try:
                self.remap_favorites(source,target)
            except OSError:
                warning='操作は完了しましたが、お気に入りの更新に失敗しました'
            return dict(path=str(target),parent=str(target.parent),warning=warning)

    def move_many(self, values, destination):
        if not isinstance(values,list) or not 1<=len(values)<=5000:
            raise ValueError('移動する項目を1〜5,000件選択してください')
        destination=self.resolve(destination)
        if not destination.is_dir() or destination.is_relative_to(self.trash):
            raise ValueError('移動先フォルダを指定してください')
        sources=[self.source(value) for value in values]
        if len(set(sources))!=len(sources):
            raise ValueError('同じ項目が重複しています')
        targets=[destination/source.name for source in sources]
        if len(set(targets))!=len(targets):
            raise FileExistsError('選択項目に同名のものが含まれています')
        # Validate every item before moving anything; never overwrite/merge.
        for source,target in zip(sources,targets):
            if source==target:
                raise ValueError('現在と異なる移動先フォルダを選択してください')
            if os.path.lexists(target):
                raise FileExistsError('移動先に同名の項目があります: '+source.name)
            if source.lstat().st_dev!=destination.stat().st_dev:
                raise ValueError('別ディスク間の移動はこの画面では実行できません')
            if not source.is_symlink() and source.is_dir():
                if destination.is_relative_to(source):
                    raise ValueError('選択したフォルダ自身や、その中には移動できません')
                if any(other!=source and other.is_relative_to(source) for other in sources):
                    raise ValueError('親と子のフォルダを同時に選択しないでください')
        moved=[]
        warning=''
        for index,(source,target) in enumerate(zip(sources,targets)):
            try:
                self.rename_exclusive(source,target)
            except (OSError,ValueError) as error:
                return dict(moved=moved,remaining=[str(p) for p in sources[index:]],
                            partial=True,error_message=str(error),warning=warning)
            moved.append(dict(source=str(source),path=str(target)))
            try:
                self.remap_favorites(source,target)
            except OSError:
                warning='移動は完了しましたが、お気に入りの更新に失敗しました'
        return dict(moved=moved,remaining=[],partial=False,warning=warning)

    def trash_items(self):
        entries=[]
        for info in sorted((self.trash/'info').glob('*.trashinfo'),key=lambda p:p.stat().st_mtime,reverse=True):
            if not re.fullmatch(r'[0-9a-f]{32}',info.stem):
                continue
            try:
                parser=configparser.RawConfigParser()
                parser.read(info,encoding='utf-8')
                if parser.get('Trash Info','X-SiriusLauncher',fallback='')!='true':
                    continue
                original=unquote(parser.get('Trash Info','Path'))
                stored=self.trash/'files'/info.stem
                if not os.path.lexists(stored):
                    continue
                entries.append(dict(id=info.stem,name=Path(original).name,original=original,
                                    deleted=parser.get('Trash Info','DeletionDate',fallback='')))
                if len(entries)>=1000:
                    break
            except (OSError,configparser.Error,ValueError):
                continue
        return entries

    def restore(self, token):
        if not isinstance(token,str) or not re.fullmatch(r'[0-9a-f]{32}',token):
            raise ValueError('ゴミ箱のIDが不正です')
        with self.lock:
            item=next((row for row in self.trash_items() if row['id']==token),None)
            if item is None:
                raise FileNotFoundError('ゴミ箱の項目が見つかりません')
            raw=Path(item['original'])
            parent=self.resolve(str(raw.parent))
            target=parent/self.filename(raw.name)
            self.rename_exclusive(self.trash/'files'/token,target)
            (self.trash/'info'/(token+'.trashinfo')).unlink()
            return dict(path=str(target),parent=str(parent))


def register_file_browser(app, auth_ok, files=None):
    files = files or HostFiles()
    bp = Blueprint('host_files', __name__)

    @bp.before_request
    def authenticate():
        if not auth_ok():
            return jsonify(error='auth'), 401

    @bp.after_request
    def no_cache(response):
        response.headers['Cache-Control'] = 'no-store'
        response.headers['X-Content-Type-Options'] = 'nosniff'
        return response

    @bp.errorhandler(OSError)
    @bp.errorhandler(ValueError)
    def invalid(error):
        return jsonify(error=str(error)), 403 if isinstance(error, PermissionError) else 409 if isinstance(error, FileExistsError) else 404 if isinstance(error, FileNotFoundError) else 400

    @bp.get('/list')
    def listing():
        return jsonify(files.listing(request.args.get('path',''), request.args.get('hidden')=='1'))

    @bp.get('/text')
    def text():
        return jsonify(files.text(request.args.get('path','')))

    @bp.post('/text')
    def save_text():
        if request.content_length and request.content_length>TEXT_LIMIT*6+4096:
            return jsonify(error='ファイルサイズの上限は512 KBです'), 413
        return jsonify(files.save_text(request.get_json(silent=True)))

    @bp.get('/preview')
    def preview():
        try:
            data, size = files.image(request.args.get('path',''))
        except Exception as error:
            # Image decoders can raise format-specific errors beyond OSError.
            return jsonify(error=str(error)), 400
        response = send_file(data, mimetype='image/png', max_age=0)
        response.headers['X-Image-Width'], response.headers['X-Image-Height'] = map(str,size)
        return response

    @bp.get('/download')
    def download():
        return send_file(files.regular_file(request.args.get('path','')), as_attachment=True, conditional=True)

    @bp.route('/favorites', methods=['GET', 'POST'])
    def favorites():
        if request.method == 'GET':
            return jsonify(favorites=files.favorites())
        body = request.get_json(silent=True)
        if not isinstance(body, dict) or body.get('action') not in ('add','remove'):
            return jsonify(error='お気に入りの操作が不正です'), 400
        return jsonify(favorites=files.favorite(body.get('path'), body['action']=='remove'))

    @bp.post('/operate')
    def operate():
        body=request.get_json(silent=True)
        if not isinstance(body,dict):
            return jsonify(error='操作が不正です'),400
        return jsonify(files.mutate(body))

    @bp.get('/trash')
    def trash():
        return jsonify(entries=files.trash_items())

    @bp.post('/restore')
    def restore():
        body=request.get_json(silent=True)
        if not isinstance(body,dict):
            return jsonify(error='操作が不正です'),400
        return jsonify(files.restore(body.get('id')))

    app.register_blueprint(bp, url_prefix='/api/files')
    return files
