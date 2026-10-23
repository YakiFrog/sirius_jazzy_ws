"""Persistent Markdown notes shared by the launcher's browser clients."""
import json
import os
import re
import tempfile
import threading
import uuid
from datetime import datetime, timezone
from pathlib import Path

from flask import Blueprint, jsonify, request


class NoteConflict(Exception):
    pass


class NoteStore:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.lock = threading.RLock()

    def _path(self, note_id):
        if not re.fullmatch(r"[0-9a-f]{32}", note_id):
            raise ValueError("メモIDが不正です")
        return self.directory / (note_id + ".json")

    def get(self, note_id):
        with self.lock:
            return self._read(self._path(note_id))

    @staticmethod
    def _read(path):
        try:
            note = json.loads(path.read_text(encoding="utf-8"))
        except (json.JSONDecodeError, UnicodeError) as error:
            raise OSError("Stored note is not valid JSON") from error
        if (not isinstance(note, dict) or type(note.get("revision")) is not int
                or not all(isinstance(note.get(key), str)
                           for key in ["id", "title", "body", "created_at", "updated_at"])):
            raise OSError("Stored note has an invalid schema")
        return note

    def list(self, query=""):
        with self.lock:
            notes = [self._read(p)
                     for p in self.directory.glob("*.json")]
            terms = query.lower().split()
            notes = [n for n in notes if all(t in (n["title"] + " " + n["body"]).lower() for t in terms)]
            return [{**{k: v for k, v in n.items() if k != "body"},
                     "excerpt": n["body"][:160]}
                    for n in sorted(notes, key=lambda n: n["updated_at"], reverse=True)]

    def save(self, payload, note_id=None):
        title, body = payload.get("title"), payload.get("body")
        if not isinstance(title, str) or not title.strip() or len(title) > 200:
            raise ValueError("タイトルを1〜200文字で入力してください")
        if not isinstance(body, str) or len(body) > 100_000:
            raise ValueError("本文は10万文字以内で入力してください")
        with self.lock:
            now = datetime.now(timezone.utc).isoformat()
            previous = self.get(note_id) if note_id else None
            if previous and (type(payload.get("revision")) is not int
                             or payload["revision"] != previous["revision"]):
                raise NoteConflict()
            note = {"id": note_id or uuid.uuid4().hex, "title": title.strip(),
                    "body": body, "format": "markdown", "revision": previous["revision"] + 1 if previous else 1,
                    "created_at": previous["created_at"] if previous else now, "updated_at": now}
            self.directory.mkdir(parents=True, exist_ok=True, mode=0o700)
            temp = None
            try:
                with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=self.directory,
                                                 prefix=".note-", suffix=".tmp", delete=False) as stream:
                    temp = Path(stream.name)
                    json.dump(note, stream, ensure_ascii=False, indent=2)
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(temp, self._path(note["id"]))
            finally:
                if temp is not None:
                    temp.unlink(missing_ok=True)
            return note

    def delete(self, note_id, revision):
        with self.lock:
            previous = self.get(note_id)
            if type(revision) is not int or revision != previous["revision"]:
                raise NoteConflict()
            trash = self.directory / ".trash"
            trash.mkdir(exist_ok=True, mode=0o700)
            self._path(note_id).replace(trash / (note_id + "-" + uuid.uuid4().hex + ".json"))


def register_documentation_notes(app, auth_ok, directory=None):
    data_root = Path(os.environ.get("XDG_DATA_HOME") or Path.home() / ".local/share")
    store = NoteStore(directory or data_root / "sirius_launcher/documents")
    app.extensions["documentation_notes"] = store
    api = Blueprint("documentation_notes", __name__)

    @api.before_request
    def authorize():
        if not auth_ok():
            return jsonify(error="auth"), 401

    def payload():
        raw = request.stream.read(500_001)
        if len(raw) > 500_000:
            raise ValueError("メモのデータが大きすぎます")
        try:
            data = json.loads(raw)
        except (ValueError, UnicodeError):
            raise ValueError("JSON形式のデータを指定してください") from None
        if not isinstance(data, dict):
            raise ValueError("JSONオブジェクトを指定してください")
        return data

    @api.errorhandler(ValueError)
    def invalid(error):
        return jsonify(error=str(error)), 400

    @api.errorhandler(FileNotFoundError)
    def missing(error):
        return jsonify(error="メモが見つかりません。更新して確認してください"), 404

    @api.errorhandler(NoteConflict)
    def conflict(error):
        return jsonify(error="別の画面で更新されています。入力は保持しています。内容を確認するか、複製して保存してください"), 409

    @api.errorhandler(OSError)
    def storage_error(error):
        app.logger.exception("Documentation note storage failed")
        return jsonify(error="メモを読み書きできません。サーバーの保存先・空き容量を確認してください"), 500

    @api.route("/api/documentation/notes", methods=["GET", "POST"])
    def collection():
        if request.method == "GET":
            return jsonify(notes=store.list(request.args.get("q", "")[:200]))
        return jsonify(note=store.save(payload())), 201

    @api.route("/api/documentation/notes/<note_id>", methods=["GET", "PUT", "DELETE"])
    def item(note_id):
        if request.method == "GET":
            return jsonify(note=store.get(note_id))
        data = payload()
        if request.method == "DELETE":
            store.delete(note_id, data.get("revision"))
            return jsonify(ok=True)
        return jsonify(note=store.save(data, note_id))

    app.register_blueprint(api)
    return store
