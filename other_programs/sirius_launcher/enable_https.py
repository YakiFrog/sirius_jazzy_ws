#!/usr/bin/env python3
"""Enable private Tailscale HTTPS entry points for Launcher, viewer and ROS bridge."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROUTES = {443: 'http://127.0.0.1:8090',
          8443: 'http://127.0.0.1:8091',
          8767: 'http://127.0.0.1:8766'}


def read_json(*args):
    return json.loads(subprocess.check_output(['tailscale', *args], text=True))


def validate_existing(config, domain):
    for port, target in ROUTES.items():
        site = config.get('Web', {}).get(f'{domain}:{port}', {})
        handler = site.get('Handlers', {}).get('/')
        if handler and handler.get('Proxy') != target:
            raise ValueError(f'{domain}:{port} already serves another application')
        listener = config.get('TCP', {}).get(str(port), {})
        if listener and not listener.get('HTTPS'):
            raise ValueError(f'Port {port} already has a non-HTTPS Serve listener')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Show plan without changing settings')
    args = parser.parse_args()
    status = read_json('status', '--json')
    domain = str(status.get('Self', {}).get('DNSName', '')).rstrip('.')
    if not domain or domain not in status.get('CertDomains', []):
        raise ValueError('Tailscale HTTPS certificate domain is unavailable')
    before = read_json('serve', 'status', '--json')
    validate_existing(before, domain)
    for port, target in ROUTES.items():
        suffix = '' if port == 443 else f':{port}'
        print(f'https://{domain}{suffix}/ -> {target}', flush=True)
    if args.check:
        return 0
    if os.geteuid() != 0:
        print(f'NUC上で実行してください: sudo python3 {Path(__file__).resolve()}', file=sys.stderr)
        return 1
    backup_dir = Path('/var/lib/sirius-launcher/https')
    backup_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
    backup = backup_dir / f'serve-before-{time.time_ns()}.json'
    backup.write_text(json.dumps(before, indent=2)+'\n', encoding='utf-8')
    for port, target in ROUTES.items():
        subprocess.run(['tailscale', 'serve', '--bg', f'--https={port}', '--yes', target], check=True)
    public_link = Path(__file__).with_name('static') / 'share-link.json'
    public_link.write_text(json.dumps({'public_url': f'https://{domain}/'})+'\n', encoding='utf-8')
    public_link.chmod(0o644)
    print(f'完了: https://{domain}/ をChromeで開いてアプリを入れ直してください。')
    print(f'以前のServe設定: {backup}')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (subprocess.CalledProcessError, ValueError, OSError) as error:
        print(f'HTTPS設定に失敗しました: {error}', file=sys.stderr)
        sys.exit(1)
