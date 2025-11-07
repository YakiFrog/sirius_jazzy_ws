#!/usr/bin/env python3
"""
Simple local tile server for offline map tiles
Usage: python3 tile_server.py [port] [tiles_directory]
"""

import http.server
import socketserver
import os
import sys
from pathlib import Path

class TileHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, tiles_dir=None, **kwargs):
        self.tiles_dir = tiles_dir
        super().__init__(*args, **kwargs)
    
    def do_GET(self):
        # Parse the path: /{z}/{x}/{y}.png
        path_parts = self.path.strip('/').split('/')
        
        if len(path_parts) == 3:
            z, x, y_with_ext = path_parts
            # Remove .png extension
            y = y_with_ext.replace('.png', '')
            
            # Construct file path
            tile_path = os.path.join(self.tiles_dir, z, x, f"{y}.png")
            
            if os.path.exists(tile_path):
                self.send_response(200)
                self.send_header('Content-type', 'image/png')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                
                with open(tile_path, 'rb') as f:
                    self.wfile.write(f.read())
                return
        
        # If tile not found, return 404
        self.send_error(404, "Tile not found")

def run_server(port=8000, tiles_dir='./tiles'):
    tiles_dir = os.path.abspath(tiles_dir)
    
    if not os.path.exists(tiles_dir):
        print(f"Error: Tiles directory '{tiles_dir}' does not exist")
        print(f"Creating directory...")
        os.makedirs(tiles_dir, exist_ok=True)
    
    print(f"Serving tiles from: {tiles_dir}")
    print(f"Server running on port {port}")
    print(f"Tile URL format: http://localhost:{port}/{{z}}/{{x}}/{{y}}.png")
    print("Press Ctrl+C to stop")
    
    handler = lambda *args, **kwargs: TileHandler(*args, tiles_dir=tiles_dir, **kwargs)
    
    with socketserver.TCPServer(("", port), handler) as httpd:
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nServer stopped")

if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
    tiles_dir = sys.argv[2] if len(sys.argv) > 2 else './tiles'
    run_server(port, tiles_dir)
