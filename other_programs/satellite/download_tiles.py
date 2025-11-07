#!/usr/bin/env python3
"""
Download map tiles for offline use
Requires: pip install requests tqdm
"""

import os
import requests
from pathlib import Path
from tqdm import tqdm
import time

def lat_lon_to_tile(lat, lon, zoom):
    """Convert latitude/longitude to tile coordinates"""
    import math
    lat_rad = math.radians(lat)
    n = 2.0 ** zoom
    x = int((lon + 180.0) / 360.0 * n)
    y = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return x, y

def download_tiles(lat_min, lat_max, lon_min, lon_max, zoom_levels, 
                   output_dir='./tiles', tile_url_template=None):
    """
    Download map tiles for a specific area
    
    Args:
        lat_min, lat_max: Latitude range
        lon_min, lon_max: Longitude range
        zoom_levels: List of zoom levels to download
        output_dir: Directory to save tiles
        tile_url_template: URL template (default: OpenStreetMap)
    """
    if tile_url_template is None:
        # Default: OpenStreetMap
        tile_url_template = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
    
    os.makedirs(output_dir, exist_ok=True)
    
    total_tiles = 0
    for zoom in zoom_levels:
        x_min, y_max = lat_lon_to_tile(lat_min, lon_min, zoom)
        x_max, y_min = lat_lon_to_tile(lat_max, lon_max, zoom)
        
        x_min, x_max = min(x_min, x_max), max(x_min, x_max)
        y_min, y_max = min(y_min, y_max), max(y_min, y_max)
        
        tiles_count = (x_max - x_min + 1) * (y_max - y_min + 1)
        total_tiles += tiles_count
        
        print(f"Zoom {zoom}: {tiles_count} tiles ({x_max-x_min+1}x{y_max-y_min+1})")
    
    print(f"\nTotal tiles to download: {total_tiles}")
    response = input("Continue? (y/n): ")
    if response.lower() != 'y':
        return
    
    downloaded = 0
    skipped = 0
    
    with tqdm(total=total_tiles, desc="Downloading tiles") as pbar:
        for zoom in zoom_levels:
            x_min, y_max = lat_lon_to_tile(lat_min, lon_min, zoom)
            x_max, y_min = lat_lon_to_tile(lat_max, lon_max, zoom)
            
            x_min, x_max = min(x_min, x_max), max(x_min, x_max)
            y_min, y_max = min(y_min, y_max), max(y_min, y_max)
            
            for x in range(x_min, x_max + 1):
                for y in range(y_min, y_max + 1):
                    # Create directory structure
                    tile_dir = os.path.join(output_dir, str(zoom), str(x))
                    os.makedirs(tile_dir, exist_ok=True)
                    
                    tile_path = os.path.join(tile_dir, f"{y}.png")
                    
                    # Skip if already exists
                    if os.path.exists(tile_path):
                        skipped += 1
                        pbar.update(1)
                        continue
                    
                    # Download tile
                    url = tile_url_template.format(z=zoom, x=x, y=y)
                    
                    try:
                        headers = {
                            'User-Agent': 'Mozilla/5.0 (ROS2 offline tile downloader)'
                        }
                        response = requests.get(url, headers=headers, timeout=10)
                        
                        if response.status_code == 200:
                            with open(tile_path, 'wb') as f:
                                f.write(response.content)
                            downloaded += 1
                        else:
                            print(f"\nFailed to download {url}: {response.status_code}")
                        
                        # Be nice to the server
                        time.sleep(0.1)
                        
                    except Exception as e:
                        print(f"\nError downloading {url}: {e}")
                    
                    pbar.update(1)
    
    print(f"\nDone! Downloaded: {downloaded}, Skipped: {skipped}")

if __name__ == "__main__":
    # Example: Download tiles for a specific area
    # Adjust coordinates for your area of interest
    
    # 大阪工業大学周辺：34.7054644 135.5001309
    LAT_CENTER = 34.7054644
    LON_CENTER = 135.5001309
    
    # Define area (approximately 1km x 1km)
    LAT_DELTA = 0.01  # ~1000m
    LON_DELTA = 0.01  # ~1000m
    
    lat_min = LAT_CENTER - LAT_DELTA
    lat_max = LAT_CENTER + LAT_DELTA
    lon_min = LON_CENTER - LON_DELTA
    lon_max = LON_CENTER + LON_DELTA
    
    # Zoom levels (higher = more detail, more tiles)
    zoom_levels = [16, 17, 18, 20]
    
    # Choose tile source:
    # OpenStreetMap (recommended for offline use, respects ToS)
    # tile_url = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
    
    # Or Google Maps satellite (use with caution, may violate ToS)
    tile_url = "https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}"
    
    print(f"Downloading tiles for area:")
    print(f"  Latitude: {lat_min:.4f} to {lat_max:.4f}")
    print(f"  Longitude: {lon_min:.4f} to {lon_max:.4f}")
    print(f"  Zoom levels: {zoom_levels}")
    print(f"  Tile source: {tile_url}")
    print()
    
    download_tiles(lat_min, lat_max, lon_min, lon_max, zoom_levels, 
                   output_dir='./tiles', tile_url_template=tile_url)
