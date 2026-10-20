"""Run from workspace root. Panels: original and gains 2/3/4, shaded + sunlit."""
import json
import sys
from pathlib import Path

import cv2
from PIL import Image, ImageDraw

sys.path.insert(0, 'bash/startup_bash')
from theta_texture_tone import lift_texture, tone_report
from review_theta_road_map import crop_world, visible_rgb

source = Path('maps_waypoints/maps/theta_road_general_full_20261020')
out = Path(__file__).parent
image = cv2.imread(str(source/'road_temporal.texture.png'), -1)
meta = json.loads((source/'manifest.json').read_text())
rows = {row['frame']: row for row in meta['frames']}
canvas = Image.new('RGB', (1600, 900), 'white')
draw = ImageDraw.Draw(canvas)
reports = []
for column, gain in enumerate((1, 2, 3, 4)):
    config = dict(enabled=True, gain=gain, pivot=180)
    toned = lift_texture(image, config)
    reports.append(tone_report(image, toned, config))
    for row, index in enumerate((2700, 4500)):
        crop = crop_world(toned, meta['grid'], rows[index]['xyz'][:2], span_m=10, width=400)
        canvas.paste(Image.fromarray(cv2.cvtColor(visible_rgb(crop), cv2.COLOR_BGR2RGB)),
                     (400*column, 450*row+45))
        draw.text((400*column+12, 450*row+12), f'gain {gain} / frame {index}', fill='black')
canvas.save(out/'pilot.jpg', quality=94)
(out/'metrics.json').write_text(json.dumps(reports, indent=2))
