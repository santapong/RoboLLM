"""Encode actual rendered simulation frames; requires ffmpeg on PATH."""
from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory', type=Path)
    args = parser.parse_args()
    directory = args.run_directory.resolve()
    report = json.loads((directory / 'report.json').read_text())
    frames = sorted((directory / 'frames').glob('frame-*.png'))
    if not frames or len(frames) != report['rendered_frames']:
        parser.error('Frame count does not match this run report')
    output = directory / 'simulation.mp4'
    # Refuse replacement so a new report cannot silently reuse an old video.
    subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-n',
        '-framerate', str(report['frame_rate']), '-i', str(directory / 'frames/frame-%05d.png'),
        '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '22', '-movflags', '+faststart',
        str(output)], check=True)
    print(output)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
