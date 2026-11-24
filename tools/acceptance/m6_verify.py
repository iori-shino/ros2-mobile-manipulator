#!/usr/bin/env python3
"""Score final physical positions using independent Ignition Pose_V recordings."""

import argparse
import json
import math
from pathlib import Path
import re


def positions(log):
    for match in re.finditer(r'stamp\s*\{([^}]+)\}.*?position\s*\{([^}]+)\}', log, re.S):
        clock, xyz = (dict(re.findall(r'(\w+):\s*([-\d.eE+]+)', block))
                      for block in match.groups())
        time_s = int(clock.get('sec', '0'))+int(clock.get('nsec', '0'))*1e-9
        yield time_s, tuple(float(xyz.get(axis, '0')) for axis in 'xyz')


def verify(run):
    manifest = json.loads((run/'manifest.json').read_text())
    task = json.loads((run/'result.json').read_text())
    colors = list(task['objects'])
    report = {'task_passed': task['passed'], 'objects': {}}
    for color in colors:
        samples = list(positions((run/(color+'_truth.log')).read_text()))
        if not samples:
            report['objects'][color] = {'passed': False, 'error': 'Missing pose recording'}
            continue
        end_s, end = samples[-1]
        tail = [p for t,p in samples if t >= end_s-.6]
        x, y = manifest['drop_zones'][color]['table_world_xy']
        height = manifest['table_top_m']+manifest['cubes'][color]['size_m']/2
        dx, dy, dz = abs(end[0]-x), abs(end[1]-y), abs(end[2]-height)
        drift = max(math.dist(p, end) for p in tail)
        lifted = max(p[2] for _,p in samples)-height
        report['objects'][color] = {'passed': dx < .040 and dy < .030 and dz < .002
                                     and len(tail) >= 3 and drift < .001 and lifted > .035,
                                     'final_world_m': end, 'lift_m': lifted, 'settled_drift_m': drift}
    report['passed'] = bool(colors) and task['passed'] and all(
        item['passed'] for item in report['objects'].values())
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    result = verify(args.run)
    print(json.dumps(result, indent=2))
    raise SystemExit(not result['passed'])
