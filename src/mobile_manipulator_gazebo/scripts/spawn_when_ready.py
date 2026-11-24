#!/usr/bin/env python3
"""Wait for a running Fortress world and require a successful create response."""
import argparse
import json
from pathlib import Path
import re
import subprocess
import sys


def spawn(world, model, name='mobile_manipulator', timeout=120):
    # Service advertisement precedes the first simulation update on a cold VM.
    # Wait for actual world statistics, not a fixed sleep or just service discovery.
    ready = subprocess.run(
        ['ign', 'topic', '-e', '-t', f'/world/{world}/stats', '-n', '1'],
        capture_output=True, text=True, timeout=timeout, check=True)
    if not re.search(r'\biterations\s*:', ready.stdout):
        raise RuntimeError('World did not publish simulation statistics')
    request = (f'sdf_filename: {json.dumps(str(Path(model).resolve()))}, '
               f'name: {json.dumps(name)}, allow_renaming: false, '
               'pose: {position: {z: 0.04}}')
    result = subprocess.run(
        ['ign', 'service', '-s', f'/world/{world}/create',
         '--reqtype', 'ignition.msgs.EntityFactory',
         '--reptype', 'ignition.msgs.Boolean', '--timeout', str(timeout * 1000),
         '--req', request], capture_output=True, text=True,
        timeout=timeout + 5, check=True)
    # Some ros_gz_sim create versions exit zero even after request timeout.
    # Do not start controllers or GUI unless the server actually accepted creation.
    if not re.search(r'\bdata\s*:\s*true\b', result.stdout):
        raise RuntimeError('Gazebo did not confirm model creation: ' +
                           (result.stdout + result.stderr).strip())
    print(f'Created {name} in {world}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', default='flat_world')
    parser.add_argument('--model', required=True)
    args = parser.parse_args()
    try:
        spawn(args.world, args.model)
    except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f'Robot creation failed: {exc}', file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
