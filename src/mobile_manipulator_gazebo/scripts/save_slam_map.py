#!/usr/bin/env python3
"""Save via SLAM Toolbox's service, then reopen the YAML and image for validation."""
import argparse
from pathlib import Path
import time
import yaml
from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from slam_toolbox.srv import SaveMap


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prefix', help='New map path without extension; existing maps are never overwritten')
    args = parser.parse_args()
    prefix = Path(args.prefix).expanduser().resolve()
    prefix.parent.mkdir(parents=True, exist_ok=True)
    if any(prefix.with_suffix(ext).exists() for ext in ['.yaml', '.pgm', '.png']):
        parser.error('Map already exists; choose a different prefix.')
    rclpy.init()
    node = Node('save_slam_map', parameter_overrides=[Parameter('use_sim_time', value=True)])
    try:
        client = node.create_client(SaveMap, '/slam_toolbox/save_map')
        if not client.wait_for_service(timeout_sec=20):
            raise RuntimeError('SLAM map saver service is not available')
        req = SaveMap.Request(); req.name.data = str(prefix)
        future = client.call_async(req)
        deadline = time.monotonic() + 60
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if not future.done() or future.result() is None or future.result().result != 0:
            raise RuntimeError(f'Map save failed: {future.result() if future.done() else "timeout"}')
        metadata = yaml.safe_load(prefix.with_suffix('.yaml').read_text())
        image_path = prefix.parent / metadata['image']
        with Image.open(image_path) as image:
            image.load()
            assert image.width > 10 and image.height > 10 and image.getextrema()[0] != image.getextrema()[1]
            assert metadata['resolution'] > 0 and len(metadata['origin']) == 3
            print(f'Saved and reopened {prefix.with_suffix(".yaml")} + {image_path.name}; {image.size}, resolution={metadata["resolution"]} m', flush=True)
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
