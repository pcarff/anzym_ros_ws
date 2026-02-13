#!/usr/bin/env python3
"""
URDF Viewer Server
Serves the URDF file and STL meshes for the web-based 3D viewer.
Resolves package:// paths to local filesystem paths.

Usage:
    python3 serve.py [--port 8080]
"""

import http.server
import json
import os
import re
import subprocess
import sys
import argparse
from pathlib import Path
from urllib.parse import unquote

WORKSPACE = Path(__file__).resolve().parent.parent.parent  # anzym_ros_ws

# Map package names to directories
PACKAGE_PATHS = {
    'anzym_description': WORKSPACE / 'src' / 'anzym_description',
}


def resolve_package_url(url):
    """Resolve package://pkg_name/path to filesystem path."""
    match = re.match(r'package://([^/]+)/(.*)', url)
    if match:
        pkg_name, rel_path = match.groups()
        if pkg_name in PACKAGE_PATHS:
            return PACKAGE_PATHS[pkg_name] / rel_path
    return None


def get_urdf_content():
    """Run xacro to get the processed URDF, then rewrite mesh paths."""
    xacro_file = WORKSPACE / 'src' / 'anzym_description' / 'urdf' / 'yahboomcar_X3plus.urdf.xacro'

    # Try using xacro
    try:
        result = subprocess.run(
            ['bash', '-c', f'source /opt/ros/humble/setup.bash && xacro {xacro_file}'],
            capture_output=True, text=True, timeout=10
        )
        if result.returncode == 0:
            urdf = result.stdout
        else:
            # Fall back to pre-generated
            with open('/tmp/anzym_robot.urdf', 'r') as f:
                urdf = f.read()
    except Exception:
        with open('/tmp/anzym_robot.urdf', 'r') as f:
            urdf = f.read()

    # Rewrite package:// URLs to relative /mesh/ URLs
    urdf = re.sub(
        r'filename="package://([^"]+)"',
        r'filename="/mesh/\1"',
        urdf
    )
    return urdf


class URDFHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        self.viewer_dir = Path(__file__).resolve().parent
        super().__init__(*args, directory=str(self.viewer_dir), **kwargs)

    def do_GET(self):
        path = unquote(self.path)

        if path == '/urdf':
            content = get_urdf_content()
            self.send_response(200)
            self.send_header('Content-Type', 'application/xml')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(content.encode())

        elif path.startswith('/mesh/'):
            # /mesh/anzym_description/meshes/X3plus/visual/base_link.STL
            rel = path[len('/mesh/'):]
            parts = rel.split('/', 1)
            if len(parts) == 2:
                pkg_name, file_path = parts
                if pkg_name in PACKAGE_PATHS:
                    full_path = PACKAGE_PATHS[pkg_name] / file_path
                    if full_path.exists():
                        self.send_response(200)
                        self.send_header('Content-Type', 'application/octet-stream')
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        with open(full_path, 'rb') as f:
                            self.wfile.write(f.read())
                        return
            self.send_error(404, f'Mesh not found: {path}')

        else:
            super().do_GET()

    def log_message(self, format, *args):
        msg = format % args
        if '.STL' not in msg and 'favicon' not in msg:
            print(f"  {msg}")


def main():
    parser = argparse.ArgumentParser(description='URDF Viewer Server')
    parser.add_argument('--port', type=int, default=8080)
    args = parser.parse_args()

    print(f"\n🤖 URDF Viewer Server")
    print(f"   Open in browser: http://localhost:{args.port}")
    print(f"   Workspace: {WORKSPACE}")
    print(f"   Press Ctrl+C to stop\n")

    server = http.server.HTTPServer(('0.0.0.0', args.port), URDFHandler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nServer stopped.")


if __name__ == '__main__':
    main()
