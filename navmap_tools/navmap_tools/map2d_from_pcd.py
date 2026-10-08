# Copyright 2026 Intelligent Robotics Lab
#
# This file is part of the project Easy Navigation (EasyNav in short)
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Build a 2D occupancy grid (for a flat NavMap) from a 3D point cloud (.pcd, ascii).

Points in the robot's height band are obstacles. Free space is flood-filled from a seed inside
the walls, never past the walls' extent; the rest is unknown. Writes <output>.pgm and
<output>.yaml (map_server format), e.g.:
  ros2 run navmap_tools navmap_map2d_from_pcd maps/warehouse.pcd maps/warehouse
"""

import argparse
import math
import os

import numpy as np

from .pcd_writer import read_pcd_xyz

FREE, OCCUPIED, UNKNOWN = 254, 0, 205   # map_server trinary PGM values


def grid_from_cloud(cloud, seed, resolution, z_min, z_max):
    """
    Occupancy grid (uint8 rows bottom-up, FREE/OCCUPIED/UNKNOWN) and its origin (x0, y0).

    Raises ValueError when no point is inside the height band or the seed is not free.
    """
    obst = cloud[(cloud[:, 2] > z_min) & (cloud[:, 2] < z_max)]
    if not len(obst):
        raise ValueError(f'no points between z {z_min} and {z_max} m')

    x0, y0 = math.floor(obst[:, 0].min()) - 1, math.floor(obst[:, 1].min()) - 1
    w = int(math.ceil((obst[:, 0].max() + 1 - x0) / resolution))
    h = int(math.ceil((obst[:, 1].max() + 1 - y0) / resolution))
    blocked = np.zeros((h, w), bool)
    rows = ((obst[:, 1] - y0) / resolution).astype(int)
    cols = ((obst[:, 0] - x0) / resolution).astype(int)
    blocked[rows, cols] = True
    grown = blocked.copy()  # close one-cell gaps between obstacle points
    for d in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        grown |= np.roll(blocked, d, axis=(0, 1))
    rmin, rmax, cmin, cmax = rows.min(), rows.max(), cols.min(), cols.max()  # walls' extent

    seed_rc = (int((seed[1] - y0) / resolution), int((seed[0] - x0) / resolution))
    if not (0 <= seed_rc[0] < h and 0 <= seed_rc[1] < w) or grown[seed_rc]:
        raise ValueError(f'seed {tuple(seed)} is not in free space')
    free = np.zeros((h, w), bool)
    stack = [seed_rc]
    while stack:
        r, c = stack.pop()
        if r < rmin or c < cmin or r > rmax or c > cmax or free[r, c] or grown[r, c]:
            continue
        free[r, c] = True
        stack.extend(((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)))

    grid = np.full((h, w), UNKNOWN, np.uint8)
    grid[free] = FREE
    grid[blocked] = OCCUPIED
    return grid, (x0, y0)


def write_map_server(prefix, grid, resolution, origin):
    """Write <prefix>.pgm and <prefix>.yaml (map_server format, trinary)."""
    h, w = grid.shape
    with open(prefix + '.pgm', 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        f.write(np.flipud(grid).tobytes())  # PGM rows go top-down
    name = os.path.basename(prefix)
    with open(prefix + '.yaml', 'w') as f:
        f.write(f'image: {name}.pgm\nmode: trinary\nresolution: {resolution}\n'
                f'origin: [{origin[0]}, {origin[1]}, 0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')


def build_arg_parser():
    """Argument parser of navmap_map2d_from_pcd."""
    parser = argparse.ArgumentParser(
        prog='navmap_map2d_from_pcd', description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('pcd', help='input cloud (.pcd, ascii, x y z)')
    parser.add_argument('output', help='output prefix: writes <output>.pgm and <output>.yaml')
    parser.add_argument('--seed', nargs=2, type=float, default=(0.0, 0.0), metavar=('X', 'Y'),
                        help='a free point inside the walls')
    parser.add_argument('--resolution', type=float, default=0.05, help='cell size (m)')
    parser.add_argument('--z-min', type=float, default=0.1, help='obstacle band bottom (m)')
    parser.add_argument('--z-max', type=float, default=1.2, help='obstacle band top (m)')
    return parser


def main(argv=None):
    """Console entry point."""
    args = build_arg_parser().parse_args(argv)
    grid, origin = grid_from_cloud(read_pcd_xyz(args.pcd), args.seed, args.resolution,
                                   args.z_min, args.z_max)
    write_map_server(args.output, grid, args.resolution, origin)
    h, w = grid.shape
    print(f'{w}x{h} at {args.resolution} m, origin {origin}: free {(grid == FREE).sum()}, '
          f'occupied {(grid == OCCUPIED).sum()}')
