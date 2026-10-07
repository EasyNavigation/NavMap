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
Resample a flat, grid-based NavMap to a coarser resolution.

The input is a .navmap file built from an occupancy grid (one flat surface, a regular grid of
vertices, two triangles per cell, as navmap_ros::from_occupancy_grid builds it) or a map_server
.yaml occupancy grid. The new resolution must be an integer multiple of the input's: each new
cell joins k x k input cells. The output is a .navmap file with the same structure, whose
"occupancy" layer (u8: 0 free, 1-253 cost, 254 lethal, 255 unknown) is, per block of cells:
  - lethal (254) if any cell is lethal: no obstacle is lost;
  - otherwise, the highest known cost if at least half of the cells are known;
  - otherwise unknown (255).
Other u8 layers take the block's maximum; float layers its mean over finite values.

For instance, a 5 cm map to 20 cm:
  ros2 run navmap_tools navmap_resample warehouse.yaml warehouse_20cm.navmap 0.2
"""

import argparse
import math
import os
import struct
import sys

from navmap_ros_interfaces.msg import NavMap, NavMapLayer, NavMapSurface
import numpy as np
from rclpy.serialization import deserialize_message, serialize_message

MAGIC = b'NAVMAP\x00\x00'
VERSION = 1
FREE, LETHAL, UNKNOWN = 0, 254, 255


# ------------------------------- Reading ---------------------------------

def read_navmap(path):
    with open(path, 'rb') as f:
        data = f.read()
    if data[:8] == MAGIC:
        _, size = struct.unpack('<IQ', data[8:20])
        data = data[20:20 + size]
    return deserialize_message(data, NavMap)


def write_navmap(msg, path):
    payload = serialize_message(msg)
    with open(path, 'wb') as f:
        f.write(MAGIC)
        f.write(struct.pack('<IQ', VERSION, len(payload)))
        f.write(payload)


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    tokens, pos = [], 0
    while len(tokens) < 4:  # magic, width, height, maxval (with comments)
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[pos:end])
        pos = end
    if tokens[0] != b'P5':
        sys.exit(f'{path}: only binary PGM (P5) is supported')
    w, h, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    pixels = np.frombuffer(data[pos + 1:pos + 1 + w * h], np.uint8).reshape(h, w)
    return np.flipud(pixels).astype(float) / maxval  # row 0 at the origin (bottom)


def read_yaml_grid(path):
    """Occupancy (u8 per cell, row 0 at the origin), origin x y z and resolution."""
    cfg = {}
    with open(path) as f:
        for line in f:
            if ':' in line and not line.lstrip().startswith('#'):
                key, value = line.split(':', 1)
                cfg[key.strip()] = value.strip()
    image = cfg['image']
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(path)), image)
    res = float(cfg['resolution'])
    origin = [float(v) for v in cfg['origin'].strip('[]').split(',')]
    negate = int(cfg.get('negate', 0))
    occ_th = float(cfg.get('occupied_thresh', 0.65))
    free_th = float(cfg.get('free_thresh', 0.196))
    mode = cfg.get('mode', 'trinary')
    if mode not in ('trinary', 'scale'):
        sys.exit(f'{path}: mode {mode} not supported (trinary or scale)')
    shade = read_pgm(image)
    p = shade if negate else 1.0 - shade  # probability of being occupied
    occ = np.full(p.shape, UNKNOWN, np.uint8)
    occ[p > occ_th] = LETHAL
    occ[p < free_th] = FREE
    if mode == 'scale':  # in-between values are costs, as occ_to_u8 maps them
        mid = (p >= free_th) & (p <= occ_th)
        pct = np.round(99.0 * (p[mid] - free_th) / (occ_th - free_th))
        occ[mid] = np.round(pct / 100.0 * LETHAL).astype(np.uint8)
    return {'occupancy': occ}, (origin[0], origin[1], origin[2] if len(origin) > 2 else 0.0), \
        res, 'map'


def grid_from_navmap(msg):
    """Rebuild the cell grid of a flat, grid-based NavMap: layers per cell, origin, resolution."""
    if len(msg.surfaces) != 1:
        sys.exit(f'expected one surface, found {len(msg.surfaces)}')
    x = np.array(msg.positions_x, float)
    y = np.array(msg.positions_y, float)
    z = np.array(msg.positions_z, float)
    if len(x) == 0:
        sys.exit('empty NavMap')
    if np.ptp(z) > 1e-6:
        sys.exit('the NavMap is not flat: only grid-based flat maps can be resampled')
    xs, ys = np.unique(np.round(x, 6)), np.unique(np.round(y, 6))
    if len(xs) < 2 or len(ys) < 2:
        sys.exit('the NavMap has no grid')
    res = xs[1] - xs[0]
    if not (np.allclose(np.diff(xs), res, atol=1e-4) and np.allclose(np.diff(ys), res, atol=1e-4)):
        sys.exit('the vertices are not a regular grid of square cells')
    w, h = len(xs) - 1, len(ys) - 1
    v = [np.array(msg.navcels_v0), np.array(msg.navcels_v1), np.array(msg.navcels_v2)]
    if len(v[0]) != 2 * w * h:
        sys.exit(f'expected {2 * w * h} triangles (2 per cell), found {len(v[0])}')
    cx = (x[v[0]] + x[v[1]] + x[v[2]]) / 3.0
    cy = (y[v[0]] + y[v[1]] + y[v[2]]) / 3.0
    ci = np.floor((cx - xs[0]) / res).astype(int)
    cj = np.floor((cy - ys[0]) / res).astype(int)
    if ci.min() < 0 or cj.min() < 0 or ci.max() >= w or cj.max() >= h:
        sys.exit('a triangle lies outside the grid')

    layers = {}
    for layer in msg.layers:
        if layer.type == NavMapLayer.U8:
            # Both triangles of a cell: the worst value (lethal > unknown > cost)
            vals = np.array(layer.data_u8, np.uint8)
            grid = np.zeros((h, w), np.uint8)
            np.maximum.at(grid, (cj, ci), vals)
        else:
            vals = np.array(layer.data_f32 if layer.type == NavMapLayer.F32 else layer.data_f64,
                            float)
            acc = np.zeros((h, w))
            cnt = np.zeros((h, w))
            np.add.at(acc, (cj, ci), np.where(np.isfinite(vals), vals, 0.0))
            np.add.at(cnt, (cj, ci), np.isfinite(vals))
            with np.errstate(invalid='ignore', divide='ignore'):
                grid = np.where(cnt > 0, acc / cnt, np.nan)
        layers[layer.name] = (layer.type, grid)
    frame = msg.surfaces[0].frame_id or msg.header.frame_id
    return layers, (xs[0], ys[0], z[0]), res, frame


# ------------------------------ Resampling -------------------------------

def blocks(grid, k, fill):
    """(h', w', k*k) view of k x k blocks; the border is padded with fill."""
    h, w = grid.shape
    hh, ww = math.ceil(h / k), math.ceil(w / k)
    padded = np.full((hh * k, ww * k), fill, grid.dtype)
    padded[:h, :w] = grid
    return padded.reshape(hh, k, ww, k).transpose(0, 2, 1, 3).reshape(hh, ww, k * k)


def resample_occupancy(grid, k):
    b = blocks(grid, k, UNKNOWN)
    known = b != UNKNOWN
    out = np.full(b.shape[:2], UNKNOWN, np.uint8)
    enough = known.sum(axis=2) * 2 >= k * k
    worst_known = np.where(known, b, 0).max(axis=2).astype(np.uint8)
    out[enough] = worst_known[enough]
    out[(b == LETHAL).any(axis=2)] = LETHAL
    return out


def resample(layers, k):
    out = {}
    for name, (ltype, grid) in layers.items():
        if name == 'occupancy':
            out[name] = (ltype, resample_occupancy(grid, k))
        elif ltype == NavMapLayer.U8:
            out[name] = (ltype, blocks(grid, k, 0).max(axis=2))
        else:
            with np.errstate(invalid='ignore'):
                out[name] = (ltype, np.nanmean(blocks(grid.astype(float), k, np.nan), axis=2))
    return out


# ------------------------------- Writing ---------------------------------

def build_navmap(layers, origin, res, frame):
    """Build a NavMap message with the structure navmap_ros::from_occupancy_grid builds."""
    h, w = next(iter(layers.values()))[1].shape
    jj, ii = np.meshgrid(np.arange(h + 1), np.arange(w + 1), indexing='ij')
    msg = NavMap()
    msg.header.frame_id = frame
    msg.positions_x = (origin[0] + ii * res).astype(np.float32).ravel().tolist()
    msg.positions_y = (origin[1] + jj * res).astype(np.float32).ravel().tolist()
    msg.positions_z = np.full((h + 1) * (w + 1), origin[2], np.float32).tolist()
    msg.has_vertex_rgba = False

    cj, ci = np.meshgrid(np.arange(h), np.arange(w), indexing='ij')
    vid = lambda i, j: (j * (w + 1) + i).ravel()  # noqa: E731
    a, b, c, d = vid(ci, cj), vid(ci + 1, cj), vid(ci + 1, cj + 1), vid(ci, cj + 1)
    # Two triangles per cell, interleaved: (a, b, c) then (a, c, d)
    msg.navcels_v0 = np.stack([a, a], axis=1).ravel().astype(np.uint32).tolist()
    msg.navcels_v1 = np.stack([b, c], axis=1).ravel().astype(np.uint32).tolist()
    msg.navcels_v2 = np.stack([c, d], axis=1).ravel().astype(np.uint32).tolist()

    surface = NavMapSurface()
    surface.frame_id = frame
    surface.navcels = list(range(2 * w * h))
    msg.surfaces = [surface]

    for name, (ltype, grid) in layers.items():
        layer = NavMapLayer()
        layer.header.frame_id = frame
        layer.name = name
        layer.type = ltype
        per_triangle = np.repeat(grid.ravel(), 2)  # both triangles of a cell
        if ltype == NavMapLayer.U8:
            layer.data_u8 = per_triangle.astype(np.uint8).tobytes()
        elif ltype == NavMapLayer.F32:
            layer.data_f32 = per_triangle.astype(np.float32).tolist()
        else:
            layer.data_f64 = per_triangle.astype(float).tolist()
        msg.layers.append(layer)
    return msg


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('input', help='input map: .navmap (grid-based, flat) or map_server .yaml')
    parser.add_argument('output', help='output .navmap file')
    parser.add_argument('resolution', type=float,
                        help='new cell size (m), an integer multiple of the input one')
    args, _ = parser.parse_known_args()

    if args.input.endswith('.yaml') or args.input.endswith('.yml'):
        layers, origin, res, frame = read_yaml_grid(args.input)
        layers = {'occupancy': (NavMapLayer.U8, layers['occupancy'])}
    else:
        layers, origin, res, frame = grid_from_navmap(read_navmap(args.input))
    if 'occupancy' not in layers:
        sys.exit('the input has no "occupancy" layer')

    k = args.resolution / res
    if k < 1.0 - 1e-6 or abs(k - round(k)) > 1e-4:
        sys.exit(f'the new resolution ({args.resolution} m) must be an integer multiple of the '
                 f'input one ({res:g} m)')
    k = int(round(k))
    out = resample(layers, k)
    msg = build_navmap(out, origin, res * k, frame)
    write_navmap(msg, args.output)

    occ_in, occ_out = layers['occupancy'][1], out['occupancy'][1]
    def stats(g):  # noqa: E306
        return f'free {np.sum(g == FREE)}, lethal {np.sum(g == LETHAL)}, ' \
               f'unknown {np.sum(g == UNKNOWN)}'
    print(f'input : {occ_in.shape[1]}x{occ_in.shape[0]} cells at {res:g} m ({stats(occ_in)})')
    print(f'output: {occ_out.shape[1]}x{occ_out.shape[0]} cells at {res * k:g} m '
          f'({stats(occ_out)}), {2 * occ_out.size} navcels -> {args.output}')
