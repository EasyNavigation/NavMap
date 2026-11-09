# Copyright 2026 Intelligent Robotics Lab
#
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
A local height grid (`geo.dem_file.LocalDem`) as a Gazebo terrain, and a world from a template.

`write_gazebo_terrain` writes, in a directory:
  terrain.stl         binary STL, the COLLISION geometry
  terrain.obj / .mtl  the same mesh with UVs and smooth normals, the VISUAL geometry
  terrain.png         the imagery (one texel per node of the full grid), only if the grid has it
in the map frame (x east, y north, z relative to the grid's reference altitude), so a robot spawned
at (0, 0) stands on z = 0 and Gazebo's world is the NavMap's map frame. `terrain_resolution`
above the grid's makes the mesh coarser (every k-th node, k a divisor of side - 1): a collision
mesh of millions of triangles is too heavy for the simulator; the texture keeps the full grid.

`write_world_from_template` fills @LATITUDE@, @LONGITUDE@, @ELEVATION@ (m above sea level of the
world origin, z = 0) and @TERRAIN_DIR@ in a world template.
"""

import io
from pathlib import Path

import numpy as np

from PIL import Image

from .dem_file import LocalDem


def _stride_for(dem: LocalDem, terrain_resolution: float) -> int:
    """Largest divisor of side - 1 not above the requested coarsening."""
    if terrain_resolution <= dem.resolution:
        return 1
    want = int(round(terrain_resolution / dem.resolution))
    for k in range(min(want, dem.side - 1), 0, -1):
        if (dem.side - 1) % k == 0:
            return k
    return 1


def _cell_triangles(n: int) -> np.ndarray:
    """Node index triples (T, 3) as build_navmap does: (v00, v10, v11) and (v00, v11, v01)."""
    j, i = np.mgrid[0:n - 1, 0:n - 1]
    v00 = (j * n + i).ravel()
    v10, v01, v11 = v00 + 1, v00 + n, v00 + n + 1
    tris = np.empty((v00.size * 2, 3), dtype=np.int64)
    tris[0::2] = np.stack([v00, v10, v11], axis=1)
    tris[1::2] = np.stack([v00, v11, v01], axis=1)
    return tris


def write_gazebo_terrain(dem: LocalDem, directory, terrain_resolution: float = 0.0) -> None:
    """Write terrain.stl, terrain.obj, terrain.mtl (and terrain.png) into `directory`."""
    out = Path(directory)
    out.mkdir(parents=True, exist_ok=True)
    stride = _stride_for(dem, terrain_resolution)
    z = dem.z[::stride, ::stride]
    n = z.shape[0]
    res = dem.resolution * stride
    n_tex = dem.side
    # node k of the subsampled grid is node k * stride of the DEM, whose centre is (side - 1) // 2
    coords = (np.arange(n) * stride - (dem.side - 1) // 2) * dem.resolution
    gx, gy = np.meshgrid(coords, coords)
    verts = np.stack([gx.ravel(), gy.ravel(), z.ravel().astype(np.float64)], axis=1)
    verts = verts.astype(np.float32)
    tris = _cell_triangles(n)

    # collision: binary STL
    p = verts[tris]   # (T, 3, 3)
    normal = np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0])
    normal /= np.maximum(np.linalg.norm(normal, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(tris), dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', 9), ('a', '<u2')]))
    rec['n'] = normal
    rec['v'] = p.reshape(-1, 9)
    with open(out / 'terrain.stl', 'wb') as f:
        f.write(b'navmap_tools terrain'.ljust(80, b'\0'))
        f.write(np.uint32(len(tris)).tobytes())
        f.write(rec.tobytes())

    # visual: OBJ with UVs (texel centres: image row 0 is the north edge) and smooth normals
    zf = z.astype(np.float64)
    i0, i1 = np.maximum(np.arange(n) - 1, 0), np.minimum(np.arange(n) + 1, n - 1)
    dzdx = (zf[:, i1] - zf[:, i0]) / ((i1 - i0) * res)[None, :]
    dzdy = (zf[i1, :] - zf[i0, :]) / ((i1 - i0) * res)[:, None]
    length = np.sqrt(dzdx ** 2 + dzdy ** 2 + 1.0)
    vn = np.stack([-dzdx / length, -dzdy / length, 1.0 / length], axis=-1).reshape(-1, 3)
    uu, vv = np.meshgrid(
        (np.arange(n) * stride + 0.5) / n_tex, (np.arange(n) * stride + 0.5) / n_tex)
    vt = np.stack([uu.ravel(), vv.ravel()], axis=1)
    buf = io.StringIO()
    buf.write('mtllib terrain.mtl\nusemtl terrain\n')
    np.savetxt(buf, verts, fmt='v %.6g %.6g %.6g')
    np.savetxt(buf, vt, fmt='vt %.6g %.6g')
    np.savetxt(buf, vn, fmt='vn %.6g %.6g %.6g')
    idx = tris + 1
    np.savetxt(buf, np.repeat(idx, 3, axis=1), fmt='f %d/%d/%d %d/%d/%d %d/%d/%d')
    (out / 'terrain.obj').write_text(buf.getvalue())

    rgb = dem.rgb
    textured = rgb is not None
    mtl = 'newmtl terrain\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nd 1\nillum 1\n'
    if textured:
        mtl += 'map_Kd terrain.png\n'
        Image.fromarray(np.ascontiguousarray(rgb[::-1]), 'RGB').save(out / 'terrain.png')
    (out / 'terrain.mtl').write_text(mtl)


def write_world_from_template(dem: LocalDem, template_path, world_path, terrain_dir) -> None:
    """Instantiate a world template for `dem` (see the module docstring)."""
    text = Path(template_path).read_text()
    if '@TERRAIN_DIR@' not in text:
        raise ValueError('world template has no @TERRAIN_DIR@ placeholder')

    def num(v):
        return '%.10g' % v

    text = (text.replace('@LATITUDE@', num(dem.origin_lat))
            .replace('@LONGITUDE@', num(dem.origin_lon))
            .replace('@ELEVATION@', num(dem.reference_altitude))
            .replace('@TERRAIN_DIR@', str(Path(terrain_dir).resolve())))
    world = Path(world_path)
    world.parent.mkdir(parents=True, exist_ok=True)
    world.write_text(text)
