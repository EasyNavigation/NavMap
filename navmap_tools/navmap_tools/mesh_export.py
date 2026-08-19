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
Grid -> textured mesh export (COLLADA `.dae` visual + `.stl` collision).

Hand-rolled (no trimesh/open3d dependency): the geometry is always a regular
grid with two triangles per cell, so a small purpose-built writer is simpler
and lighter than pulling in a general mesh library. Mirrors the
STL-collision / DAE-visual split already used by
src/urjc-excavation-world/models/urjc_excavation/model.sdf.

Both meshes are Z-up (x=east, y=north, z=up), matching SDF's native frame, so
the model.sdf that references them needs no extra rotation. The DAE
explicitly declares `<up_axis>Z_UP</up_axis>` for the same reason: without
it, some COLLADA importers apply an implicit Y-up-to-Z-up rotation that would
put the visual mesh out of alignment with the (unrotated) STL collision mesh.
"""

from pathlib import Path

import numpy as np

from PIL import Image

from .terrain import TerrainGrid


def _grid_to_vertices_faces(grid: TerrainGrid):
    """
    Grid (rows=north/y, cols=east/x) -> shared vertices + CCW-from-above triangles.

    Winding must put the "front" face (right-hand-rule normal) up (+Z): a
    viewer above the terrain (the normal case) then sees the lit, textured
    front face instead of the backface-culled one, which is otherwise
    invisible and lets the scene background show through instead -- exactly
    the "one side white, other side background-grey" symptom a real
    generated world showed in Gazebo before this was caught. Confirmed by
    computing the normal of each winding directly: (v00, v10, v11) points
    -Z (down, wrong); (v00, v11, v10) points +Z (up, correct).
    """
    rows, cols = grid.elevation.shape
    verts = np.stack([grid.xs, grid.ys, grid.elevation], axis=-1).reshape(-1, 3)

    def vid(i, j):
        return i * cols + j

    faces = []
    for i in range(rows - 1):
        for j in range(cols - 1):
            v00, v01 = vid(i, j), vid(i, j + 1)
            v10, v11 = vid(i + 1, j), vid(i + 1, j + 1)
            faces.append((v00, v11, v10))
            faces.append((v00, v01, v11))
    return verts, np.asarray(faces, dtype=np.int64)


def _uvs_for_grid(grid: TerrainGrid, size_m: float) -> np.ndarray:
    """
    Grid -> COLLADA UVs (v=0 at the BOTTOM of the texture, per the COLLADA spec).

    gz-common's ColladaLoader::LoadTexCoords flips v (`1.0 - v`) on load to
    convert from COLLADA's bottom-left origin to Ogre's top-left one. Writing
    v already flipped here double-flips it, draping the north-up texture
    north-south mirrored in Gazebo (the NavMap side is unaffected: its
    per-vertex colors are assigned by 3D nearest-neighbor against the source
    imagery, not through this UV/DAE path at all -- see
    pointcloud_to_navmap.cpp). North (y=+half) must map to v=1 here so that,
    after gz's own flip, it lands on row 0 (the top, i.e. north row) of the
    texture PNG written by terrain.render_texture().
    """
    half = size_m / 2.0
    u = (grid.xs + half) / size_m
    v = (grid.ys + half) / size_m
    return np.stack([u, v], axis=-1).reshape(-1, 2)


def _vertex_normals(grid: TerrainGrid) -> np.ndarray:
    """
    Per-vertex smooth normals from the heightfield gradient.

    Required, not cosmetic: gz-rendering's Ogre2 PBS pipeline needs a NORMAL
    vertex attribute to shade with -- without one, it silently falls back to
    flat/unlit rendering and never samples the diffuse texture at all, which
    looks exactly like "no texture" even though the material/image data is
    completely valid. Confirmed empirically: a generated world stayed plain
    white (with a known-good reference texture swapped in, ruling out the
    image) until a NORMAL source was added, at which point the texture
    appeared immediately. See mesh_export.py's module history / decisions
    log in easynav_gis_tool.md for the full diagnostic trail.
    """
    dz_dy, dz_dx = np.gradient(
        grid.elevation.astype(np.float64), grid.spacing_m, grid.spacing_m)
    normals = np.stack([-dz_dx, -dz_dy, np.ones_like(dz_dx)], axis=-1)
    normals /= np.linalg.norm(normals, axis=-1, keepdims=True)
    return normals.reshape(-1, 3)


def write_stl(path, grid: TerrainGrid) -> int:
    """Write a binary STL (used as the Gazebo collision mesh); returns triangle count."""
    verts, faces = _grid_to_vertices_faces(grid)
    with open(Path(path), 'wb') as f:
        f.write(b'\x00' * 80)
        f.write(np.uint32(len(faces)).tobytes())
        for a, b, c in faces:
            p0, p1, p2 = verts[a], verts[b], verts[c]
            normal = np.cross(p1 - p0, p2 - p0)
            norm = np.linalg.norm(normal)
            if norm > 0:
                normal = normal / norm
            f.write(np.asarray(normal, dtype='<f4').tobytes())
            for p in (p0, p1, p2):
                f.write(np.asarray(p, dtype='<f4').tobytes())
            f.write(np.uint16(0).tobytes())
    return len(faces)


def write_texture_png(path, rgb: np.ndarray) -> None:
    """Write an (H, W, 3) uint8 array as a PNG texture."""
    if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
        raise ValueError(
            f'expected an (H, W, 3) uint8 array, got shape {rgb.shape} dtype {rgb.dtype}'
        )
    Image.fromarray(rgb, mode='RGB').save(Path(path))


_DAE_TEMPLATE = """<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <up_axis>Z_UP</up_axis>
  </asset>
  <library_images>
    <image id="terrain_texture" name="terrain_texture">
      <init_from>{texture_filename}</init_from>
    </image>
  </library_images>
  <library_effects>
    <effect id="terrain_effect">
      <profile_COMMON>
        <newparam sid="terrain_texture_surface">
          <surface type="2D"><init_from>terrain_texture</init_from></surface>
        </newparam>
        <newparam sid="terrain_texture_sampler">
          <sampler2D><source>terrain_texture_surface</source></sampler2D>
        </newparam>
        <technique sid="common">
          <lambert>
            <diffuse><texture texture="terrain_texture_sampler" texcoord="UVMap"/></diffuse>
          </lambert>
        </technique>
      </profile_COMMON>
    </effect>
  </library_effects>
  <library_materials>
    <material id="terrain_material" name="terrain_material">
      <instance_effect url="#terrain_effect"/>
    </material>
  </library_materials>
  <library_geometries>
    <geometry id="terrain_mesh" name="terrain_mesh">
      <mesh>
        <source id="terrain_positions">
          <float_array id="terrain_positions_array" count="{n_pos}">{positions}</float_array>
          <technique_common>
            <accessor source="#terrain_positions_array" count="{n_verts}" stride="3">
              <param name="X" type="float"/><param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <source id="terrain_normals">
          <float_array id="terrain_normals_array" count="{n_pos}">{normals}</float_array>
          <technique_common>
            <accessor source="#terrain_normals_array" count="{n_verts}" stride="3">
              <param name="X" type="float"/><param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <source id="terrain_uvs">
          <float_array id="terrain_uvs_array" count="{n_uv}">{uvs}</float_array>
          <technique_common>
            <accessor source="#terrain_uvs_array" count="{n_verts}" stride="2">
              <param name="S" type="float"/><param name="T" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <vertices id="terrain_vertices">
          <input semantic="POSITION" source="#terrain_positions"/>
          <input semantic="NORMAL" source="#terrain_normals"/>
        </vertices>
        <triangles material="terrain_material" count="{n_tris}">
          <input semantic="VERTEX" source="#terrain_vertices" offset="0"/>
          <input semantic="TEXCOORD" source="#terrain_uvs" offset="1" set="0"/>
          <p>{indices}</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="Scene" name="Scene">
      <node id="terrain_node" name="terrain_node">
        <instance_geometry url="#terrain_mesh">
          <bind_material>
            <technique_common>
              <instance_material symbol="terrain_material" target="#terrain_material"/>
            </technique_common>
          </bind_material>
        </instance_geometry>
      </node>
    </visual_scene>
  </library_visual_scenes>
  <scene>
    <instance_visual_scene url="#Scene"/>
  </scene>
</COLLADA>
"""


def write_dae(path, grid: TerrainGrid, size_m: float, texture_filename: str) -> int:
    """
    Write a textured COLLADA mesh; returns the triangle count.

    `texture_filename` is written verbatim into <init_from> and must be a
    path relative to the .dae file's directory.

    POSITION and TEXCOORD use distinct offsets (0 and 1) in `<p>`, each
    index repeated per corner even though UVs are computed per grid vertex
    (so the two numbers are always equal): gz-common's ColladaLoader sizes
    its `<p>` stride as the *count of declared (semantic, offset) inputs*,
    not `max(offset) + 1` -- giving VERTEX and TEXCOORD the same offset="0"
    (each its own distinct entry) makes it expect 2 values per corner while
    only 1 was written, desyncing the whole index stream into a garbage,
    invisible mesh. Confirmed by reading gz-common's ColladaLoader.cc
    (`LoadTriangles`, `offsetSize += input.second.size()`) after a generated
    world loaded in Gazebo with no visible geometry despite no load errors.
    """
    verts, faces = _grid_to_vertices_faces(grid)
    uvs = _uvs_for_grid(grid, size_m)
    normals = _vertex_normals(grid)

    positions_str = ' '.join(f'{v:.6f}' for v in verts.reshape(-1))
    normals_str = ' '.join(f'{v:.6f}' for v in normals.reshape(-1))
    uvs_str = ' '.join(f'{v:.6f}' for v in uvs.reshape(-1))
    indices_str = ' '.join(f'{i} {i}' for i in faces.reshape(-1))
    xml = _DAE_TEMPLATE.format(
        texture_filename=texture_filename,
        n_pos=verts.size,
        positions=positions_str,
        normals=normals_str,
        n_verts=len(verts),
        n_uv=uvs.size,
        uvs=uvs_str,
        n_tris=len(faces),
        indices=indices_str,
    )
    Path(path).write_text(xml)
    return len(faces)
