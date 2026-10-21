^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package navmap_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Mesh simplification keeping the layers
* .navmap files compressed with zlib
* Support for unknown values, int16 layers and log-odds occupancy
* Enable cpplint and copyright linters
* Contributors: Francisco Martín Rico

0.6.0 (2026-10-08)
------------------
* First release
* GIS tools to build a simulated world and its NavMap from satellite imagery and elevation (DEM) tiles
* navmap_map_builder: 3D map (.pcd, e.g. for Bonxai) of a Gazebo world, putting the clouds of one or more sensors together at the ground-truth poses
* navmap_map2d_from_pcd: 2D occupancy grid (map_server format) from a .pcd
* GeoTIFF tiles read with Pillow instead of tifffile (no rosdep key, and imagecodecs would also be needed)
* Conda packages with pixi (pixi-build-ros)
* Contributors: Francisco Martín Rico

