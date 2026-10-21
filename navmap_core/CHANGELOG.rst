^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package navmap_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2026-10-21)
------------------
* Rewrite from_points as a multi-level grid mesher
* BVH over the surfaces for raycast, locate and closest queries; locate_navcel fixed on multi-surface maps
* Rays through a shared edge always hit a NavCel
* Unknown values, int16 layers and log-odds occupancy
* Layers combined with explicit operators and provenance; deep copy of layers
* Geometry ids, sparse layer updates and layer change tracking
* Distance field, incremental distance field and per-NavCel geometric descriptors
* Links between non-adjacent NavCels
* NavCel iterators
* Mesh simplification keeping the layers
* Enable cpplint and copyright linters
* Contributors: Francisco Martín Rico

0.6.0 (2026-10-08)
------------------
* Conda packages with pixi (pixi-build-ros)
* Contributors: Francisco Martín Rico

0.5.0 (2026-07-25)
------------------
* Update version
* Speedup the navcel location
* Cleanup unused headers
* Fix potential linker error and warning
* Merge branch 'jazzy' into rolling
* Contributors: Francisco Martín Rico, Francisco Miguel Moreno

0.4.0 (2025-11-24)
------------------
* Speedup the navcel location
* Cleanup unused headers
* Fix potential linker error and warning
* Merge branch 'jazzy' into rolling
* Contributors: Francisco Martín Rico, Francisco Miguel Moreno

0.2.5 (2025-10-17)
------------------

0.2.4 (2025-10-16)
------------------

0.2.3 (2025-10-16)
------------------
* Fix dependencies
* Contributors: Francisco Martín Rico

0.2.2 (2025-10-12)
------------------
* Creates a surface if there isn't anyone
* Contributors: Francisco Martín Rico

0.2.1 (2025-09-25)
------------------
* Fix Eigen3 dep in package.xml
* Contributors: Francisco Martín Rico

0.2.0 (2025-09-25)
------------------
* Change License to Apache 2
* Fix no update layers when dirty
* Set by areas (cicrcular or rectangular)
* Improve efficiency in layer copy
* Add Contructors
* NavMap assignment Optimization
* Conversion from/to messages for layers
* Easy API, tests and Readme
* Fix locate_navcel
* Mesh form points
* Stable version with complete basic functionality
* Basic NavMap structures
* Contributors: Francisco Martín Rico
