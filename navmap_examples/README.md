# navmap_examples

Pequeño paquete de **ejemplos** que usan la API real observada en tus tests (`navmap_core/NavMap.hpp`).  
Incluye ejemplos core (sin ROS) y ejemplos ROS 2 (opcionales).

## Build (colcon)
```bash
colcon build --packages-select navmap_examples   --cmake-args -DBUILD_ROS_EXAMPLES=ON
# o desactiva los ejemplos ROS 2:
# --cmake-args -DBUILD_ROS_EXAMPLES=OFF
```
> Si tu paquete `navmap_core` no exporta config CMake todavía, puedes pasar rutas de include con:
> `--cmake-args -DNAVMAP_CORE_INCLUDE_DIR=/ruta/a/include -DNAVMAP_ROS_INCLUDE_DIR=/ruta/a/include`

## Ejecutables (core)
- `01_flat_plane`
- `02_two_floors`
- `03_slope_surface`
- `04_layers`
- `05_neighbors_and_centroids`
- `06_area_marking`
- `07_raycast`
- `08_copy_and_assign`

## Ejecutables (ROS 2)
- `01_from_occgrid` (suscribe `map` → `NavMap`)
- `02_to_occgrid` (publica `navmap_grid`)
- `03_save_load` (serializa geometría+capas a JSON muy simple; demo de carga/guardado)
