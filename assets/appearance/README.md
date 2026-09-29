# Room and R5 appearance assets

All 16 source files are included locally (about 7 MB); simulation startup needs
no download. `source.lock.json` records exact source URLs, sizes and SHA-256.
Restore missing/damaged files with `python tools/fetch_appearance_assets.py`
(`--proxy http://127.0.0.1:7897` is optional and machine-specific).

## R5 black/silver materials

Source: [fiveages-sim/robot-descriptions-arx](https://github.com/fiveages-sim/robot-descriptions-arx/tree/71b069961eafe3d71524d199d99ac24c752b348a/arx5_description/meshes/r5),
revision `71b069961eafe3d71524d199d99ac24c752b348a`. The original BSD 3-Clause
license and ARXrobotics copyright notice are retained in `r5/LICENSE`.

These GLBs contain black/silver PBR material groups, not factory UV image maps.
They do not establish exact measured paint color, reflectance or sticker layout.
The existing manufacturer R5a URDF remains the physics and kinematics source.
The visual importer applies the GLB node transform and corrects link5's local
X offset by -0.0037 m to align it to the pinned manufacturer STL. After that
correction, all nine meshes' vertices agree within 1 micrometre. Source files
are preserved byte-for-byte; the correction is an explicit runtime transform.

## Room PBR textures

1K JPEG diffuse, roughness and OpenGL normal maps, downloaded from the official
Poly Haven file API and verified against its MD5 before pinning SHA-256:

- [Kitchen Wood](https://polyhaven.com/a/kitchen_wood): table, door and cabinet top.
- [Concrete Floor 02](https://polyhaven.com/a/concrete_floor_02): room floor.
- [Poly Haven CC0 license](https://polyhaven.com/license).

Room geometry and its metre-scaled UV coordinates are generated locally by
`src/robot_tasks/backends/lab_room.py`. No external executable or asset package
is installed. See [usage and experiment boundaries](../../docs/simulation_appearance.md).
