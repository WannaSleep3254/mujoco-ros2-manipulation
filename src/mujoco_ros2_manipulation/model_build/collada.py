"""Convert constant-color COLLADA scene meshes into MuJoCo OBJ visuals."""

from dataclasses import dataclass
from pathlib import Path

import collada
import numpy as np


@dataclass(frozen=True)
class VisualPart:
    path: Path
    rgba: tuple
    triangles: int
    bounds_m: list


def export_dae_visuals(source, directory, prefix):
    """Preserve scene transforms, units, surface normals and per-material colors.

    Each color becomes a separate OBJ/geom because MuJoCo's URDF importer does
    not read COLLADA or OBJ material files. These assets are visual-only.
    """
    document = collada.Collada(str(source))
    if document.scene is None:
        raise ValueError(f'COLLADA file has no active scene: {source}')
    unit = float(document.assetInfo.unitmeter or 1.0)
    rotations = {
        'Z_UP': np.eye(3),
        'Y_UP': np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]]),
        'X_UP': np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]]),
    }
    rotation = rotations[document.assetInfo.upaxis or 'Y_UP']
    groups = {}
    for geometry in document.scene.objects('geometry'):
        linear = rotation @ np.asarray(geometry.matrix[:3, :3], dtype=float) * unit
        translation = rotation @ geometry.matrix[:3, 3] * unit
        for primitive in geometry.primitives():
            if isinstance(primitive, collada.lineset.BoundLineSet):
                continue  # CAD construction lines are not robot surfaces.
            if isinstance(primitive, collada.polylist.BoundPolylist):
                primitive = primitive.triangleset()
            if not isinstance(primitive, collada.triangleset.BoundTriangleSet):
                raise ValueError(f'Unsupported COLLADA primitive: {type(primitive).__name__}')
            if not len(primitive):
                continue
            material = primitive.material
            diffuse = material.effect.diffuse if material is not None else (0.7, 0.7, 0.7, 1.0)
            if not isinstance(diffuse, (tuple, list)) or len(diffuse) not in [3, 4]:
                raise ValueError(f'Expected a constant diffuse color in {source}')
            rgba = tuple(float(v) for v in diffuse)
            if len(rgba) == 3:
                rgba += (1.0,)
            vertices = np.asarray(primitive.original.vertex, dtype=float) @ linear.T + translation
            faces = primitive.vertex_index.copy()
            if primitive.original.normal is not None:
                # Inverse transpose also handles nonuniform scale correctly.
                normals = np.asarray(primitive.original.normal, dtype=float) @ np.linalg.inv(linear)
                normal_indices = primitive.normal_index.copy()
            else:
                triangles = vertices[faces]
                normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
                normal_indices = np.repeat(np.arange(len(faces))[:, None], 3, axis=1)
            if np.linalg.det(linear) < 0:
                faces = faces[:, ::-1]
                normal_indices = normal_indices[:, ::-1]
                if primitive.original.normal is None:
                    normals *= -1
            lengths = np.linalg.norm(normals, axis=1)
            if not np.isfinite(vertices).all() or not np.isfinite(lengths).all() or np.any(lengths == 0):
                raise ValueError(f'Invalid COLLADA vertices or normals: {source}')
            normals /= lengths[:, None]
            groups.setdefault(rgba, []).append((vertices, faces, normals, normal_indices))
    if not groups:
        raise ValueError(f'COLLADA file has no triangle surfaces: {source}')
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    parts = []
    for index, (rgba, chunks) in enumerate(groups.items()):
        path = directory / f'{prefix}_{index}.obj'
        vertex_offset = normal_offset = triangle_count = 0
        all_vertices = []
        with path.open('w') as output:
            output.write('# Generated from the original COLLADA visual; coordinates in meters.\n')
            for vertices, faces, normals, normal_indices in chunks:
                np.savetxt(output, vertices, fmt='v %.9g %.9g %.9g')
                np.savetxt(output, normals, fmt='vn %.9g %.9g %.9g')
                for face, normal in zip(faces + vertex_offset + 1,
                                        normal_indices + normal_offset + 1):
                    output.write('f ' + ' '.join(f'{v}//{n}' for v, n in zip(face, normal)) + '\n')
                vertex_offset += len(vertices)
                normal_offset += len(normals)
                triangle_count += len(faces)
                all_vertices.append(vertices)
        vertices = np.vstack(all_vertices)
        parts.append(VisualPart(path, rgba, triangle_count,
                                [vertices.min(0).tolist(), vertices.max(0).tolist()]))
    return parts
