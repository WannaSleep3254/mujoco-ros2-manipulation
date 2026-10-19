"""Visual scene fidelity and physical-model regression checks."""

from copy import deepcopy
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import collada
import mujoco
import numpy as np

from mujoco_ros2_manipulation.model_build.collada import export_dae_visuals
from mujoco_ros2_manipulation.model_build.prepare import prepare
from mujoco_ros2_manipulation.profiles import ROOT, load_profile
from tests.helpers import TemporaryOutputProfile, axis_rotation, urdf_fk


class ColladaTests(unittest.TestCase):
    def test_scene_units_transforms_colors_and_normals_are_preserved(self):
        document = collada.Collada()
        document.assetInfo.unitmeter = 0.001
        document.assetInfo.unitname = 'millimeter'
        document.assetInfo.upaxis = 'Z_UP'
        points = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=float)
        vertices = collada.source.FloatSource('vertices', points, ('X', 'Y', 'Z'))
        normals = collada.source.FloatSource('normals', np.ones((4, 3)), ('X', 'Y', 'Z'))
        geometry = collada.geometry.Geometry(document, 'geometry', 'test geometry', [vertices, normals])
        inputs = collada.source.InputList()
        inputs.addInput(0, 'VERTEX', '#vertices')
        inputs.addInput(1, 'NORMAL', '#normals')
        faces = np.array([[0, 1, 2], [0, 3, 1], [0, 2, 3], [1, 3, 2]])
        colors = [(0.8, 0.1, 0.1, 1.0), (0.1, 0.2, 0.8, 1.0)]
        material_nodes = []
        for index, color in enumerate(colors):
            symbol = f'color{index}'
            effect = collada.material.Effect(f'effect{index}', [], 'phong', diffuse=color)
            material = collada.material.Material(f'material{index}', symbol, effect)
            document.effects.append(effect)
            document.materials.append(material)
            indices = np.stack((faces[index*2:index*2+2], faces[index*2:index*2+2]), axis=-1).ravel()
            geometry.primitives.append(geometry.createTriangleSet(indices, inputs, symbol))
            material_nodes.append(collada.scene.MaterialNode(symbol, material, inputs=[]))
        document.geometries.append(geometry)
        node = collada.scene.Node('scaled', children=[collada.scene.GeometryNode(geometry, material_nodes)],
            transforms=[collada.scene.TranslateTransform(10, 20, 30),
                        collada.scene.ScaleTransform(-2, 3, 4)])
        scene = collada.scene.Scene('scene', [node])
        document.scenes.append(scene)
        document.scene = scene
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / 'scene.dae'
            with source.open('wb') as output:
                document.write(output)
            parts = export_dae_visuals(source, temporary, 'test')
            self.assertEqual([part.rgba for part in parts], colors)
            self.assertEqual(sum(part.triangles for part in parts), 4)
            expected_vertices = (points * [-2, 3, 4] + [10, 20, 30]) * 0.001
            expected_normal = np.array([-0.5, 1/3, 0.25])
            expected_normal /= np.linalg.norm(expected_normal)
            for part in parts:
                rows = [line.split() for line in part.path.read_text().splitlines()]
                actual_vertices = np.array([[float(v) for v in row[1:]] for row in rows if row[0] == 'v'])
                actual_normals = np.array([[float(v) for v in row[1:]] for row in rows if row[0] == 'vn'])
                np.testing.assert_allclose(actual_vertices, expected_vertices, atol=1e-9, rtol=0)
                np.testing.assert_allclose(actual_normals, np.tile(expected_normal, (4, 1)), atol=1e-8, rtol=0)
                first_face = next(row for row in rows if row[0] == 'f')
                face = [int(value.split('//')[0])-1 for value in first_face[1:]]
                expected_face = faces[colors.index(part.rgba)*2][::-1]
                np.testing.assert_array_equal(face, expected_face)


