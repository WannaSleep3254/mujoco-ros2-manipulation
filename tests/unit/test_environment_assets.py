from copy import deepcopy
import itertools
import json
import struct
import stat
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch
from types import SimpleNamespace

from mujoco_ros2_manipulation.workcells.environment_assets import (canonical,check_spec,normalize,publish_directory,
    sha,stl_vertices,validate_bundle,_bounded_read)

def spec():return dict(id='synthetic_negative_box',declared_length_unit='mm',source_frame='cad',asset_frame='fixture',source_up_axis='Z',
    source_to_asset_rotation=[1,0,0,0,1,0,0,0,1],expected_bbox_size_m=[.1,.2,.3],bbox_tolerance_m=1e-6,deflection_m=.0005,padding_m=.005)

def tessellation():return dict(valid=True,solid_count=1,bbox_min_mm=[-50,-100,-150],bbox_max_mm=[50,100,150],
    vertices_mm=[list(p) for p in itertools.product([-50,50],[-100,100],[-150,150])],
    triangles=[[0,2,3],[0,3,1],[4,5,7],[4,7,6],[0,1,5],[0,5,4],[2,6,7],[2,7,3],[0,4,6],[0,6,2],[1,3,7],[1,7,5]])

CONVERTER=dict(name='FreeCAD',version='0.0-test',occt_version='0.0-test',script_sha256='a'*64,executable_sha256='b'*64)

class EnvironmentAssetTests(unittest.TestCase):
    def setUp(self):
        directory=TemporaryDirectory();self.addCleanup(directory.cleanup);self.root=Path(directory.name)

    def test_bounded_read_rejects_growth_after_stat_and_does_not_follow_final_links(self):
        path=self.root/'growing';path.write_bytes(b'123456789')
        with patch('mujoco_ros2_manipulation.workcells.environment_assets.os.fstat',return_value=SimpleNamespace(st_mode=stat.S_IFREG,st_size=1)):
            with self.assertRaisesRegex(ValueError,'byte limit'):_bounded_read(path,8)
        link=self.root/'linked';link.symlink_to(path)
        with self.assertRaises(OSError):_bounded_read(link,100)

    def test_synthetic_fixture_has_its_own_honest_exact_converter_and_source_format(self):
        converter=dict(name='synthetic-fixture',version='1',script_sha256='c'*64)
        source=canonical(dict(spec=spec(),tessellation=tessellation()))
        manifest,mesh=normalize(source,tessellation(),spec(),converter)
        self.assertEqual(manifest['source']['format'],'synthetic-fixture')
        self.assertNotIn('occt_version',manifest['normalization']['converter'])
        self.assertNotIn('executable_sha256',manifest['normalization']['converter'])
        directory=self.root/'synthetic';directory.mkdir()
        (directory/'visual.stl').write_bytes(mesh);(directory/'asset.json').write_bytes(canonical(manifest))
        self.assertEqual(validate_bundle(directory),manifest)
        manifest['source']['format']='step';(directory/'asset.json').write_bytes(canonical(manifest))
        with self.assertRaisesRegex(ValueError,'Source format'):
            validate_bundle(directory)
        with self.assertRaisesRegex(ValueError,'provenance'):
            normalize(source,tessellation(),spec(),dict(converter,occt_version='fake'))
        with self.assertRaisesRegex(ValueError,'canonical JSON'):
            normalize(source+b'\n',tessellation(),spec(),converter)

    def bundle(self):
        manifest,mesh=normalize(b'synthetic-step-bytes',tessellation(),spec(),CONVERTER)
        directory=self.root/'bundle';directory.mkdir()
        (directory/'asset.json').write_bytes(canonical(manifest)+b'\n');(directory/'visual.stl').write_bytes(mesh)
        return directory,manifest

    def test_mm_normalizes_once_preserves_negative_origin_and_explicit_padding(self):
        original=tessellation();before=deepcopy(original)
        manifest,raw=normalize(b'original-step',original,spec(),CONVERTER)
        self.assertEqual(original,before)
        self.assertEqual(manifest['source']['sha256'],sha(b'original-step'))
        self.assertEqual(manifest['normalization']['cad_bbox_min_m'],[-.05,-.1,-.15])
        self.assertEqual(manifest['normalization']['cad_bbox_max_m'],[.05,.1,.15])
        for value,expected in zip(manifest['collision']['size_m'],[.11,.21,.31]):self.assertAlmostEqual(value,expected)
        self.assertEqual(len(stl_vertices(raw)),36)
        # Declared source units never trigger another multiplier after FreeCAD.
        declared_m=spec();declared_m['declared_length_unit']='m'
        second,_=normalize(b'other-source',original,declared_m,CONVERTER)
        self.assertEqual(second['collision'],manifest['collision'])

    def test_1000x_imported_unit_mismatch_is_rejected_by_expected_physical_dimensions(self):
        wrong=tessellation()
        for key in ('bbox_min_mm','bbox_max_mm'):wrong[key]=[v*1000 for v in wrong[key]]
        wrong['vertices_mm']=[[v*1000 for v in p] for p in wrong['vertices_mm']]
        with self.assertRaisesRegex(ValueError,'physical dimensions'):
            normalize(b'metre-declaration-same-numerical-values',wrong,spec(),CONVERTER)

    def test_axis_conversion_is_proper_rotation_and_not_mirror_scale_or_double_install(self):
        rotated=spec();rotated.update(source_up_axis='Y',source_to_asset_rotation=[1,0,0,0,0,-1,0,1,0],expected_bbox_size_m=[.1,.3,.2])
        manifest,_=normalize(b'step',tessellation(),rotated,CONVERTER)
        self.assertEqual(manifest['normalization']['cad_bbox_min_m'],[-.05,-.15,-.1])
        for matrix in ([2,0,0,0,1,0,0,0,1],[-1,0,0,0,1,0,0,0,1],[1,.1,0,0,1,0,0,0,1]):
            bad=spec();bad['source_to_asset_rotation']=matrix
            with self.subTest(matrix=matrix),self.assertRaises(ValueError):check_spec(bad)
        bad=spec();bad['source_up_axis']='Y'
        with self.assertRaisesRegex(ValueError,'up axis'):check_spec(bad)

    def test_bundle_hash_inventory_and_no_symlink_or_collision_shrink(self):
        directory,manifest=self.bundle();claimed=sha((directory/'asset.json').read_bytes())
        self.assertEqual(validate_bundle(directory,expected_sha256=claimed),manifest)
        bad=deepcopy(manifest);bad['collision']['size_m'][0]-=.01
        (directory/'asset.json').write_bytes(canonical(bad))
        with self.assertRaises(ValueError):validate_bundle(directory)
        (directory/'asset.json').write_bytes(canonical(manifest))
        (directory/'unexpected.txt').write_text('private')
        with self.assertRaisesRegex(ValueError,'bundle files'):validate_bundle(directory)
        (directory/'unexpected.txt').unlink()
        mesh=(directory/'visual.stl').read_bytes();(directory/'visual.stl').unlink()
        target=self.root/'private.stl';target.write_bytes(mesh);(directory/'visual.stl').symlink_to(target)
        with self.assertRaisesRegex(ValueError,'links'):validate_bundle(directory)

    def test_invalid_mesh_and_missing_explicit_units_fail_closed(self):
        for change in (lambda d:d.update(valid=False),lambda d:d.update(solid_count=2),
                lambda d:d['triangles'][0].__setitem__(0,999),lambda d:d['vertices_mm'][0].__setitem__(0,float('nan'))):
            invalid=tessellation();change(invalid)
            with self.assertRaises(ValueError):normalize(b'step',invalid,spec(),CONVERTER)
        bad=spec();bad['declared_length_unit']='unknown'
        with self.assertRaises(ValueError):check_spec(bad)
        bad=spec();bad['hidden_input_path']='/home/private/cad.step'
        with self.assertRaises(ValueError):check_spec(bad)

    def test_atomic_publish_never_overwrites_existing_output(self):
        source=self.root/'stage';source.mkdir();(source/'data').write_text('new')
        target=self.root/'target';target.mkdir()
        with self.assertRaisesRegex(ValueError,'already exists'):publish_directory(source,target)
        self.assertTrue((source/'data').is_file());self.assertEqual(list(target.iterdir()),[])
        target.rmdir();publish_directory(source,target)
        self.assertEqual((target/'data').read_text(),'new');self.assertFalse(source.exists())

    def test_binary_stl_nonfinite_normal_and_byte_count_are_rejected(self):
        _,raw=normalize(b'step',tessellation(),spec(),CONVERTER)
        bad=bytearray(raw);struct.pack_into('<f',bad,84,float('nan'))
        with self.assertRaises(ValueError):stl_vertices(bytes(bad))
        with self.assertRaises(ValueError):stl_vertices(raw[:-1])

if __name__=='__main__':unittest.main()
