"""Verify a new synthetic environment artifact without ROS, GUI or physics steps.

Default: stdlib-generated asymmetric box and explicit fixture provenance, followed
by actual CPU MuJoCo/URDF geometry checks against an unchanged existing base.
--with-freecad adds only an owned generated STEP; there is no user CAD input.
All generated files, including deliberate negative cases, stay in a new ignored
VerificationReport directory. Nothing is copied into docs/public or legacy output.
"""
from copy import deepcopy
import argparse
import itertools
import json
import math
import os
from pathlib import Path
import shutil
import time
import xml.etree.ElementTree as ET

if __package__:
    from .environment_assets import canonical,normalize,sha,validate_bundle
    from .environment_overlay import OUTPUTS,build_overlay,check_overlay,check_overlay_identity
else:  # Detached draft; root compatibility entry point imports the package branch.
    from environment_assets import canonical,normalize,sha,validate_bundle
    from environment_overlay import OUTPUTS,build_overlay,check_overlay,check_overlay_identity


def require(value,message):
    if not value:raise RuntimeError(message)


def synthetic_fixture():
    """Only our generated closed box; coordinates are deliberately not centered."""
    low=[-40.,-80.,-120.];high=[60.,120.,180.]
    tessellation=dict(valid=True,solid_count=1,bbox_min_mm=low,bbox_max_mm=high,
        vertices_mm=[list(point) for point in itertools.product(*zip(low,high))],
        triangles=[[0,2,3],[0,3,1],[4,5,7],[4,7,6],[0,1,5],[0,5,4],
            [2,6,7],[2,7,3],[0,4,6],[0,6,2],[1,3,7],[1,7,5]])
    spec=dict(id='synthetic_asymmetric_box_v1',declared_length_unit='mm',source_frame='fixture_source',
        asset_frame='fixture_asset',source_up_axis='Y',source_to_asset_rotation=[1,0,0,0,0,-1,0,1,0],
        expected_bbox_size_m=[.1,.3,.2],bbox_tolerance_m=1e-6,deflection_m=.0005,padding_m=.005)
    source=canonical(dict(schema_version=1,kind='generated_closed_box_fixture',
        tessellation=tessellation,normalization=spec))
    converter=dict(name='synthetic-fixture',version='1',script_sha256=sha(Path(__file__).read_bytes()))
    return source,tessellation,spec,converter


def inventory(directory):
    directory=Path(directory)
    require(directory.is_dir() and not directory.is_symlink(),'Expected an owned verification directory')
    result={}
    for path in sorted(directory.rglob('*')):
        require(not path.is_symlink(),'Verification artifact unexpectedly contains a symbolic link')
        if path.is_file():result[path.relative_to(directory).as_posix()]=sha(path.read_bytes())
        else:require(path.is_dir(),'Verification artifact unexpectedly contains a device')
    return result


def base_snapshot(cell):
    return {name:sha((cell.output/name).read_bytes()) for name in (*OUTPUTS,'source.json')}


def write_fixture_bundle(directory,source,tessellation,spec,converter):
    directory=Path(directory);directory.mkdir(mode=0o700)
    manifest,mesh=normalize(source,tessellation,spec,converter)
    (directory/'asset.json').write_bytes(canonical(manifest)+b'\n')
    (directory/'visual.stl').write_bytes(mesh)
    claimed=sha((directory/'asset.json').read_bytes())
    require(validate_bundle(directory,expected_sha256=claimed)==manifest,'Synthetic bundle round trip changed')
    return dict(directory=str(directory),manifest_sha256=claimed,source_sha256=sha(source),
        visual_sha256=manifest['visual']['sha256']),manifest


def rejected(function,label):
    try:function()
    except ValueError as error:
        return dict(check=label,rejected=True,error_type=type(error).__name__)
    raise RuntimeError('Negative case was accepted: '+label)


def normalization_negatives(source,tessellation,spec,converter):
    results=[]
    for factor,label in ((1000.,'imported_geometry_1000x_larger'),(.001,'imported_geometry_1000x_smaller')):
        wrong=deepcopy(tessellation)
        for name in ('bbox_min_mm','bbox_max_mm'):wrong[name]=[v*factor for v in wrong[name]]
        wrong['vertices_mm']=[[v*factor for v in point] for point in wrong['vertices_mm']]
        results.append(rejected(lambda wrong=wrong:normalize(source,wrong,spec,converter),label))
    for matrix,label in (([-1,0,0,0,1,0,0,0,1],'left_handed_axis_transform'),
                         ([2,0,0,0,1,0,0,0,1],'scaled_axis_transform')):
        wrong=deepcopy(spec);wrong.update(source_up_axis='Z',source_to_asset_rotation=matrix)
        results.append(rejected(lambda wrong=wrong:normalize(source,tessellation,wrong,converter),label))
    return results


def two_model_negatives(directory,cell,work_directory,*,atomic_json):
    """Both formats must be bound to the base+installation, even after rehashing.

    Copies retain the original final-path contract solely for this verification.
    They are never published/registered as usable overlay outputs.
    """
    results=[]
    for filename,suffix in (('cell.xml','mjcf_collision_shift'),('cell.urdf','urdf_collision_shrink')):
        copied=work_directory/('negative_'+suffix);shutil.copytree(directory,copied)
        # Prove a copy is valid before mutation, so a path relocation alone
        # cannot accidentally satisfy this expected-negative case.
        check_overlay(copied,cell,final_directory=directory)
        tree=ET.parse(copied/filename)
        if filename=='cell.xml':
            node=next(element for element in tree.getroot().iter('geom')
                if (element.get('name') or '').startswith('environment_') and element.get('name').endswith('_collision'))
            position=[float(v) for v in node.get('pos').split()];position[0]+=.01
            node.set('pos',' '.join(format(v,'.17g') for v in position))
        else:
            link=next(element for element in tree.getroot().findall('link')
                if (element.get('name') or '').startswith('environment_'))
            node=link.find('collision/geometry/box');size=[float(v) for v in node.get('size').split()];size[0]*=.5
            node.set('size',' '.join(format(v,'.17g') for v in size))
        tree.write(copied/filename,encoding='utf-8',xml_declaration=True)
        report=json.loads((copied/'overlay.json').read_bytes())
        report['outputs'][filename]=sha((copied/filename).read_bytes())
        atomic_json(copied/'overlay.json',report)
        result=rejected(lambda:check_overlay(copied,cell,final_directory=directory),suffix+'_with_recomputed_output_sha')
        result.update(directory=str(copied),manifest_sha256=sha((copied/'overlay.json').read_bytes()),
            changed_output=filename,changed_output_sha256=report['outputs'][filename])
        results.append(result)
    return results


def verify_artifacts(cell,report,*,with_freecad=False,native=False):
    from mujoco_ros2_manipulation.verification.reports import atomic_json
    work=report.directory/'artifacts';work.mkdir(mode=0o700)
    report.stage('checked_existing_base_and_synthetic_fixture')
    source,tessellation,spec,converter=synthetic_fixture()
    (work/'synthetic-fixture.json').write_bytes(source)
    receipt,manifest=write_fixture_bundle(work/'synthetic_bundle',source,tessellation,spec,converter)
    negatives=normalization_negatives(source,tessellation,spec,converter)
    report.update(normalization_negative_checks=negatives,synthetic_converter=converter,
        synthetic_source_sha256=receipt['source_sha256'],synthetic_source_file=str(work/'synthetic-fixture.json'),
        source_kind=manifest['source']['format'])
    require(manifest['source']['format']=='synthetic-fixture' and manifest['normalization']['converter']==converter,
            'Default CI fixture falsely claims an actual CAD/FreeCAD conversion')
    expected=[[-.04,-.18,-.08],[.06,.12,.12]]
    require(max(abs(a-b) for actual,want in zip((manifest['normalization']['cad_bbox_min_m'],
        manifest['normalization']['cad_bbox_max_m']),expected) for a,b in zip(actual,want))<=1e-12,
        'Synthetic axes/origin/metre normalization changed')
    placed=dict(name='synthetic_fixture',parent_frame='world',xyz_m=[0.,.65,.3],
        rpy_rad=[0.,0.,math.pi/2],rgba=[.5,.7,.8,1.])
    item=dict(bundle=Path(receipt['directory']),manifest_sha256=receipt['manifest_sha256'],installation=placed)
    overlays=[]

    def checked_build(asset,destination,overlay_id):
        started=time.perf_counter();value=build_overlay(cell,[asset],destination,overlay_id=overlay_id)
        claimed=sha((destination/'overlay.json').read_bytes())
        require(check_overlay(destination,cell,expected_sha256=claimed)==value
            and check_overlay_identity(destination,cell,expected_sha256=claimed)==value,'Overlay identity/CPU checks disagree')
        require(value['geometry_check']['physics_steps']==0 and value['geometry_check']['home_contacts']==0,
            'Artifact verification unexpectedly stepped physics or admitted a contact start')
        row=dict(directory=str(destination),overlay_id=overlay_id,manifest_sha256=claimed,
            artifact_kind=value['artifact_kind'],geometry=deepcopy(value['geometry_check']),
            asset_manifest_sha256=asset['manifest_sha256'],elapsed_wall_sec=time.perf_counter()-started,
            native=None)
        if native:
            report.stage('native_installed_geometry_'+overlay_id)
            if __package__:
                from .environment_native import check_native_overlay
            else:
                from mujoco_ros2_manipulation.workcells.environment_native import check_native_overlay
            native_work=work/('native_'+overlay_id);native_work.mkdir(mode=0o700)
            row['native']=check_native_overlay(destination,cell,value,native_work)
            require(type(row['native']) is dict,'Native overlay checker returned no receipt')
        overlays.append(row);report.update(checked_overlays=overlays)
        return value

    report.stage('actual_cpu_synthetic_overlay_geometry')
    directory=work/'synthetic_overlay_v1';checked_build(item,directory,'synthetic_overlay_v1')
    before=inventory(directory)
    existing=rejected(lambda:build_overlay(cell,[item],directory,overlay_id='must_not_replace'),
        'existing_populated_output_not_replaced')
    require(inventory(directory)==before,'Existing overlay changed during rejected replacement')
    empty=work/'existing_empty_output';empty.mkdir(mode=0o700)
    empty_check=rejected(lambda:build_overlay(cell,[item],empty,overlay_id='must_not_replace_empty'),
        'existing_empty_output_not_replaced')
    require(not list(empty.iterdir()),'Existing empty output was overwritten')
    report.update(existing_output_checks=[existing,empty_check],existing_output_preserved=True)
    report.stage('independent_mjcf_and_urdf_negative_contracts')
    model_negatives=two_model_negatives(directory,cell,work,atomic_json=atomic_json)
    require(inventory(directory)==before,'Negative copies changed the usable overlay')
    report.update(model_negative_checks=model_negatives)

    if with_freecad:
        report.stage('owned_generated_step_and_actual_freecad_conversion')
        if __package__:
            from .import_step import import_step,synthetic_fixture as create_step
        else:
            from import_step import import_step,synthetic_fixture as create_step
        step=create_step(work/'generated_negative_origin_box.step',executable='/usr/bin/freecadcmd')
        step_spec=deepcopy(spec);step_spec.update(id='synthetic_step_box_v1',source_up_axis='Z',
            source_to_asset_rotation=[1,0,0,0,1,0,0,0,1],expected_bbox_size_m=[.1,.2,.3])
        step_receipt=import_step(step,work/'step_bundle',step_spec,executable='/usr/bin/freecadcmd')
        step_manifest=validate_bundle(work/'step_bundle',expected_sha256=step_receipt['manifest_sha256'])
        require(step_manifest['source']['format']=='step' and step_manifest['normalization']['converter']['name']=='FreeCAD',
            'Optional STEP branch did not use actual FreeCAD provenance')
        # This byte substitution is specific to our one generated fixture, not a
        # general STEP-unit parser and never a modification of user CAD data.
        raw=step.read_bytes();marker=b'SI_UNIT(.MILLI.,.METRE.)'
        require(raw.count(marker)==1,'Generated STEP unit fixture layout unexpectedly changed')
        wrong=work/'generated_same_coordinates_metre_unit.step';wrong.write_bytes(raw.replace(marker,b'SI_UNIT($,.METRE.)'))
        wrong_spec=deepcopy(step_spec);wrong_spec['declared_length_unit']='m'
        destination=work/'must_not_publish_1000x_step'
        step_negative=rejected(lambda:import_step(wrong,destination,wrong_spec,executable='/usr/bin/freecadcmd'),
            'actual_generated_step_1000x_unit_mismatch')
        require(not destination.exists(),'Unit-mismatched synthetic STEP published an output')
        report.update(generated_step_source_sha256=sha(raw),actual_step_receipt=step_receipt,
            actual_step_unit_negative=step_negative)
        report.stage('actual_cpu_freecad_overlay_geometry')
        checked_build(dict(bundle=work/'step_bundle',manifest_sha256=step_receipt['manifest_sha256'],installation=placed),
            work/'step_overlay_v1','step_overlay_v1')
    return dict(checked_overlays=overlays,normalization_negative_checks=negatives,model_negative_checks=model_negatives,
        environment_artifacts_checked=True,two_model_contracts_checked=True,no_legacy_output_overwritten=True,
        existing_output_preserved=True,actual_freecad_checked=with_freecad,native_overlay_checked=native,
        ros_used=False,physics_steps=0,gui_requested=False,jpeg_requested=False,
        scope='Synthetic local artifact import/units/installed geometry only; no ROS, path clearance, settle or robot execution')


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workcell',default='fr5_ur5e')
    parser.add_argument('--with-freecad',action='store_true',help='Generate and import only an owned synthetic STEP with FreeCADCmd')
    parser.add_argument('--native',action='store_true',help='Also compile/check installed geometry with the pinned native 3.12 helper')
    parser.add_argument('--catalog-output',type=Path,help='New local preview catalog for the verified outputs (never replace)')
    args=parser.parse_args(argv)
    from mujoco_ros2_manipulation.paths import ROOT
    from mujoco_ros2_manipulation.verification.reports import VerificationReport,utc_now
    from mujoco_ros2_manipulation.workcells.config import load_workcell
    from mujoco_ros2_manipulation.workcells.integrity import check_workcell
    report=VerificationReport(ROOT,'environment_overlay','cpu',None)
    cell=before=metadata=None;failed=False
    report.update(with_freecad=args.with_freecad,native_requested=args.native,ros_used=False,physics_steps=0,
        verifier_sha256=sha(Path(__file__).read_bytes()),public_export_requested=False,
        scope='Local synthetic artifact verification only; no ROS/controller/physics execution')
    try:
        cell=load_workcell(args.workcell);metadata=deepcopy(check_workcell(cell));before=base_snapshot(cell)
        report.update(base_workcell_id=cell.id,base_workcell_sha256=cell.fingerprint,base_before_sha256=before)
        result=verify_artifacts(cell,report,with_freecad=args.with_freecad,native=args.native)
        report.update(**result)
        if args.catalog_output is not None:
            require(base_snapshot(cell)==before and check_workcell(cell)==metadata,'Base changed before publishing preview catalog')
            catalog=args.catalog_output.absolute()
            require(not catalog.exists() and not catalog.is_symlink(),'Preview catalog output already exists')
            raw=canonical(dict(schema_version=1,overlays=[dict(directory=row['directory'],
                manifest_sha256=row['manifest_sha256']) for row in result['checked_overlays']]))+b'\n'
            catalog.parent.mkdir(parents=True,exist_ok=True)
            descriptor=os.open(catalog,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
            with os.fdopen(descriptor,'wb') as stream:stream.write(raw)
            report.update(preview_catalog=str(catalog),preview_catalog_sha256=sha(raw))
    except (Exception,KeyboardInterrupt) as error:
        failed=True;report.update(error=dict(type=type(error).__name__,message=str(error)),failure_stage=report.data['stage'])
    finally:
        unchanged=False
        if cell is not None and before is not None:
            try:
                after=base_snapshot(cell);unchanged=after==before and check_workcell(cell)==metadata
                require(unchanged,'Existing base bytes/metadata changed during environment verification')
                report.update(base_after_sha256=after)
            except (Exception,KeyboardInterrupt) as error:
                failed=True;report.update(base_preservation_error=dict(type=type(error).__name__,message=str(error)))
        if sha(Path(__file__).read_bytes())!=report.data['verifier_sha256']:
            failed=True;report.update(verifier_changed_during_check=True)
        report.update(status='failed' if failed else 'passed',stage='complete',completed_utc=utc_now(),
            base_unchanged=unchanged,shutdown_clean=not failed)
    print(report.data['status'].upper()+': '+str(report.path),flush=True)
    if report.data.get('error'):print(report.data['error'],flush=True)
    return int(failed)


if __name__=='__main__':raise SystemExit(main())
