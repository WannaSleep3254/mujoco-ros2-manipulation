"""New immutable environment overlays over a checked base; legacy builds stay intact.

This is a new artifact/validator, NOT a workcells.config/check_workcell-compatible
workcell. It does not launch ROS, step physics, switch a viewer or publish commands.
An explicit launch/view adapter is required before using the new output.
"""
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import stat
import tempfile
from urllib.parse import unquote,urlsplit
import xml.etree.ElementTree as ET

if __package__:
    from .environment_assets import (bbox,canonical,corners,digest,identifier,installation,
        publish_directory,require,rotation,rpy_matrix,sha,stl_vertices,transformed,validate_bundle,vector)
else:
    from environment_assets import (bbox,canonical,corners,digest,identifier,installation,
        publish_directory,require,rotation,rpy_matrix,sha,stl_vertices,transformed,validate_bundle,vector)

OUTPUTS=('cell.xml','cell.urdf','cell.srdf','controllers.yaml','moveit.yaml','moveit.rviz')
PRODUCERS=('environment_assets.py','environment_overlay.py','import_step.py','freecad_step_worker.py')
MAX_FILE_BYTES=128*1024*1024

def _read(path,maximum=MAX_FILE_BYTES):
    descriptor=os.open(path,os.O_RDONLY|os.O_NONBLOCK|getattr(os,'O_NOFOLLOW',0))
    try:
        info=os.fstat(descriptor)
        require(stat.S_ISREG(info.st_mode) and info.st_size<=maximum,'Overlay input is not a bounded regular file')
        with os.fdopen(descriptor,'rb',closefd=False) as stream:raw=stream.read(maximum+1)
        require(len(raw)<=maximum,'Overlay input exceeds byte limit')
        return raw
    finally:os.close(descriptor)

def _json(raw):
    def pairs(items):
        result={}
        for key,value in items:require(key not in result,'Duplicate overlay metadata field');result[key]=value
        return result
    result=json.loads(raw,object_pairs_hook=pairs,parse_constant=lambda _:(_ for _ in ()).throw(ValueError('Nonfinite overlay metadata')))
    canonical(result)
    return result

def _producer_hashes():
    return {name:sha(_read(Path(__file__).with_name(name),1024*1024)) for name in PRODUCERS}

def _numbers(values):return ' '.join(format(value,'.17g') for value in values)

def _quat(rpy):
    r,p,y=[v/2 for v in rpy];cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return [cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy]

def _xml(root):
    ET.indent(root)
    return ET.tostring(root,encoding='utf-8',xml_declaration=True)

def _inventory(directory):
    result={};total=0
    for path in sorted(directory.rglob('*')):
        require(not path.is_symlink() and (path.is_dir() or path.is_file()),'Overlay links/devices are forbidden')
        if path.is_file() and path!=directory/'overlay.json':
            raw=_read(path);total+=len(raw)
            require(total<=256*1024*1024,'Overlay output inventory exceeds byte limit')
            result[path.relative_to(directory).as_posix()]=sha(raw)
    return result

def _base_checker(cell):
    from mujoco_ros2_manipulation.workcells.integrity import check_workcell
    return check_workcell(cell)

def append_environment(mj,robot,manifest,placed,relative_mesh,final_mesh):
    """The SAME asset-local collider and installation transform go to both models."""
    name='environment_'+placed['name']
    occupied={element.get('name') for root in (mj,robot) for element in root.iter() if element.get('name')}
    require(not occupied.intersection({name,name+'_fixed',name+'_visual',name+'_collision',name+'_mesh'}),'Environment name collision')
    asset=mj.find('asset')
    if asset is None:asset=ET.SubElement(mj,'asset')
    ET.SubElement(asset,'mesh',{'name':name+'_mesh','file':relative_mesh})
    body=ET.SubElement(mj.find('worldbody'),'body',{'name':name,'pos':_numbers(placed['xyz_m']),'quat':_numbers(_quat(placed['rpy_rad']))})
    ET.SubElement(body,'geom',{'name':name+'_visual','type':'mesh','mesh':name+'_mesh','group':'0',
        'contype':'0','conaffinity':'0','rgba':_numbers(placed['rgba'])})
    collider=manifest['collision']
    ET.SubElement(body,'geom',{'name':name+'_collision','type':'box','group':'3','pos':_numbers(collider['center_m']),
        'size':_numbers([v/2 for v in collider['size_m']]),'contype':'1','conaffinity':'1','rgba':'0.8 0.2 0.1 0.25'})
    link=ET.SubElement(robot,'link',{'name':name})
    visual=ET.SubElement(link,'visual');geometry=ET.SubElement(visual,'geometry')
    ET.SubElement(geometry,'mesh',{'filename':final_mesh.as_uri(),'scale':'1 1 1'})
    material=ET.SubElement(visual,'material',{'name':name+'_material'})
    ET.SubElement(material,'color',{'rgba':_numbers(placed['rgba'])})
    collision=ET.SubElement(link,'collision')
    ET.SubElement(collision,'origin',{'xyz':_numbers(collider['center_m']),'rpy':'0 0 0'})
    ET.SubElement(ET.SubElement(collision,'geometry'),'box',{'size':_numbers(collider['size_m'])})
    joint=ET.SubElement(robot,'joint',{'name':name+'_fixed','type':'fixed'})
    ET.SubElement(joint,'parent',{'link':'world'});ET.SubElement(joint,'child',{'link':name})
    ET.SubElement(joint,'origin',{'xyz':_numbers(placed['xyz_m']),'rpy':_numbers(placed['rpy_rad'])})

def _composed_xml(base_cell,entries,bundles,overlay_id,final_directory):
    mj=ET.fromstring(_read(base_cell.output/'cell.xml',16*1024*1024))
    robot=ET.fromstring(_read(base_cell.output/'cell.urdf',16*1024*1024))
    semantic=ET.fromstring(_read(base_cell.output/'cell.srdf',16*1024*1024))
    mj.set('model',overlay_id);robot.set('name',overlay_id);semantic.set('name',overlay_id)
    require(mj.find('worldbody') is not None,'Base has no world frame')
    for node in mj.iter():
        if node.get('file') and not Path(node.get('file')).is_absolute():
            node.set('file',str((base_cell.output/node.get('file')).resolve()))
    for entry in entries:
        append_environment(mj,robot,bundles[entry['bundle_path']],entry['installation'],
            entry['bundle_path']+'/visual.stl',final_directory/entry['bundle_path']/'visual.stl')
    hardware=robot.find("ros2_control/hardware/param[@name='mujoco_model']")
    require(hardware is not None,'Base lacks the expected simulation hardware model path')
    hardware.text=str(final_directory/'cell.xml')
    return mj,robot,semantic

def _package_roots(base_cell):
    roots={}
    for instance in getattr(base_cell,'instances',()):
        profile=instance.profile
        for package,relative in profile.data['source']['mesh_packages'].items():
            root=profile.path(relative).resolve()
            require(package not in roots or roots[package]==root,'Ambiguous base mesh package')
            roots[package]=root
    return roots

def _references(directory,base_cell,*,final_directory=None):
    """Resolve only actual XML mesh/texture references, never manifest paths."""
    final_directory=directory if final_directory is None else final_directory
    result=set();package_roots=_package_roots(base_cell)
    mj=ET.fromstring(_read(directory/'cell.xml',16*1024*1024))
    require(mj.find('.//include') is None,'Nested MJCF includes require a separate dependency adapter')
    compiler=mj.find('compiler')
    if compiler is not None:
        require(all(compiler.get(key) in (None,'.','') for key in ('meshdir','texturedir','assetdir')),
            'Overlay requires explicit model-local or absolute asset paths')
    for node in mj.iter():
        value=node.get('file')
        if value is None:continue
        require(node.tag in ('mesh','texture','hfield','skin'),'Unknown MJCF file dependency')
        require('://' not in value,'MJCF file references must be local')
        path=Path(value);path=path if path.is_absolute() else directory/path
        result.add(path.resolve())
    robot=ET.fromstring(_read(directory/'cell.urdf',16*1024*1024))
    for node in robot.findall('.//mesh'):
        value=node.get('filename');require(isinstance(value,str) and value,'Missing URDF mesh dependency')
        if value.startswith('package://'):
            package,relative=value.removeprefix('package://').split('/',1)
            require(package in package_roots,'Unknown base URDF mesh package')
            path=(package_roots[package]/relative).resolve()
            require(path.is_relative_to(package_roots[package]),'URDF mesh escaped its base package')
        elif value.startswith('file://'):
            uri=urlsplit(value);require(not uri.netloc and not uri.query and not uri.fragment,'Nonlocal URDF mesh URI')
            path=Path(unquote(uri.path));require(path.is_absolute(),'URDF file URI must be absolute')
            if path.is_relative_to(final_directory):path=directory/path.relative_to(final_directory)
            path=path.resolve()
        else:
            require('://' not in value,'Unknown URDF mesh URI')
            path=Path(value);path=(path if path.is_absolute() else directory/path).resolve()
        result.add(path)
    return result

def _dependencies(directory,base_cell,*,final_directory=None):
    base_allowed=_references(base_cell.output.resolve(),base_cell)
    result={};total=0
    for path in sorted(_references(directory,base_cell,final_directory=final_directory)):
        if path.is_relative_to(directory):continue  # Exact owned inventory covers it.
        require(path in base_allowed,'Overlay introduced an external asset outside the checked base')
        raw=_read(path);total+=len(raw)
        require(total<=512*1024*1024,'Overlay external dependencies exceed byte limit')
        result[str(path)]=dict(sha256=sha(raw),bytes=len(raw))
    return result

def geometry_check(directory,entries,joint_names,home,*,final_directory=None):
    """CPU mj_forward only. Check installed visual bounds and both box transforms."""
    import mujoco
    directory=Path(directory);final_directory=directory if final_directory is None else Path(final_directory)
    model=mujoco.MjModel.from_xml_path(str(directory/'cell.xml'));data=mujoco.MjData(model)
    require(model.njnt==len(joint_names),'Overlay changed the number of robot joints')
    for name,value in zip(joint_names,home):
        joint_id=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,name)
        require(joint_id>=0,'Overlay lost a base joint');data.qpos[int(model.jnt_qposadr[joint_id])]=value
    mujoco.mj_forward(model,data)
    robot=ET.parse(directory/'cell.urdf').getroot()
    require([joint.get('name') for joint in robot.findall('ros2_control/joint')]==list(joint_names),'URDF hardware joint contract changed')
    require(robot.find("ros2_control/hardware/param[@name='mujoco_model']").text==str(final_directory/'cell.xml'),'Overlay hardware still points at another model')
    max_bounds=max_centers=max_corners=0.
    for entry in entries:
        placed=installation(entry['installation']);name='environment_'+placed['name']
        bundle=directory/entry['bundle_path'];manifest=validate_bundle(bundle,expected_sha256=entry['manifest_sha256'])
        matrix=rpy_matrix(placed['rpy_rad']);offset=placed['xyz_m'];collider=manifest['collision']
        asset_points=stl_vertices(_read(bundle/'visual.stl',32*1024*1024))
        expected_low,expected_high=bbox([transformed(point,matrix,offset) for point in asset_points])
        visual_id=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,name+'_visual')
        collision_id=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,name+'_collision')
        require(visual_id>=0 and collision_id>=0 and int(model.geom_group[collision_id])==3
            and not model.geom_contype[visual_id] and not model.geom_conaffinity[visual_id]
            and model.geom_contype[collision_id] and model.geom_conaffinity[collision_id],'Visual/collision roles changed')
        mesh_id=int(model.geom_dataid[visual_id]);start=int(model.mesh_vertadr[mesh_id]);count=int(model.mesh_vertnum[mesh_id])
        actual_low,actual_high=bbox([transformed(point.tolist(),data.geom_xmat[visual_id].tolist(),data.geom_xpos[visual_id].tolist())
            for point in model.mesh_vert[start:start+count]])
        max_bounds=max(max_bounds,*[abs(a-b) for a,b in zip(actual_low+actual_high,expected_low+expected_high)])
        expected_center=transformed(collider['center_m'],matrix,offset)
        max_centers=max(max_centers,*[abs(a-b) for a,b in zip(data.geom_xpos[collision_id],expected_center)])
        actual_size=[float(value)*2 for value in model.geom_size[collision_id]]
        require(max(abs(a-b) for a,b in zip(actual_size,collider['size_m']))<1e-12,'MuJoCo box dimensions changed')
        require(max(abs(a-b) for a,b in zip(data.geom_xmat[collision_id],matrix))<1e-12,'MuJoCo installation rotation changed')
        fixed=robot.find("joint[@name='"+name+"_fixed']")
        require(fixed is not None and fixed.get('type')=='fixed' and fixed.find('parent').get('link')=='world'
            and fixed.find('child').get('link')==name,'URDF installation frame changed')
        xyz=[float(v) for v in fixed.find('origin').get('xyz').split()]
        rpy=[float(v) for v in fixed.find('origin').get('rpy').split()]
        urdf_matrix=rpy_matrix(rpy)
        require(xyz==placed['xyz_m'] and max(abs(a-b) for a,b in zip(urdf_matrix,matrix))<1e-12,'URDF installation pose differs')
        link=robot.find("link[@name='"+name+"']")
        visual=link.find('visual/geometry/mesh');collision=link.find('collision')
        require(visual.get('filename')==(final_directory/entry['bundle_path']/'visual.stl').as_uri()
            and visual.get('scale')=='1 1 1','URDF visual must use the same metre mesh without rescaling')
        urdf_center=[float(v) for v in collision.find('origin').get('xyz').split()]
        urdf_size=[float(v) for v in collision.find('geometry/box').get('size').split()]
        require(urdf_center==collider['center_m'] and urdf_size==collider['size_m'],'URDF collider differs from declared AABB')
        lows=[urdf_center[k]-urdf_size[k]/2 for k in range(3)];highs=[urdf_center[k]+urdf_size[k]/2 for k in range(3)]
        urdf_world=[transformed(point,urdf_matrix,xyz) for point in corners(lows,highs)]
        mj_lows=[-v/2 for v in actual_size];mj_highs=[v/2 for v in actual_size]
        mj_world=[transformed(point,data.geom_xmat[collision_id].tolist(),data.geom_xpos[collision_id].tolist()) for point in corners(mj_lows,mj_highs)]
        max_corners=max(max_corners,*[abs(a-b) for left,right in zip(urdf_world,mj_world) for a,b in zip(left,right)])
    require(max(max_bounds,max_centers,max_corners)<=1e-6,'Installed world geometry differs between source/URDF/MuJoCo')
    require(data.ncon==0,'Overlay home has contacts; do not publish it as a usable start')
    return dict(engine='MuJoCo '+mujoco.__version__,physics_steps=0,home_contacts=int(data.ncon),
        visual_world_bbox_error_m=max_bounds,collision_center_error_m=max_centers,urdf_mjcf_corner_error_m=max_corners)

def build_overlay(base_cell,assets,destination,*,overlay_id,base_validator=_base_checker):
    import yaml
    identifier(overlay_id);requested=Path(destination).absolute()
    require(overlay_id!=base_cell.id and not requested.exists() and not requested.is_symlink(),'Overlay requires a new ID and output directory')
    destination=requested.resolve()
    require(not destination.is_relative_to(base_cell.output.resolve()),'Overlay cannot overwrite or nest inside active base output')
    before=base_validator(base_cell);base_raw=_read(base_cell.output/'source.json',4*1024*1024)
    require(set(before['outputs'])==set(OUTPUTS),'Unsupported base output inventory')
    require(type(assets) is list and 1<=len(assets)<=32,'Expected a bounded explicit asset list')
    entries=[];bundles={};names=set()
    for item in assets:
        require(type(item) is dict and set(item)=={'bundle','manifest_sha256','installation'},'Invalid overlay asset fields')
        placed=installation(item['installation']);require(placed['name'] not in names,'Duplicate installed environment name');names.add(placed['name'])
        bundle=Path(item['bundle']).resolve();manifest=validate_bundle(bundle,expected_sha256=item['manifest_sha256'])
        relative='assets/'+item['manifest_sha256'];bundles[relative]=(bundle,manifest)
        entries.append(dict(bundle_path=relative,manifest_sha256=item['manifest_sha256'],asset_id=manifest['id'],installation=placed))
    inputs=dict(base_manifest_sha256=sha(base_raw),base_workcell_sha256=base_cell.fingerprint,
        asset_installations=deepcopy(entries),producer_sha256=_producer_hashes())
    destination.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.environment-overlay-',dir=destination.parent) as temporary:
        stage=Path(temporary)/'output';stage.mkdir(mode=0o700)
        for name in OUTPUTS:(stage/name).write_bytes(_read(base_cell.output/name,16*1024*1024))
        for relative,(bundle,manifest) in bundles.items():
            copied=stage/relative;copied.mkdir(parents=True)
            for name in ('asset.json','visual.stl'):(copied/name).write_bytes(_read(bundle/name,32*1024*1024))
        mj,robot,semantic=_composed_xml(base_cell,entries,{relative:value[1] for relative,value in bundles.items()},overlay_id,destination)
        (stage/'cell.xml').write_bytes(_xml(mj));(stage/'cell.urdf').write_bytes(_xml(robot));(stage/'cell.srdf').write_bytes(_xml(semantic))
        moveit=yaml.safe_load((stage/'moveit.yaml').read_text());moveit['robot_description']=ET.tostring(robot,encoding='unicode');moveit['robot_description_semantic']=ET.tostring(semantic,encoding='unicode')
        (stage/'moveit.yaml').write_text(yaml.safe_dump(moveit,sort_keys=False))
        dependencies=_dependencies(stage,base_cell,final_directory=destination)
        inputs['dependencies']=deepcopy(dependencies)
        measured=geometry_check(stage,entries,base_cell.joints,base_cell.home,final_directory=destination)
        require(base_validator(base_cell)==before and sha(_read(base_cell.output/'source.json',4*1024*1024))==sha(base_raw),'Base changed during overlay composition')
        for relative,(bundle,_) in bundles.items():validate_bundle(bundle,expected_sha256=Path(relative).name)
        report=dict(schema_version=1,artifact_kind='environment-overlay-v1',id=overlay_id,
            fingerprint=sha(canonical(inputs)),base_workcell_id=base_cell.id,inputs=inputs,
            base_manifest=deepcopy(before),joint_names=list(base_cell.joints),home_rad=list(base_cell.home),joint_limits=deepcopy(before['joint_limits']),
            installations=entries,dependencies=dependencies,geometry_check=measured,outputs=_inventory(stage),
            integration_status='Artifact only; explicit viewer/launch adapter required. No ROS/physics execution verified.')
        (stage/'overlay.json').write_bytes(canonical(report)+b'\n')
        check_overlay(stage,base_cell,base_validator=base_validator,final_directory=destination)
        publish_directory(stage,destination)
    return check_overlay(destination,base_cell,base_validator=base_validator)

def check_overlay_identity(directory,base_cell,*,expected_sha256=None,base_validator=_base_checker,final_directory=None):
    """Pure file/contract validation; never imports MuJoCo, YAML or steps physics.

    Geometry evidence is shape-checked and bound to exact outputs, not recomputed.
    A trusted CPU preflight receipt remains required before first viewer use.
    """
    directory=Path(directory)
    require(directory.is_absolute() and directory.is_dir() and not directory.is_symlink(),'Overlay must be an explicit real directory')
    directory=directory.resolve();final_directory=directory if final_directory is None else Path(final_directory).absolute()
    raw=_read(directory/'overlay.json',4*1024*1024)
    if expected_sha256 is not None:require(sha(raw)==digest(expected_sha256),'Overlay manifest SHA mismatch')
    report=_json(raw)
    require(type(report) is dict and set(report)=={'schema_version','artifact_kind','id','fingerprint','base_workcell_id','inputs',
        'base_manifest','joint_names','home_rad','joint_limits','installations','dependencies','geometry_check','outputs','integration_status'},'Invalid overlay metadata fields')
    require(type(report['schema_version']) is int and report['schema_version']==1 and report['artifact_kind']=='environment-overlay-v1','Not an environment overlay artifact')
    identifier(report['id']);digest(report['fingerprint'])
    require(report['id']!=base_cell.id and report['base_workcell_id']==base_cell.id,'Overlay/base identity mismatch')
    require(type(report['inputs']) is dict and set(report['inputs'])=={'base_manifest_sha256','base_workcell_sha256','asset_installations','producer_sha256','dependencies'},'Invalid overlay input fields')
    current=base_validator(base_cell)
    require(report['base_manifest']==current and report['inputs']['base_manifest_sha256']==sha(_read(base_cell.output/'source.json',4*1024*1024))
        and report['inputs']['base_workcell_sha256']==base_cell.fingerprint,'Overlay base provenance changed')
    require(report['inputs']['producer_sha256']==_producer_hashes() and report['fingerprint']==sha(canonical(report['inputs'])),'Overlay producer/input fingerprint changed')
    require(report['joint_names']==list(base_cell.joints) and report['home_rad']==list(base_cell.home)
        and report['joint_limits']==current['joint_limits'] and report['installations']==report['inputs']['asset_installations'],'Overlay changed robot joints/home/installation provenance')
    require(type(report['installations']) is list and 1<=len(report['installations'])<=32,'Invalid installed asset count')
    names=set();bundles={};files=set(OUTPUTS);directories={'assets'}
    for entry in report['installations']:
        require(type(entry) is dict and set(entry)=={'bundle_path','manifest_sha256','asset_id','installation'},'Invalid installed asset fields')
        placed=installation(entry['installation']);require(placed['name'] not in names,'Duplicate installed environment name');names.add(placed['name'])
        require(entry['bundle_path']=='assets/'+digest(entry['manifest_sha256']),'Unowned overlay asset path')
        manifest=validate_bundle(directory/entry['bundle_path'],expected_sha256=entry['manifest_sha256'])
        require(entry['asset_id']==manifest['id'],'Installed asset identity differs from its bundle')
        bundles[entry['bundle_path']]=manifest;directories.add(entry['bundle_path'])
        files.update(entry['bundle_path']+'/'+name for name in ('asset.json','visual.stl'))
    require(type(report['outputs']) is dict and set(report['outputs'])==files and report['outputs']==_inventory(directory),'Overlay outputs or asset inventory changed')
    require({p.relative_to(directory).as_posix() for p in directory.rglob('*') if p.is_dir()}==directories,'Unexpected overlay directories')
    for name in ('controllers.yaml','moveit.rviz'):
        require(_read(directory/name)==_read(base_cell.output/name),'Overlay changed base controller/RViz contract')
    expected=_composed_xml(base_cell,report['installations'],bundles,report['id'],final_directory)
    for name,root in zip(('cell.xml','cell.urdf','cell.srdf'),expected):
        require(_xml(ET.fromstring(_read(directory/name,16*1024*1024)))==_xml(root),'Overlay XML contract changed: '+name)
    dependencies=_dependencies(directory,base_cell,final_directory=final_directory)
    require(report['dependencies']==dependencies and report['inputs']['dependencies']==dependencies,'Overlay external dependencies changed')
    measured=report['geometry_check']
    require(type(measured) is dict and set(measured)=={'engine','physics_steps','home_contacts','visual_world_bbox_error_m','collision_center_error_m','urdf_mjcf_corner_error_m'},'Invalid CPU geometry evidence fields')
    require(isinstance(measured['engine'],str) and measured['engine'].startswith('MuJoCo ')
        and type(measured['physics_steps']) is int and measured['physics_steps']==0
        and type(measured['home_contacts']) is int and measured['home_contacts']==0,'Invalid read-only CPU geometry evidence')
    require(all(type(measured[key]) in (int,float) and math.isfinite(measured[key]) and 0<=measured[key]<=1e-6
        for key in ('visual_world_bbox_error_m','collision_center_error_m','urdf_mjcf_corner_error_m')),'Invalid geometry tolerance evidence')
    require(report['integration_status']=='Artifact only; explicit viewer/launch adapter required. No ROS/physics execution verified.','Unsupported overlay integration claim')
    return report

def check_overlay(directory,base_cell,*,expected_sha256=None,base_validator=_base_checker,final_directory=None):
    """Full validation: pure identity plus mandatory fresh CPU geometry and YAML."""
    import yaml
    directory=Path(directory).absolute()
    report=check_overlay_identity(directory,base_cell,expected_sha256=expected_sha256,base_validator=base_validator,final_directory=final_directory)
    geometry=geometry_check(directory,report['installations'],base_cell.joints,base_cell.home,final_directory=final_directory)
    require(geometry==report['geometry_check'],'Overlay geometry evidence changed')
    moveit=yaml.safe_load((directory/'moveit.yaml').read_text())
    for key,name in (('robot_description','cell.urdf'),('robot_description_semantic','cell.srdf')):
        require(ET.tostring(ET.fromstring(moveit[key]))==ET.tostring(ET.parse(directory/name).getroot()),'MoveIt uses a different robot/scene description')
    require(check_overlay_identity(directory,base_cell,expected_sha256=expected_sha256,base_validator=base_validator,final_directory=final_directory)==report,
        'Overlay changed during CPU verification')
    return report
