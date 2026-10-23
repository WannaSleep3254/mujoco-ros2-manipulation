"""Meter-normalized local CAD bundles; no ROS, CAD engine or simulator imports."""
from copy import deepcopy
import ctypes
import errno
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import re
import stat
import struct

MAX_VERTICES=250_000
MAX_TRIANGLES=500_000
MAX_STL_BYTES=84+50*MAX_TRIANGLES
SPEC_FIELDS={'id','declared_length_unit','source_frame','asset_frame','source_up_axis',
    'source_to_asset_rotation','expected_bbox_size_m','bbox_tolerance_m','deflection_m','padding_m'}

def require(value,message):
    if not value:raise ValueError(message)

def _bounded_read(path,maximum):
    descriptor=os.open(path,os.O_RDONLY|os.O_NONBLOCK|getattr(os,'O_NOFOLLOW',0))
    try:
        info=os.fstat(descriptor)
        require(stat.S_ISREG(info.st_mode) and info.st_size<=maximum,'Asset input is not a bounded regular file')
        with os.fdopen(descriptor,'rb',closefd=False) as stream:raw=stream.read(maximum+1)
        require(len(raw)<=maximum,'Asset input exceeds byte limit')
        return raw
    finally:os.close(descriptor)

def canonical(value):
    return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False,ensure_ascii=True).encode('ascii')

def sha(raw):return hashlib.sha256(raw).hexdigest()

def number(value,*,nonnegative=False,maximum=1e6):
    require(type(value) in (int,float) and abs(value)<=maximum and math.isfinite(value)
        and (not nonnegative or value>=0),'Expected a finite bounded number')
    return float(value)

def vector(value,n=3):
    require(type(value) is list and len(value)==n,'Invalid vector length')
    return [number(item) for item in value]

def identifier(value):
    require(isinstance(value,str) and len(value)<=80 and re.fullmatch('[a-z][a-z0-9_]*',value),'Invalid asset identifier')
    return value

def digest(value):
    require(isinstance(value,str) and re.fullmatch('[0-9a-f]{64}',value),'Invalid SHA256')
    return value

def exact(value,fields,message):require(type(value) is dict and set(value)==set(fields),message)

def converter_info(value):
    if type(value) is dict and value.get('name')=='synthetic-fixture':
        exact(value,{'name','version','script_sha256'},'Invalid synthetic fixture provenance')
        require(value['version']=='1','Unsupported synthetic fixture version')
        digest(value['script_sha256'])
        return
    exact(value,{'name','version','occt_version','script_sha256','executable_sha256'},'Invalid converter provenance')
    require(value['name']=='FreeCAD' and all(isinstance(value[k],str) and len(value[k])<=100
        and re.fullmatch('[0-9][A-Za-z0-9_.+-]*',value[k]) for k in ('version','occt_version')),'Invalid converter version')
    digest(value['script_sha256']);digest(value['executable_sha256'])

def rotation(value):
    matrix=vector(value,9)
    for row in range(3):
        for other in range(row,3):
            require(abs(sum(matrix[row*3+k]*matrix[other*3+k] for k in range(3))-(row==other))<=1e-9,
                    'Axis transform must be orthonormal')
    a,b,c,d,e,f,g,h,i=matrix
    require(abs(a*(e*i-f*h)-b*(d*i-f*g)+c*(d*h-e*g)-1)<=1e-9,'Axis transform must preserve handedness')
    return matrix

def transformed(point,matrix,translation=(0.,0.,0.)):
    return [sum(matrix[row*3+col]*point[col] for col in range(3))+translation[row] for row in range(3)]

def bbox(points):
    require(bool(points),'Empty geometry')
    return [min(p[k] for p in points) for k in range(3)],[max(p[k] for p in points) for k in range(3)]

def corners(low,high):return [list(p) for p in itertools.product(*zip(low,high))]

def check_spec(spec):
    require(type(spec) is dict and set(spec)==SPEC_FIELDS,'Unknown/missing normalization fields')
    result=deepcopy(spec)
    for name in ('id','source_frame','asset_frame'):identifier(spec[name])
    require(spec['source_frame']!=spec['asset_frame'],'Source and asset frames must be explicit and distinct')
    require(spec['declared_length_unit'] in ('mm','m'),'Declare a known STEP length unit; do not guess')
    require(spec['source_up_axis'] in ('X','Y','Z'),'Invalid source up axis')
    matrix=rotation(spec['source_to_asset_rotation'])
    source_up=[float(k=='XYZ'.index(spec['source_up_axis'])) for k in range(3)]
    require(max(abs(a-b) for a,b in zip(transformed(source_up,matrix),[0,0,1]))<=1e-9,'Declared source up axis must map to asset +Z')
    dimensions=vector(spec['expected_bbox_size_m'])
    require(all(v>0 for v in dimensions),'Expected physical dimensions must be positive')
    for name in ('bbox_tolerance_m','deflection_m','padding_m'):
        require(0<number(spec[name],nonnegative=True)<=.1,'Explicit tolerances/padding must be positive and <=0.1m')
    return result

def stl_bytes(vertices,triangles):
    output=bytearray(b'mujoco-ros2 environment visual, metres'.ljust(80,b'\0'))
    output+=struct.pack('<I',len(triangles))
    for face in triangles:
        a,b,c=[vertices[index] for index in face]
        first=[b[k]-a[k] for k in range(3)];second=[c[k]-a[k] for k in range(3)]
        normal=[first[1]*second[2]-first[2]*second[1],first[2]*second[0]-first[0]*second[2],first[0]*second[1]-first[1]*second[0]]
        length=math.sqrt(sum(v*v for v in normal));require(length>1e-18,'Degenerate visual triangle')
        output+=struct.pack('<12fH',*(v/length for v in normal),*a,*b,*c,0)
    return bytes(output)

def stl_vertices(raw):
    require(type(raw) is bytes and 84<=len(raw)<=MAX_STL_BYTES,'Invalid bounded binary STL')
    count=struct.unpack_from('<I',raw,80)[0]
    require(1<=count<=MAX_TRIANGLES and len(raw)==84+50*count,'STL triangle count/bytes mismatch')
    result=[]
    for offset in range(84,len(raw),50):
        values=struct.unpack_from('<12fH',raw,offset)
        for value in values[:12]:number(value)
        for index in (3,6,9):result.append(vector(list(values[index:index+3])))
    return result

def normalize(source_raw,tessellation,spec,converter):
    """FreeCAD already interpreted STEP units; convert INTERNAL mm to m once.

    The declared STEP unit is provenance, not a second scale factor. Mandatory
    expected metre dimensions reject a 1000x unit mismatch rather than guessing.
    The conservative box covers the imported exact shape AABB (not just chords).
    """
    spec=check_spec(spec)
    require(type(source_raw) is bytes and source_raw,'Missing original STEP bytes')
    fields={'vertices_mm','triangles','bbox_min_mm','bbox_max_mm','valid','solid_count'}
    require(type(tessellation) is dict and set(tessellation)==fields,'Invalid tessellation fields')
    require(tessellation['valid'] is True and type(tessellation['solid_count']) is int and tessellation['solid_count']==1,
            'MVP requires one valid closed solid')
    points=tessellation['vertices_mm'];faces=tessellation['triangles']
    require(type(points) is list and 4<=len(points)<=MAX_VERTICES and type(faces) is list and 4<=len(faces)<=MAX_TRIANGLES,'Mesh exceeds resource bounds')
    source_points=[vector(point) for point in points]
    for face in faces:
        require(type(face) is list and len(face)==3 and all(type(i) is int and 0<=i<len(points) for i in face)
            and len(set(face))==3,'Invalid triangle vertex indices')
    low,high=vector(tessellation['bbox_min_mm']),vector(tessellation['bbox_max_mm'])
    require(all(a<b for a,b in zip(low,high)),'Imported shape bbox must have positive extent')
    require(all(low[k]-1e-6<=point[k]<=high[k]+1e-6 for point in source_points for k in range(3)),'Tessellation escapes imported CAD bbox')
    matrix=spec['source_to_asset_rotation']
    vertices=[transformed([value*.001 for value in point],matrix) for point in source_points]
    cad_low,cad_high=bbox([transformed([v*.001 for v in point],matrix) for point in corners(low,high)])
    actual=[b-a for a,b in zip(cad_low,cad_high)]
    require(max(abs(a-b) for a,b in zip(actual,spec['expected_bbox_size_m']))<=spec['bbox_tolerance_m'],
            'Imported physical dimensions differ from explicit expected metres; check STEP unit interpretation')
    raw=stl_bytes(vertices,faces)
    visual_low,visual_high=bbox(stl_vertices(raw))
    padding=spec['padding_m'];center=[(a+b)/2 for a,b in zip(cad_low,cad_high)]
    size=[b-a+2*padding for a,b in zip(cad_low,cad_high)]
    converter_info(converter)
    source_format='step'
    if converter['name']=='synthetic-fixture':
        descriptor=json.loads(source_raw,parse_constant=lambda _:(_ for _ in ()).throw(ValueError('Nonfinite fixture descriptor')))
        require(type(descriptor) is dict and source_raw==canonical(descriptor),'Synthetic fixture source must be a canonical JSON descriptor')
        source_format='synthetic-fixture'
    manifest=dict(schema_version=1,artifact_kind='environment-asset-v1',id=spec['id'],
        source=dict(format=source_format,sha256=sha(source_raw),declared_length_unit=spec['declared_length_unit'],frame=spec['source_frame'],up_axis=spec['source_up_axis'],handedness='right'),
        normalization=dict(converter=deepcopy(converter),imported_length_unit='mm',metres_per_imported_unit=.001,
            source_to_asset_rotation=deepcopy(matrix),origin_policy='preserve',asset_frame=spec['asset_frame'],up_axis='Z',handedness='right',
            tessellation_linear_deflection_m=spec['deflection_m'],expected_bbox_size_m=spec['expected_bbox_size_m'],
            bbox_tolerance_m=spec['bbox_tolerance_m'],cad_bbox_min_m=cad_low,cad_bbox_max_m=cad_high),
        visual=dict(path='visual.stl',sha256=sha(raw),bytes=len(raw),triangles=len(faces),length_unit='m',bbox_min_m=visual_low,bbox_max_m=visual_high),
        collision=dict(method='aabb_cover',basis='imported_shape_bbox',padding_m=padding,center_m=center,size_m=size))
    return manifest,raw

def validate_bundle(directory,*,expected_sha256=None):
    directory=Path(directory)
    require(directory.is_dir() and not directory.is_symlink(),'Bundle must be an owned real directory')
    require({p.name for p in directory.iterdir()}=={'asset.json','visual.stl'},'Unexpected/missing bundle files')
    for name in ('asset.json','visual.stl'):require(not (directory/name).is_symlink() and (directory/name).is_file(),'Bundle links/devices are forbidden')
    require((directory/'asset.json').stat().st_size<=1024*1024 and (directory/'visual.stl').stat().st_size<=MAX_STL_BYTES,'Bundle exceeds byte limits')
    raw=_bounded_read(directory/'asset.json',1024*1024)
    if expected_sha256 is not None:require(sha(raw)==digest(expected_sha256),'Asset manifest SHA mismatch')
    def pairs(items):
        result={}
        for key,value in items:require(key not in result,'Duplicate asset field');result[key]=value
        return result
    data=json.loads(raw,object_pairs_hook=pairs,parse_constant=lambda _:(_ for _ in ()).throw(ValueError('Nonfinite manifest')))
    require(type(data) is dict and set(data)=={'schema_version','artifact_kind','id','source','normalization','visual','collision'}
        and type(data['schema_version']) is int and data['schema_version']==1 and data['artifact_kind']=='environment-asset-v1','Invalid asset schema')
    exact(data['source'],{'format','sha256','declared_length_unit','frame','up_axis','handedness'},'Unknown source fields')
    exact(data['normalization'],{'converter','imported_length_unit','metres_per_imported_unit','source_to_asset_rotation',
        'origin_policy','asset_frame','up_axis','handedness','tessellation_linear_deflection_m','expected_bbox_size_m',
        'bbox_tolerance_m','cad_bbox_min_m','cad_bbox_max_m'},'Unknown normalization fields')
    exact(data['visual'],{'path','sha256','bytes','triangles','length_unit','bbox_min_m','bbox_max_m'},'Unknown visual fields')
    exact(data['collision'],{'method','basis','padding_m','center_m','size_m'},'Unknown collision fields')
    require(data['source']['handedness']=='right','Invalid source handedness')
    identifier(data['id']);digest(data['source']['sha256']);converter_info(data['normalization']['converter'])
    expected_format='synthetic-fixture' if data['normalization']['converter']['name']=='synthetic-fixture' else 'step'
    require(data['source']['format']==expected_format,'Source format differs from explicit converter provenance')
    visual=data['visual'];require(visual['path']=='visual.stl' and visual['length_unit']=='m','Only bundle-local metre STL is supported')
    mesh=_bounded_read(directory/'visual.stl',MAX_STL_BYTES);points=stl_vertices(mesh)
    require(type(visual['bytes']) is int and type(visual['triangles']) is int and sha(mesh)==digest(visual['sha256'])
        and len(mesh)==visual['bytes'] and len(points)==visual['triangles']*3,'Mesh SHA/count mismatch')
    low,high=bbox(points);require(low==visual['bbox_min_m'] and high==visual['bbox_max_m'],'Visual bbox differs from actual STL')
    norm=data['normalization'];require(norm['imported_length_unit']=='mm' and norm['metres_per_imported_unit']==.001
        and norm['origin_policy']=='preserve' and norm['up_axis']=='Z' and norm['handedness']=='right','Invalid normalization unit/frame contract')
    spec=dict(id=data['id'],declared_length_unit=data['source']['declared_length_unit'],source_frame=data['source']['frame'],
        asset_frame=norm['asset_frame'],source_up_axis=data['source']['up_axis'],source_to_asset_rotation=norm['source_to_asset_rotation'],
        expected_bbox_size_m=norm['expected_bbox_size_m'],bbox_tolerance_m=norm['bbox_tolerance_m'],
        deflection_m=norm['tessellation_linear_deflection_m'],padding_m=data['collision']['padding_m'])
    check_spec(spec)
    cad_low,cad_high=vector(norm['cad_bbox_min_m']),vector(norm['cad_bbox_max_m'])
    require(all(a<b for a,b in zip(cad_low,cad_high)),'Invalid CAD bbox extent')
    collider=data['collision'];require(collider['method']=='aabb_cover' and collider['basis']=='imported_shape_bbox','Unsupported collision approximation')
    center,size=vector(collider['center_m']),vector(collider['size_m']);padding=number(collider['padding_m'],nonnegative=True)
    require(all(v>0 for v in size) and all(abs(center[k]-(cad_low[k]+cad_high[k])/2)<1e-12
        and abs(size[k]-(cad_high[k]-cad_low[k]+2*padding))<1e-12 for k in range(3)),'Collision box does not preserve explicit CAD padding')
    require(all(center[k]-size[k]/2-1e-9<=point[k]<=center[k]+size[k]/2+1e-9 for point in points for k in range(3)),'Visual vertex escapes collision box')
    require(max(abs((cad_high[k]-cad_low[k])-spec['expected_bbox_size_m'][k]) for k in range(3))<=spec['bbox_tolerance_m'],'Unit dimension confirmation changed')
    return data

def publish_directory(staging,destination):
    """Linux atomic no-replace publish: never overwrite even an empty directory."""
    staging,destination=Path(staging),Path(destination)
    require(not destination.exists() and not destination.is_symlink(),'Output already exists')
    api=ctypes.CDLL(None,use_errno=True)
    rename=getattr(api,'renameat2',None);require(rename is not None,'Atomic no-replace directory publish requires Linux renameat2')
    rename.argtypes=[ctypes.c_int,ctypes.c_char_p,ctypes.c_int,ctypes.c_char_p,ctypes.c_uint];rename.restype=ctypes.c_int
    if rename(-100,os.fsencode(staging),-100,os.fsencode(destination),1)!=0:
        code=ctypes.get_errno()
        if code==errno.EEXIST:raise ValueError('Output already exists')
        raise OSError(code,os.strerror(code))

def rpy_matrix(rpy):
    r,p,y=vector(rpy);cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return [cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr,sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr,-sp,cp*sr,cp*cr]

def installation(value):
    require(type(value) is dict and set(value)=={'name','parent_frame','xyz_m','rpy_rad','rgba'},'Invalid installation fields')
    identifier(value['name']);require(value['parent_frame']=='world','MVP supports explicit world installation only')
    vector(value['xyz_m']);vector(value['rpy_rad']);rgba=vector(value['rgba'],4)
    require(all(0<=v<=1 for v in rgba),'Invalid installation RGBA')
    return deepcopy(value)
