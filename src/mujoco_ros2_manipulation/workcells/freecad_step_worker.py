"""Run only in an owned FreeCADCmd process with owned user/system config files."""
import json
import os
from pathlib import Path
import traceback
import FreeCAD as App
import Part

job_path=Path(os.environ['MANIPULATION_STEP_JOB'])
job=json.loads(job_path.read_text())
try:
    if job['operation']=='synthetic_fixture':
        shape=Part.makeBox(100,200,300,App.Vector(-50,-100,-150))
        shape.exportStep(job['output'])
        (job_path.parent/'fixture.json').write_text(json.dumps({'status':'passed','size_mm':[100,200,300],
            'bbox_min_mm':[-50,-100,-150],'bbox_max_mm':[50,100,150]})+'\n')
    elif job['operation']=='convert':
        shape=Part.read(job['input'])
        if shape.isNull() or not shape.isValid() or len(shape.Solids)!=1 or shape.Volume<=0:
            raise ValueError('Only one valid closed solid is supported')
        if len(shape.Faces)!=len(shape.Solids[0].Faces):
            raise ValueError('Loose assembly faces are not supported')
        vertices,faces=shape.tessellate(job['deflection_m']*1000.)
        if len(vertices)>250000 or len(faces)>500000:raise ValueError('Tessellation exceeds resource limits')
        bounds=shape.BoundBox
        result={'tessellation':{'valid':True,'solid_count':len(shape.Solids),
            'vertices_mm':[[v.x,v.y,v.z] for v in vertices],'triangles':[list(face) for face in faces],
            'bbox_min_mm':[bounds.XMin,bounds.YMin,bounds.ZMin],'bbox_max_mm':[bounds.XMax,bounds.YMax,bounds.ZMax]},
            'version':'.'.join(str(value) for value in App.Version()[:2]),'occt_version':str(Part.OCC_VERSION)}
        (job_path.parent/'converted.json').write_text(json.dumps(result,allow_nan=False,separators=(',',':'))+'\n')
    else:raise ValueError('Unsupported owned CAD operation')
except BaseException as error:
    (job_path.parent/'error.json').write_text(json.dumps({'type':type(error).__name__,'message':str(error),'traceback':traceback.format_exc()})+'\n')
    raise
