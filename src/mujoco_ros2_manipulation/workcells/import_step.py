"""Explicit local STEP import; no upload endpoint, no generated-model rebuild."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import resource
import shutil
import signal
import stat
import subprocess
import tempfile

if __package__:
    from .environment_assets import canonical,check_spec,normalize,publish_directory,require,sha,validate_bundle
else:
    from environment_assets import canonical,check_spec,normalize,publish_directory,require,sha,validate_bundle

MAX_STEP_BYTES=32*1024*1024
MAX_RESULT_BYTES=64*1024*1024

def _bounded_read(path,maximum):
    descriptor=os.open(path,os.O_RDONLY|os.O_NONBLOCK|getattr(os,'O_NOFOLLOW',0))
    try:
        info=os.fstat(descriptor)
        require(stat.S_ISREG(info.st_mode) and info.st_size<=maximum,'Local input exceeds byte limit or is not a regular file')
        with os.fdopen(descriptor,'rb',closefd=False) as stream:raw=stream.read(maximum+1)
        require(len(raw)<=maximum,'Local input exceeds byte limit')
        return raw
    finally:os.close(descriptor)

def _stop_owned(process):
    """Reap only our still-unreaped Popen child and its newly created group."""
    if process.poll() is not None:return
    try:os.killpg(process.pid,signal.SIGTERM)
    except ProcessLookupError:pass
    try:process.wait(timeout=3)
    except BaseException:
        # This is already cleanup: a repeated interrupt must not strand the
        # converter. The original wait exception is re-raised by the caller.
        try:os.killpg(process.pid,signal.SIGKILL)
        except ProcessLookupError:pass
        process.wait(timeout=3)

def _cleanup_after_error(process,error,directory):
    try:_stop_owned(process)
    except BaseException as cleanup:
        # Local diagnostic only; cleanup cannot replace the triggering error.
        try:(directory/'cleanup-error.json').write_bytes(canonical(dict(type=type(cleanup).__name__,message=str(cleanup)))+b'\n')
        except BaseException:pass

def _limits():
    resource.setrlimit(resource.RLIMIT_FSIZE,(MAX_RESULT_BYTES,MAX_RESULT_BYTES))
    resource.setrlimit(resource.RLIMIT_AS,(3*1024**3,3*1024**3))

def run_freecad(job,directory,*,executable='/usr/bin/freecadcmd',timeout=30.):
    """Every interrupted wait cleans up only this newly owned converter group."""
    require(type(timeout) in (int,float) and math.isfinite(timeout) and 0<timeout<=120,'Conversion timeout must be finite and in (0,120] seconds')
    directory=Path(directory);script=Path(__file__).with_name('freecad_step_worker.py')
    binary=Path(executable).resolve();require(binary.is_file(),'FreeCADCmd executable is unavailable')
    before=dict(name='FreeCAD',script_sha256=sha(_bounded_read(script,1024*1024)),executable_sha256=sha(_bounded_read(binary,256*1024*1024)))
    job_path=directory/'job.json';job_path.write_bytes(canonical(job)+b'\n')
    argv=[str(binary),'-u',str(directory/'user.cfg'),'-s',str(directory/'system.cfg'),str(script)]
    with (directory/'converter.log').open('wb') as log:
        process=subprocess.Popen(argv,cwd=directory,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,
            env=dict(os.environ,MANIPULATION_STEP_JOB=str(job_path),QT_QPA_PLATFORM='offscreen'),
            start_new_session=True,preexec_fn=_limits)
    try:
        code=process.wait(timeout=timeout)
    except BaseException as error:
        _cleanup_after_error(process,error,directory)
        raise
    require(code==0 and not (directory/'error.json').exists(),'Owned FreeCAD conversion failed')
    require(before['script_sha256']==sha(_bounded_read(script,1024*1024)) and before['executable_sha256']==sha(_bounded_read(binary,256*1024*1024)),
        'Converter script/executable changed during conversion')
    return before

def import_step(source,destination,spec,*,executable='/usr/bin/freecadcmd',timeout=30.):
    spec=check_spec(spec);source=Path(source).resolve();destination=Path(destination).absolute()
    require(source.is_file() and source.suffix.lower() in ('.step','.stp') and source.stat().st_size<=MAX_STEP_BYTES,'Expected a bounded local STEP file')
    require(not destination.exists() and not destination.is_symlink(),'Output already exists')
    original=_bounded_read(source,MAX_STEP_BYTES);require(original,'STEP exceeds byte limit')
    destination.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.step-job-',dir=destination.parent) as job_folder:
        work=Path(job_folder);copied=work/'source.step';copied.write_bytes(original)
        provenance=run_freecad(dict(operation='convert',input=str(copied),deflection_m=spec['deflection_m']),work,
            executable=executable,timeout=timeout)
        result_path=work/'converted.json';require(result_path.is_file() and result_path.stat().st_size<=MAX_RESULT_BYTES,'Missing/oversized tessellation result')
        raw=_bounded_read(result_path,MAX_RESULT_BYTES)
        result=json.loads(raw,parse_constant=lambda _:(_ for _ in ()).throw(ValueError('Nonfinite converter output')))
        provenance.update(version=result['version'],occt_version=result['occt_version'])
        manifest,mesh=normalize(original,result['tessellation'],spec,provenance)
        require(sha(_bounded_read(source,MAX_STEP_BYTES))==sha(original) and _bounded_read(copied,MAX_STEP_BYTES)==original,'STEP source changed during conversion')
        staging=work/'bundle';staging.mkdir(mode=0o700)
        (staging/'visual.stl').write_bytes(mesh);(staging/'asset.json').write_bytes(canonical(manifest)+b'\n')
        manifest_sha=sha(_bounded_read(staging/'asset.json',1024*1024));validate_bundle(staging,expected_sha256=manifest_sha)
        publish_directory(staging,destination)
    return dict(directory=str(destination),manifest_sha256=manifest_sha,source_sha256=sha(original),visual_sha256=manifest['visual']['sha256'])

def synthetic_fixture(destination,*,executable='/usr/bin/freecadcmd'):
    """Create an owned asymmetric/negative-origin STEP, never a user CAD file."""
    destination=Path(destination).absolute();require(not destination.exists(),'Synthetic STEP output already exists')
    destination.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.step-fixture-',dir=destination.parent) as directory:
        work=Path(directory);output=work/'synthetic_negative_box.step'
        run_freecad(dict(operation='synthetic_fixture',output=str(output)),work,executable=executable)
        require(output.is_file() and output.stat().st_size<MAX_STEP_BYTES,'Synthetic STEP creation failed')
        with destination.open('xb') as stream:stream.write(_bounded_read(output,MAX_STEP_BYTES))
    return destination

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--spec',type=Path,required=True);parser.add_argument('--freecad',default='/usr/bin/freecadcmd')
    args=parser.parse_args()
    print(json.dumps(import_step(args.source,args.output,json.loads(args.spec.read_text()),executable=args.freecad)))

if __name__=='__main__':main()
