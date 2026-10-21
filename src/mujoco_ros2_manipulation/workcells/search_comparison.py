"""Compare full two-arm grid search with safe incumbent pruning on frozen inputs."""
import argparse
from copy import deepcopy
from datetime import datetime, timezone
import gzip
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import uuid

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.verification.diagnostics import host_info
from mujoco_ros2_manipulation.verification.reports import atomic_json
from .config import load_workcell
from .distance import DistanceEvaluator
from .integrity import check_workcell
from .interference import Spline
from .interference_comparison import load_bundle
from .timing import choose_delay, load_policy


MODES = ("reference", "optimized")


def selected(result):
    return deepcopy({k:result[k] for k in ("delayed_arm", "delay_sec", "curves", "evaluation")})


def compare_case(curves, initial, cell, evaluator, policy, repeats, *, record=None, progress=None, case_index=0):
    if type(repeats) is not int or not 1 <= repeats <= 20:
        raise ValueError("Comparison repeats must be 1..20")
    data = {} if record is None else record
    data.update(status="running", initial_rad=deepcopy(initial), input_curves=[deepcopy(c.record()) for c in curves],
                warmup=[], samples=[], error=None)
    save = progress or (lambda: None)

    def run(mode, target):
        entry=dict(mode=mode,status="running",diagnostics={})
        target.append(entry)
        try:
            result=choose_delay(curves,initial,cell,evaluator,policy,optimize=mode=="optimized",diagnostics=entry["diagnostics"])
            entry.update(status="passed",selection=selected(result))
        except BaseException as error:
            entry.update(status="failed",error=dict(type=type(error).__name__,message=str(error)))
            raise
        finally:
            save()
        return entry

    try:
        for mode in MODES:
            run(mode,data["warmup"])
        expected=data["warmup"][0]["selection"]
        if data["warmup"][1]["selection"] != expected:
            raise ValueError("Warmup changed the selected curve, delay or clearance evaluation")
        for repeat in range(repeats):
            order=list(MODES if (case_index+repeat)%2==0 else reversed(MODES))
            pair=dict(repeat=repeat,order=order,variants=[])
            data["samples"].append(pair)
            entries={mode:run(mode,pair["variants"]) for mode in order}
            if any(entries[mode]["selection"] != expected for mode in MODES):
                raise ValueError("Optimization changed the exact selected curve, delay or sampled clearance")
            if entries["optimized"]["diagnostics"]["profiling"]["native_distance_calls"] > entries["reference"]["diagnostics"]["profiling"]["native_distance_calls"]:
                raise ValueError("Optimization increased native distance calls")
            pair["selection_and_clearance_identical"]=True
            save()
        if [c.record() for c in curves] != data["input_curves"]:
            raise ValueError("Comparison mutated source geometry")
        data["status"]="passed"
    except BaseException as error:
        data.update(status="failed",error=dict(type=type(error).__name__,message=str(error)))
        raise
    finally:
        save()
    return data


def published_cases(path, cell, evaluator, policy):
    from mujoco_ros2_manipulation.reporting.data import read_json
    from mujoco_ros2_manipulation.reporting.random_motion import checked_requests, checked_fixed_search
    report=read_json(path)
    asset=next(a for a in report["assets"] if a["path"]=="evidence.json.gz")
    evidence_path=path.parent/asset["path"]
    if file_sha256(evidence_path) != asset["sha256"]:
        raise ValueError("Published evidence hash differs")
    evidence=json.loads(gzip.decompress(evidence_path.read_bytes()))
    checked_requests(evidence)
    checked_fixed_search(evidence)
    if (evidence["verification"]["model_manifest_sha256"] != file_sha256(cell.output/"source.json")
            or evidence["session"]["evaluator"] != evaluator.metadata()
            or evidence["session"]["policy"] != policy):
        raise ValueError("Frozen comparison requires the exact source model, native library and policy")
    requests={r["request_id"]:r for r in evidence["requests"]}
    cases=[]
    for row in evidence["benchmark"]["rows"]:
        if row["strategy"] != "departure_timing":
            continue
        request=requests[row["request_id"]]
        if request["status"] != "succeeded" or request["departure_search"]["delay_sec"] != 0:
            raise ValueError("Random source requires successful zero-delay source curves")
        cases.append(("random-"+row["case_id"],request["initial_state_rad"],[Spline(**c) for c in request["trajectories"]]))
    bundle=evidence["fixed_search"]["bundle"]
    cases.append(("crossing",bundle["initial_rad"],[Spline(**bundle["curves"][name]) for name in ("left","right")]))
    return cases,dict(kind="published verified immutable source",file=str(path),sha256=file_sha256(path),evidence_sha256=asset["sha256"])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    source=parser.add_mutually_exclusive_group()
    source.add_argument("--published-report",type=Path)
    source.add_argument("--bundle",type=Path)
    parser.add_argument("--case",choices=("all","crossing","random"),default="all")
    parser.add_argument("--repeats",type=int,default=5)
    parser.add_argument("--output",type=Path)
    args=parser.parse_args()
    if not 1 <= args.repeats <= 20:
        parser.error("--repeats must be 1..20")
    if args.bundle and args.case=="random":
        parser.error("--bundle supplies the crossing case only")
    output=(args.output or ROOT/"runtime/validation/search-comparison"/uuid.uuid4().hex/"result.json").resolve()
    if output.exists() or not output.is_relative_to((ROOT/"runtime").resolve()):
        parser.error("Output must be a new runtime/ file")
    output.parent.mkdir(parents=True,exist_ok=True)
    data=dict(schema_version=1,status="running",started_utc=datetime.now(timezone.utc).isoformat(),
              code_commit=subprocess.check_output(["git","-C",str(ROOT),"rev-parse","HEAD"],text=True).strip(),
              repeats=args.repeats,cases=[],error=None,measurement="Offline search only; one unscored warmup per mode/case, alternating measured order; no simulator/RPC/execution time")
    def save(): atomic_json(output,data)
    save()
    try:
        cell=load_workcell()
        check_workcell(cell)
        _,policy=load_policy(cell)
        with DistanceEvaluator(cell) as evaluator:
            if args.bundle:
                bundle,curves=load_bundle(args.bundle,cell)
                cases=[("crossing",bundle["initial_rad"],[curves["left"],curves["right"]])]
                provenance=dict(kind="fresh verified workcell bundle",file=str(args.bundle),sha256=file_sha256(args.bundle))
            else:
                path=args.published_report or ROOT/"docs/reports/2026-10-21-random-motion/report.json"
                cases,provenance=published_cases(path,cell,evaluator,policy)
            if args.case != "all":
                cases=[c for c in cases if (c[0]=="crossing")== (args.case=="crossing")]
            if not cases:
                raise ValueError("No comparison inputs selected")
            data.update(source=provenance,workcell=cell.id,model_manifest_sha256=file_sha256(cell.output/"source.json"),
                policy=policy,evaluator=evaluator.metadata(),host=host_info(),
                runtime=dict(python=platform.python_version(),os=platform.freedesktop_os_release().get("PRETTY_NAME",platform.platform())),
                implementation_sha256={name:file_sha256(Path(__file__).parent/name) for name in ("timing.py","interference.py","path_quality.py","distance.py","search_comparison.py")})
            for index,(identifier,initial,curves) in enumerate(cases):
                record=dict(case_id=identifier)
                data["cases"].append(record)
                save()
                compare_case(curves,initial,cell,evaluator,policy,args.repeats,record=record,progress=save,case_index=index)
            data["status"]="passed"
    except BaseException as error:
        data.update(status="failed",error=dict(type=type(error).__name__,message=str(error)))
        raise
    finally:
        data["completed_utc"]=datetime.now(timezone.utc).isoformat()
        save()
        print(str(output),flush=True)


if __name__=="__main__":
    main()
