"""Import exact-equivalence search A/B evidence and a separate GUI readiness check."""
import argparse
from datetime import date
import gzip
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
from statistics import median

from .data import json_text, read_json, validate
from ..workcells.interference import Spline
from ..workcells.timing import delayed_curve


def check_comparison(data):
    if data["status"] != "passed" or data["error"] is not None or not data["cases"]:
        raise ValueError("Only complete successful A/B comparisons can be published")
    if type(data["repeats"]) is not int or not 1 <= data["repeats"] <= 20:
        raise ValueError("Invalid repeat count")
    if [c["case_id"] for c in data["cases"]] != data["planned_case_ids"]:
        raise ValueError("Planned input cases were dropped, added or reordered")
    if len({c["case_id"] for c in data["cases"]}) != len(data["cases"]):
        raise ValueError("Duplicate comparison cases")
    for index,case in enumerate(data["cases"]):
        if case["status"] != "passed" or case["error"] is not None or len(case["samples"]) != data["repeats"]:
            raise ValueError("Failed, truncated or filtered comparison case")
        if [v["mode"] for v in case["warmup"]] != ["reference","optimized"]:
            raise ValueError("Missing warmups")
        reference=case["warmup"][0]["selection"]
        source=[Spline(**v) for v in case["input_curves"]]
        if len(source)!=2:
            raise ValueError("Expected a pair of input curves")
        for repeat,pair in enumerate(case["samples"]):
            order=["reference","optimized"] if (index+repeat)%2==0 else ["optimized","reference"]
            if pair["repeat"]!=repeat or pair["order"]!=order or [v["mode"] for v in pair["variants"]]!=order or pair.get("selection_and_clearance_identical") is not True:
                raise ValueError("Missing or reordered paired observations")
        entries=case["warmup"]+[v for p in case["samples"] for v in p["variants"]]
        for entry in entries:
            if entry["status"]!="passed" or entry["selection"]!=reference:
                raise ValueError("Selected spline, delay or sampled clearance changed")
            chosen=entry["selection"]
            if chosen["delayed_arm"] not in ("left","right"):
                raise ValueError("Unknown selected arm")
            expected=[delayed_curve(c,chosen["delay_sec"] if arm==chosen["delayed_arm"] else 0).record() for c,arm in zip(source,("left","right"))]
            if chosen["curves"]!=expected or not chosen["evaluation"]["complete"] or chosen["evaluation"]["minimum"]["distance_m"] < data["policy"]["min_inter_arm_distance_m"]+data["policy"]["tracking_reserve_m"]:
                raise ValueError("Selection changed geometry or failed planning clearance")
            diagnostics=entry["diagnostics"];profile=diagnostics["profiling"];candidates=diagnostics["candidates"]
            if (profile["optimized"]!=(entry["mode"]=="optimized") or profile["candidate_count"]!=len(candidates)
                    or profile["native_distance_calls"]!=sum(c["native_distance_calls"] for c in candidates)
                    or any(c["status"]!="completed" or c["native_distance_calls"]!=c["samples"] for c in candidates)):
                raise ValueError("Candidate/call accounting differs")
            times=[profile[k] for k in ("elapsed_wall_sec","candidate_wall_sec","native_distance_wall_sec","other_wall_sec")]
            if (not all(type(v) in (int,float) and math.isfinite(v) and v>=0 for v in times) or times[2]>times[1]
                    or abs(times[0]-times[1]-times[3])>1e-8
                    or abs(times[1]-sum(c["elapsed_wall_sec"] for c in candidates))>1e-8
                    or abs(times[2]-sum(c["native_distance_wall_sec"] for c in candidates))>1e-8):
                raise ValueError("Wall timing boundaries are inconsistent")
        for pair in case["samples"]:
            values={v["mode"]:v["diagnostics"]["profiling"]["native_distance_calls"] for v in pair["variants"]}
            if values["optimized"]>values["reference"]:
                raise ValueError("Optimization increased evaluator calls")
    return data


def check_gui(evidence):
    gui=evidence["gui"]
    run=gui["verification"]
    if (run["status"]!="passed" or not run["shutdown_clean"] or not run["physics_checks_passed"]
            or run["physics"]["contact_steps"]!=0 or not run["mujoco_gpu_process_seen"] or not run["rviz_gpu_process_seen"]
            or len(run["joints"])!=12 or run["clock_publishers"]!=1 or run["joint_state_publishers"]!=1
            or not run["collision_goal_rejected"] or not run["active_cancellation_stops_both_arms"]
            or run["model_manifest_sha256"]!=evidence["comparison"]["model_manifest_sha256"]):
        raise ValueError("GUI readiness lacks shared-state, GPU, motion, physical or clean-shutdown evidence")
    images=run["screenshots"]["images"]
    if {r["application"] for r in images}!={"ros2_control_node","rviz2"}:
        raise ValueError("Missing owned GUI captures")
    return run


def search_report(evidence,identifier,published,asset_hashes):
    data=check_comparison(evidence["comparison"])
    gui=check_gui(evidence)
    image_hashes={r["application"]:r["sha256"] for r in gui["screenshots"]["images"]}
    if asset_hashes["mujoco-gui.png"]!=image_hashes["ros2_control_node"] or asset_hashes["rviz-gui.png"]!=image_hashes["rviz2"]:
        raise ValueError("Published screenshots differ from the GUI verification source")
    groups=[];series={};findings=[]
    labels=dict(reference="기존 두 순서 탐색",optimized="중복·상한 제거")
    for case in data["cases"]:
        for mode in ("reference","optimized"):
            key=case["case_id"]+"-"+mode
            samples=[next(v for v in p["variants"] if v["mode"]==mode) for p in case["samples"]]
            series[key]=samples
            groups.append(dict(id=key,label=case["case_id"]+" / "+labels[mode],kind="offline",unique_cases=1,
                observations=len(samples),successes=len(samples),failures=0,seed_policy="고정 spline 재생; 모드별 워밍업 1회 제외·반복 순서 교대",
                description="같은 모델·정책·시작·입력에서 선택 팔/지연/전체 spline/선택 경로 거리 평가가 정확히 일치해야 통과."))
        a=series[case["case_id"]+"-reference"];b=series[case["case_id"]+"-optimized"]
        stats=lambda values,key:median(v["diagnostics"]["profiling"][key] for v in values)
        findings.append(f"{case['case_id']}: 탐색 중앙값 {stats(a,'elapsed_wall_sec'):.3f}→{stats(b,'elapsed_wall_sec'):.3f}wall초, 거리 호출 {int(stats(a,'native_distance_calls'))}→{int(stats(b,'native_distance_calls'))}회.")
    metrics=[]
    specifications=[("search-median","탐색 시간 중앙값","s","wall","median",lambda e:e["diagnostics"]["profiling"]["elapsed_wall_sec"],median),
        ("search-max","탐색 시간 최댓값","s","wall","max",lambda e:e["diagnostics"]["profiling"]["elapsed_wall_sec"],max),
        ("native-calls","네이티브 거리 호출","count","not_applicable","median",lambda e:e["diagnostics"]["profiling"]["native_distance_calls"],median),
        ("candidates","실제 평가 후보","count","not_applicable","median",lambda e:e["diagnostics"]["profiling"]["candidate_count"],median),
        ("native-wall","네이티브 거리 계산 시간","s","wall","median",lambda e:e["diagnostics"]["profiling"]["native_distance_wall_sec"],median),
        ("selected-delay","선택된 팔의 지연","s","simulation","median",lambda e:e["selection"]["delay_sec"],median),
        ("clearance","선택 경로 최소 간격","mm","not_applicable","min",lambda e:e["selection"]["evaluation"]["minimum"]["distance_m"]*1000,min)]
    for key,label,unit,clock,stat,getter,reduce in specifications:
        metrics.append(dict(id=key,label=label,unit=unit,clock=clock,scope="같은 고정 입력의 오프라인 출발 탐색",
            values=[dict(group_id=k,value=reduce(getter(v) for v in values),sample_count=len(values),statistic=stat) for k,values in series.items()],
            note="모델 로딩·ROS 왕복·예약 대기·물리 실행 제외. native 시간은 전체 탐색 시간의 일부이며 따로 더하지 않음."))
    groups.append(dict(id="gui-readiness",label="현재 PC GUI 통합 점검",kind="gui",unique_cases=1,observations=1,successes=1,failures=0,
        seed_policy="별도 실제 GUI 실행 1회",description="MuJoCo·RViz GPU 렌더링, 12관절/공유 시계, 계획·충돌 거부·양팔 취소·정상 종료."))
    metrics.append(dict(id="gui-holding-rtf",label="GUI 정지 유지 구간 real-time factor",unit="ratio",clock="not_applicable",
        scope=f"동작 검증 후 {gui['performance']['requested_wall_sec']}wall초 paced stationary holding",values=[dict(group_id="gui-readiness",value=gui["performance"]["real_time_factor"],sample_count=1,statistic="single")],
        note="움직임 중 전체 성능·최대 물리 처리량·GUI FPS 측정이 아님."))
    findings.insert(0,f"고정 입력 {len(data['cases'])}개 × 두 방식 × {data['repeats']}회. 선택 팔·지연·전체 spline·거리 평가가 모든 반복에서 정확히 같음.")
    findings.append("별도 GUI 실제 실행: MuJoCo·RViz GPU 사용, 12관절·공유 시계/상태·계획·충돌 거부·양팔 취소·접촉 0·정상 종료 확인.")
    return dict(schema_version=1,report_id=identifier,published_date=published,title="출발 탐색 최적화 A/B와 시뮬레이터 GUI 점검",status="preliminary",
        overview="입력·검사 해상도를 보존하고 공유 0초 평가와 최선 지연 상한으로 계산을 줄인 비교. 실제 GUI 제작 상태를 별도 실행으로 확인했습니다.",
        findings=findings,limitations=["같은 구현의 optimize=False 참조 경로와 optimize=True를 비교. 과거 리포트의 다른 실행 시간과 직접 속도 향상을 비교하지 않음.",
            "고정 입력·소규모 반복이며 OS 스케줄링·배경 부하를 고정하지 않음. p95/p99·통계적 신뢰구간·전체 작업 공간 성공률을 제시하지 않음.",
            "절감한 탐색 시간과 지령→실제 이동 전체 지연은 구분. 선택 경로의 최종 MuJoCo/MoveIt 안전검사와 네이티브 접촉 감시는 계속 수행.",
            "GUI 사진은 실제 소유 프로세스의 X11 화면. 전용 통합 조작 화면은 미구현이며 현재 조작은 MuJoCo·RViz·CLI로 구성.",
            "GUI 점검과 오프라인 탐색은 별도 실행. 정지 유지 RTF는 GUI FPS나 최대 처리량이 아님. 물리·경로 계산은 현재 CPU 구성, GPU 확인은 GUI 렌더링 범위.",
            "그리퍼·공구 TCP·CAD·양팔 Pose/IK·실물 연결과 대규모 교차/장애물 평가가 남아 있음."],
        provenance=dict(repository="https://github.com/WannaSleep3254/mujoco-ros2-manipulation",commit=data["code_commit"],workcell=data["workcell"],models=["FR5","UR5e"],scenario="exact_departure_search_ab",input_kind="fixed_joint_targets"),
        environment=dict(os=data["runtime"]["os"],cpu=data["host"]["cpu"],python=data["runtime"]["python"],ros="ROS 2 Humble (별도 GUI 검증)",native_mujoco=data["evaluator"]["engine"],gui_run=gui["run_id"]),
        sample_groups=groups,stages=[dict(id="search",label="고정 입력 출발 탐색 A/B",clock="wall",coverage="measured",note="입력 로딩·ROS·동작과 구분; 정확한 결과 일치와 호출 수 확인"),
            dict(id="gui",label="별도 GUI 통합 점검",clock="wall",coverage="measured",note="GPU 렌더링·12관절 제어·계획·취소·종료와 실제 화면")],metrics=metrics,
        assets=[dict(path=name,label=label,role=role,sha256=asset_hashes[name]) for name,label,role in (("evidence.json.gz","전체 A/B 반복·후보·GUI 검증 원본 (gzip JSON)","data"),("mujoco-gui.png","실제 MuJoCo 양팔 장면","figure"),("rviz-gui.png","실제 RViz 양팔 계획 화면","figure"))],
        references=[dict(label="평가 구현 커밋",url="https://github.com/WannaSleep3254/mujoco-ros2-manipulation/commit/"+data["code_commit"])])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--comparison",type=Path);parser.add_argument("--gui-run",type=Path)
    parser.add_argument("--report-id");parser.add_argument("--date",default=date.today().isoformat())
    parser.add_argument("--check",type=Path)
    args=parser.parse_args()
    if args.check:
        folder=args.check;report=read_json(folder/"report.json");raw=(folder/"evidence.json.gz").read_bytes()
        hashes={a["path"]:hashlib.sha256((folder/a["path"]).read_bytes()).hexdigest() for a in report["assets"]}
        if search_report(json.loads(gzip.decompress(raw)),report["report_id"],report["published_date"],hashes)!=report:
            parser.error("Report differs from the preserved search comparison/GUI evidence")
        validate(report,folder);print("Search optimization and GUI report match preserved evidence");return
    if not args.comparison or not args.gui_run or not args.report_id or not re.fullmatch(r"[a-z0-9]+(?:-[a-z0-9]+)*",args.report_id) or len(args.report_id)>100:
        parser.error("Supply comparison, GUI run and a plain new report ID")
    date.fromisoformat(args.date)
    evidence=dict(comparison=read_json(args.comparison),gui=dict(verification=read_json(args.gui_run)))
    check_comparison(evidence["comparison"]);run=check_gui(evidence)
    from ..paths import workspace_root
    root=workspace_root();output=root/"docs/reports"/args.report_id
    if output.exists():parser.error("Report ID already exists; preserve previous publications")
    raw=gzip.compress(json_text(evidence).encode(),mtime=0)
    images={"mujoco-gui.png":next(i for i in run["screenshots"]["images"] if i["application"]=="ros2_control_node"),
            "rviz-gui.png":next(i for i in run["screenshots"]["images"] if i["application"]=="rviz2")}
    hashes={"evidence.json.gz":hashlib.sha256(raw).hexdigest()}
    for name,item in images.items():
        source_image=(root/item["file"]).resolve()
        if not source_image.is_relative_to((root/"runtime").resolve()):
            parser.error("GUI image source must be under runtime/")
        if hashlib.sha256(source_image.read_bytes()).hexdigest()!=item["sha256"]:
            parser.error("GUI screenshot differs from its verification hash")
        hashes[name]=item["sha256"]
    report=search_report(evidence,args.report_id,args.date,hashes);validate(report,output,check_assets=False)
    output.mkdir();(output/"evidence.json.gz").write_bytes(raw)
    for name,item in images.items():shutil.copyfile(root/item["file"],output/name)
    (output/"report.json").write_text(json_text(report));validate(report,output);print(output)


if __name__=="__main__":main()
