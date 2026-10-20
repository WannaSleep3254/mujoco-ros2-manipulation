"""Validate matched timing results and optionally compose actual GUI recordings."""
import argparse
import json
from pathlib import Path
from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.workcells.timing_comparison import compare_timing
from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.verification.reports import atomic_json


def compose(reports, directory):
    import cv2
    import numpy as np
    from PIL import Image, ImageDraw, ImageFont

    fps = 24
    lead = 0.5
    w, h = 900, 566
    title = 100
    size = (2 * w, h + title + 50)
    count = int((lead + reports[0]["comparison_duration_sec"]) * fps)
    font = ImageFont.truetype(
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 25
    )
    small = ImageFont.truetype(
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 20
    )
    videos = []
    sources = []
    for r in reports:
        meta = r["recording"]
        source = ROOT / meta["directory"] / meta["video"]
        v = cv2.VideoCapture(str(source))
        offset = round(
            (r["motion_start_clock_sec"] - meta["video_start_clock_sec"] - lead) * fps
        )
        if (
            meta["errors"]
            or meta["maximum_capture_gap_sec"] > 0.25
            or not v.isOpened()
            or v.get(cv2.CAP_PROP_FPS) != fps
            or offset < 0
            or offset + count > v.get(cv2.CAP_PROP_FRAME_COUNT)
        ):
            raise ValueError(
                "Recording does not cover comparison or has invalid capture timing"
            )
        v.set(cv2.CAP_PROP_POS_FRAMES, offset)
        videos.append(v)
        sources.append(
            dict(
                file=str(source.relative_to(ROOT)),
                sha256=file_sha256(source),
                first_frame=offset,
                run_id=r["run_id"],
            )
        )
    writer = cv2.VideoWriter(
        str(directory / "comparison.mp4"), cv2.VideoWriter_fourcc(*"avc1"), fps, size
    )
    if not writer.isOpened():
        raise RuntimeError("H264 encoder unavailable")
    snapshots = {
        round((lead + t) * fps): name
        for t, name in [
            (0, "start"),
            (4, "crossing"),
            (reports[1]["motion_duration_sec"] + 0.5, "timing-complete"),
            (reports[0]["motion_duration_sec"] + 0.5, "baseline-complete"),
        ]
    }
    try:
        for index in range(count):
            elapsed = index / fps - lead
            canvas = np.zeros((size[1], size[0], 3), dtype=np.uint8)
            for panel, v in enumerate(videos):
                ok, f = v.read()
                if not ok or f.shape[:2] != (900, 1440):
                    raise ValueError("Unexpected source frame")
                canvas[title : title + h, panel * w : (panel + 1) * w] = cv2.resize(
                    f[120:560, 400:1100], (w, h)
                )
            picture = Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
            draw = ImageDraw.Draw(picture)
            for panel, r in enumerate(reports):
                x = panel * w + 15
                heading = (
                    "기존 공동 회피: 관절 경로 변경"
                    if panel == 0
                    else f'타이밍 조정: FR5 출발 {r["departure_delay_sec"]:.1f}초 지연'
                )
                draw.text(
                    (x, 5),
                    heading,
                    font=font,
                    fill=(255, 205, 145) if panel == 0 else (145, 235, 205),
                )
                status = (
                    "목표 도달 완료"
                    if elapsed >= r["motion_duration_sec"]
                    else "기존 궤적 실행 중"
                    if panel == 0
                    else "원래 경로 유지 · 순서 조정"
                )
                draw.text((x, 40), status, font=small, fill=(240, 240, 240))
                gap = r["observed"]["minimum"]["distance_m"] * 1000
                draw.text(
                    (x, 69),
                    f'완료 {r["motion_duration_sec"]:.2f}s / 관측 최소 간격 {gap:.1f}mm',
                    font=small,
                    fill=(240, 240, 240),
                )
            draw.text(
                (15, size[1] - 40),
                f"같은 장면·시작·목표 | 실제 GUI · 원속도 | 명령 시작 기준 t={elapsed:.2f}s | 왼쪽 FR5 / 오른쪽 UR5e",
                font=small,
                fill=(235, 235, 235),
            )
            writer.write(cv2.cvtColor(np.asarray(picture), cv2.COLOR_RGB2BGR))
            if index in snapshots:
                picture.save(directory / (snapshots[index] + ".png"))
    finally:
        writer.release()
        for v in videos:
            v.release()
    return dict(
        video="comparison.mp4",
        fps=fps,
        frames=count,
        source_crop_xyxy=[400, 120, 1100, 560],
        kind="Actual independent GUI runs, aligned at command start; fixed crop/scale and editorial labels only",
        sources=sources,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--summary-only", action="store_true")
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    mode = "gui" if args.gui else "headless"
    paths = [
        ROOT / f"runtime/validation/timing_{name}_fr5_ur5e_{mode}.json"
        for name in ("baseline", "timing")
    ]
    result = compare_timing(paths, args.plan)
    out = args.output_dir.resolve()
    if not out.is_relative_to(ROOT / "runtime") or out.exists():
        parser.error("Output must be a new runtime/ directory")
    out.mkdir(parents=True)
    if not args.summary_only:
        result["composition"] = compose([json.loads(p.read_text()) for p in paths], out)
    atomic_json(out / "comparison.json", result)
    print(out / "comparison.json")


if __name__ == "__main__":
    raise SystemExit(main())
