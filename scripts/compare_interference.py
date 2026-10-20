"""Validate four matched launches and compose actual GUI pixels on one time axis."""
import argparse
import json
from pathlib import Path

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.workcells.interference_comparison import (
    compare_reports,
    MODES,
)
from mujoco_ros2_manipulation.verification.reports import atomic_json


def compose(reports, directory):
    import cv2
    import numpy as np
    from PIL import Image, ImageDraw, ImageFont

    fps, leading = 24, 0.5
    span = reports[0]["comparison_duration_sec"]
    count = int((leading + span) * fps)
    panel_w, panel_h, title_h = 900, 558, 96
    size = (panel_w * 2, (panel_h + title_h) * 2 + 54)
    fontpath = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
    font = ImageFont.truetype(fontpath, 26)
    small = ImageFont.truetype(fontpath, 21)
    titles = [
        "1-A  FR5 단독 정상 동작",
        "1-B  UR5e 단독 정상 동작",
        "2  회피 없음: 실제 팔 간 접촉 → 양팔 정지",
        "3  공동 간섭 회피: 양팔 목표 도달",
    ]
    notes = [
        "상대 UR5e는 시작 자세 유지",
        "상대 FR5는 시작 자세 유지",
        "접촉 대상: FR5·UR5e 손목 (고정 장애물 아님)",
        "같은 시작·목표 / 상대 팔을 고려한 공동 계획",
    ]
    videos = []
    sources = []
    for report in reports:
        meta = report["recording"]
        if meta["errors"] or meta["maximum_capture_gap_sec"] > 0.25:
            raise ValueError("Recording has errors or a gap larger than 250ms")
        source = ROOT / meta["directory"] / meta["video"]
        video = cv2.VideoCapture(str(source))
        if not video.isOpened() or video.get(cv2.CAP_PROP_FPS) != fps:
            raise ValueError("Expected a 24fps owned GUI recording")
        offset = round(
            (report["motion_start_clock_sec"] - meta["video_start_clock_sec"] - leading)
            * fps
        )
        if offset < 0 or offset + count > video.get(cv2.CAP_PROP_FRAME_COUNT):
            raise ValueError(
                "Recording does not cover complete timeline; padding is not allowed"
            )
        video.set(cv2.CAP_PROP_POS_FRAMES, offset)
        videos.append(video)
        sources.append(
            dict(
                run_id=report["run_id"],
                video=str(source.relative_to(ROOT)),
                sha256=file_sha256(source),
                first_frame=offset,
            )
        )
    writer = cv2.VideoWriter(
        str(directory / "comparison.mp4"), cv2.VideoWriter_fourcc(*"avc1"), fps, size
    )
    if not writer.isOpened():
        raise RuntimeError("H264 encoder unavailable")
    collision = reports[2]
    hit = collision["guard_latch_clock_sec"] - collision["motion_start_clock_sec"]
    snapshots = {
        round((leading + t) * fps): name
        for t, name in [
            (0, "start"),
            (hit + 0.25, "contact"),
            (reports[0]["motion_duration_sec"] + 0.5, "normal-complete"),
            (reports[3]["motion_duration_sec"] + 0.5, "avoidance-complete"),
        ]
    }
    try:
        for index in range(count):
            elapsed = index / fps - leading
            canvas = np.zeros((size[1], size[0], 3), dtype=np.uint8)
            for panel, video in enumerate(videos):
                ok, frame = video.read()
                if not ok or frame.shape[:2] != (900, 1440):
                    raise RuntimeError(
                        "Source recording ended or camera dimensions changed"
                    )
                # Fixed editorial crop and scale only; never move, synthesize or mask robot pixels.
                crop = frame[190:500, 520:1020]
                x = (panel % 2) * panel_w
                y = (panel // 2) * (panel_h + title_h)
                canvas[
                    y + title_h : y + title_h + panel_h, x : x + panel_w
                ] = cv2.resize(crop, (panel_w, panel_h))
            picture = Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
            draw = ImageDraw.Draw(picture)
            for panel, report in enumerate(reports):
                x = (panel % 2) * panel_w
                y = (panel // 2) * (panel_h + title_h)
                color = (255, 150, 145) if panel == 2 else (150, 235, 200)
                draw.text((x + 16, y + 5), titles[panel], font=font, fill=color)
                if panel == 2 and elapsed >= hit:
                    note = f'접촉 정지 t={hit:.2f}s / 남은 명령 {report["remaining_motion_at_contact_sec"]:.2f}s'
                elif elapsed >= report["motion_duration_sec"]:
                    note = "목표 도달 완료 · 자세 유지" if panel != 2 else notes[panel]
                else:
                    note = notes[panel]
                draw.text((x + 16, y + 44), note, font=small, fill=(230, 230, 230))
            draw.text(
                (16, size[1] - 44),
                f"독립 실행 4개 · 실제 GUI 확대 · 같은 명령 시작 기준 t={elapsed:.2f}s · 원속도 · 왼쪽 FR5 / 오른쪽 UR5e",
                font=small,
                fill=(235, 235, 235),
            )
            pixels = cv2.cvtColor(np.asarray(picture), cv2.COLOR_RGB2BGR)
            writer.write(pixels)
            if index in snapshots:
                picture.save(directory / (snapshots[index] + ".png"))
    finally:
        writer.release()
        for video in videos:
            video.release()
    return dict(
        video="comparison.mp4",
        frames=count,
        fps=fps,
        duration_sec=count / fps,
        camera_crop_xyxy=[520, 190, 1020, 500],
        scale_to_panel=[panel_w, panel_h],
        source_size=[1440, 900],
        sources=sources,
        leading_sec=leading,
        kind="Four independent actual GUI recordings, aligned at command start; fixed crop/scale and editorial labels; no speed changes or padded tail",
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--reports", type=Path, nargs=4)
    parser.add_argument("--summary-only", action="store_true")
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    paths = args.reports or [
        ROOT
        / f'runtime/validation/interference_comparison_{mode}_fr5_ur5e_{"gui" if args.gui else "headless"}.json'
        for mode in MODES
    ]
    summary = compare_reports(paths)
    directory = args.output_dir.resolve()
    if not directory.is_relative_to(ROOT / "runtime") or directory.exists():
        parser.error("Output must be a new directory under runtime/")
    directory.mkdir(parents=True)
    reports = [json.loads(p.read_text()) for p in paths]
    reports.sort(key=lambda r: MODES.index(r["comparison_mode"]))
    if not args.summary_only:
        summary["composition"] = compose(reports, directory)
    atomic_json(directory / "comparison.json", summary)
    print(directory / "comparison.json")


if __name__ == "__main__":
    raise SystemExit(main())
