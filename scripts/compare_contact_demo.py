"""Verify matched runs and optionally compose their actual GUI recordings side by side."""
import argparse
import json
from pathlib import Path
from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.workcells.contact_comparison import compare_runs


def compose(contact, control, directory):
    import cv2
    import numpy as np

    reports = [contact, control]
    videos = []
    sources = []
    leading = 1.0
    fps = 24
    end = contact["commanded_duration_sec"] + 2.0
    total = round((leading + end) * fps)
    for report in reports:
        source = ROOT / report["recording"]["directory"] / report["recording"]["video"]
        video = cv2.VideoCapture(str(source))
        if not video.isOpened() or video.get(cv2.CAP_PROP_FPS) != fps:
            raise ValueError("Expected a recorded 24fps GUI video")
        start = (
            report["motion_start_clock_sec"]
            - report["recording"]["video_start_clock_sec"]
        )
        offset = round((start - leading) * fps)
        if offset < 0 or offset + total > video.get(cv2.CAP_PROP_FRAME_COUNT):
            raise ValueError(
                "Source video does not cover the complete comparison; no frame padding is allowed"
            )
        video.set(cv2.CAP_PROP_POS_FRAMES, offset)
        videos.append(video)
        sources.append(
            dict(
                file=str(source.relative_to(ROOT)),
                first_frame=offset,
                motion_start_in_source_sec=start,
                source_run_id=report["run_id"],
            )
        )
    width, height = 1280, 800
    writer = cv2.VideoWriter(
        str(directory / "comparison.mp4"),
        cv2.VideoWriter_fourcc(*"avc1"),
        fps,
        (2 * width, height + 110),
    )
    if not writer.isOpened():
        raise RuntimeError("H.264 encoder unavailable")
    snapshots = {
        round((leading + t) * fps): name
        for t, name in [
            (0, "start"),
            (2.8, "interrupted"),
            (4.1, "outbound"),
            (8.8, "returned"),
        ]
    }
    try:
        for index in range(total):
            canvas = np.zeros((height + 110, 2 * width, 3), dtype=np.uint8)
            for panel, video in enumerate(videos):
                ok, frame = video.read()
                if not ok:
                    raise RuntimeError(
                        "Source recording ended before comparison completed"
                    )
                canvas[
                    60 : 60 + height, panel * width : (panel + 1) * width
                ] = cv2.resize(frame, (width, height))
            cv2.putText(
                canvas,
                "WITH OBSTACLE: BOTH ARMS STOP",
                (20, 38),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (100, 130, 255),
                2,
            )
            cv2.putText(
                canvas,
                "NO OBSTACLE: SAME FULL OUTBOUND + RETURN",
                (width + 20, 38),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (120, 255, 120),
                2,
            )
            elapsed = index / fps - leading
            label = f"Two independent runs | aligned at motion start | t={elapsed:.2f}s | outbound 0-4s, return 4-8s"
            cv2.putText(
                canvas,
                label,
                (20, height + 94),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (240, 240, 240),
                2,
            )
            writer.write(canvas)
            if index in snapshots:
                cv2.imwrite(str(directory / (snapshots[index] + ".png")), canvas)
    finally:
        writer.release()
        for video in videos:
            video.release()
    return dict(
        file="comparison.mp4",
        fps=fps,
        frames=total,
        duration_sec=total / fps,
        width_px=2 * width,
        height_px=height + 110,
        sources=sources,
        processing="Two independent captured runs; resize, labels, motion-start alignment only. No speed change, frame padding or synthetic robot/contact animation.",
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contact-report", type=Path, required=True)
    parser.add_argument("--control-report", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--summary-only", action="store_true")
    args = parser.parse_args()
    contact = json.loads(args.contact_report.read_text())
    control = json.loads(args.control_report.read_text())
    summary = compare_runs(contact, control)
    args.output_dir.mkdir(parents=True, exist_ok=False)
    if not args.summary_only:
        summary["recording"] = compose(contact, control, args.output_dir)
    (args.output_dir / "comparison.json").write_text(
        json.dumps(summary, indent=2) + "\n"
    )
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
