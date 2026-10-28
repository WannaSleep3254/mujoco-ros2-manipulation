"""One fixed, owned FR5 simulation job with atomic latest observations.

The loopback demo owner supplies paths and identities at process creation.
They are never accepted through a browser goal or arbitrary path API.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import signal
import threading

from ..verification.reports import atomic_json
from .fr5_model import write_model
from .fr5_run import FR5Prototype


ENGINE = "MuJoCo 3.14.0"
MAX_LATEST_BYTES = 128 * 1024
MAX_RESULT_BYTES = 8 * 1024 * 1024
SOURCE_FILES = (
    "grasping/fr5_model.py", "grasping/fr5_run.py", "grasping/contract.py",
    "grasping/display_export.py", "grasping/live_worker.py", "grasping/demo_server.py",
    "grasping/static/demo.html", "grasping/static/demo.js", "grasping/static/demo.css",
    "operator_console/display_contract.py", "operator_console/scene_view.py",
    "operator_console/view_binding.py", "operator_console/recorded_server.py",
    "operator_console/scene_host.py", "operator_console/recording.py",
    "operator_console/static/viewer.js", "operator_console/static/viewer_contract.mjs",
    "operator_console/static/vendor/vendor-manifest.json",
)


def source_documents():
    root = Path(__file__).resolve().parents[1]
    documents = {name: (root/name).read_bytes() for name in SOURCE_FILES}
    if any(len(raw) > 512*1024 for raw in documents.values()) or sum(map(len, documents.values())) > 2*1024*1024:
        raise ValueError("Demo implementation source exceeds its bounded snapshot")
    return documents


def source_digests(documents):
    return {name: hashlib.sha256(raw).hexdigest() for name, raw in documents.items()}


def preserve_sources(directory):
    pin = directory/"source-pin.json"
    if pin.is_symlink() or pin.stat().st_size > 16*1024:
        raise ValueError("Invalid bounded demo source pin")
    expected = json.loads(pin.read_text())
    if type(expected) is not dict or set(expected) != set(SOURCE_FILES):
        raise ValueError("Missing fixed demo implementation source pin")
    documents = source_documents()
    if source_digests(documents) != expected:
        raise ValueError("Demo implementation changed before worker initialization")
    output = directory/"implementation_sources"
    output.mkdir(exist_ok=False)
    for name, raw in documents.items():
        path = output/name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(raw)
    atomic_json(output/"manifest.json", dict(schema_version=1, files_sha256=expected,
        note="Exact allowlisted implementation sources at this job's initialization; no credentials or runtime token"))
    return expected


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True)
    parser.add_argument("--job-directory", required=True)
    parser.add_argument("--job-id", required=True)
    parser.add_argument("--generation", type=int, required=True)
    parser.add_argument("--binding-id", required=True)
    parser.add_argument("--model-sha256", required=True)
    parser.add_argument("--manifest-sha256", required=True)
    args = parser.parse_args(argv)
    if (not re.fullmatch(r"[0-9a-f]{32}", args.job_id) or not 1 <= args.generation <= 32
            or any(not re.fullmatch(r"[0-9a-f]{64}", value)
                   for value in (args.binding_id, args.model_sha256, args.manifest_sha256))):
        parser.error("Expected the owner's fixed job/generation/model identities")
    directory = Path(args.job_directory).resolve()
    if not directory.is_dir() or directory.is_symlink():
        parser.error("Expected an existing owned job directory")
    cancelled = threading.Event()
    previous = signal.signal(signal.SIGINT, lambda *_: cancelled.set())
    sequence = 0
    prototype = None
    identity = dict(schema_version=1, job_id=args.job_id, generation=args.generation,
        binding_id=args.binding_id, model_sha256=args.model_sha256,
        manifest_sha256=args.manifest_sha256, engine=ENGINE)

    def publish(payload, *, terminal=None):
        nonlocal sequence
        sequence += 1
        if sequence > 5000:
            raise RuntimeError("Live observation bound exceeded")
        frame = payload["frame"]
        value = dict(identity, sequence=sequence, observed_monotonic_ns=frame["observed_monotonic_ns"],
            frame=frame, task=payload["task"], integrated_steps=payload["integrated_steps"],
            terminal_outcome=terminal)
        if len(json.dumps(value, allow_nan=False).encode()) > MAX_LATEST_BYTES:
            raise RuntimeError("Live observation exceeds its byte bound")
        atomic_json(directory/"latest.json", value)

    try:
        implementation_sha = preserve_sources(directory)
        import mujoco
        if "MuJoCo "+mujoco.__version__ != ENGINE or mujoco.mj_versionString() != "3.14.0":
            raise ValueError("The fixed demo requires its tested MuJoCo 3.14.0 engine")
        run = directory/"run"
        manifest = write_model(args.source, run)
        if manifest["mjcf_sha256"] != args.model_sha256 or manifest["manifest_sha256"] != args.manifest_sha256:
            raise ValueError("Source/model changed before the owned job started")
        prototype = FR5Prototype(run, manifest, observer=publish, cancel=cancelled.is_set, realtime=True)
        result = prototype.run()
        result["implementation_sha256"] = implementation_sha
        result["implementation_source_snapshot"] = "../implementation_sources"
        result["live_job"] = identity
        if len(json.dumps(result, allow_nan=False).encode()) > MAX_RESULT_BYTES:
            raise RuntimeError("Live result exceeds its byte bound")
        atomic_json(run/"result.json", result)
        if result.get("final") is not None:
            publish(dict(frame=result["final"], task=result["task"],
                         integrated_steps=result["physics"]["integrated_steps"]), terminal=result["outcome"])
        return 0 if result["outcome"] == "succeeded" else 130 if result["outcome"] == "cancelled" else 1
    except (Exception, KeyboardInterrupt) as error:
        # A killed worker may leave only latest.json and stdout.log. When Python
        # is still executing, retain its initialization/publication failure too.
        failure = dict(identity, error=dict(type=type(error).__name__, message=str(error)[:2048]),
            actual_steps=0 if prototype is None else prototype.steps,
            physical_stop_confirmed=False, simulation_stop_observed=False,
            note="Worker error is not evidence of physical stop or a successful grasp")
        atomic_json(directory/"worker_error.json", failure)
        return 1
    finally:
        signal.signal(signal.SIGINT, previous)


if __name__ == "__main__":
    raise SystemExit(main())
