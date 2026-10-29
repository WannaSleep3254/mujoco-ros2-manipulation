"""One bounded read-only validator; ROS and mutation remain on the owner thread."""
from pathlib import Path
import json
import threading
import os
import signal
import subprocess
import sys
import uuid
import re
import time
from .ros_observation import ObservationUnavailable


class AcceptedObservationRecorder:
    """Owner-only bounded raw/receipt logs, independent of physical eligibility."""
    def __init__(self, frames, receipts, *, max_frame_bytes, max_receipt_bytes, clock_ns=time.monotonic_ns):
        self.frames, self.receipts, self.clock_ns = frames, receipts, clock_ns
        self.max_frame_bytes, self.max_receipt_bytes = max_frame_bytes, max_receipt_bytes
        self.frame_bytes = self.receipt_bytes = 0
        self.last_context = None

    def record(self, value, *, received_ns, eligible_snapshot):
        raw = (json.dumps(value, separators=(",", ":"), allow_nan=False)+"\n").encode()
        self.frame_bytes += len(raw)
        if self.frame_bytes > self.max_frame_bytes:
            raise ValueError("native observation record limit exceeded")
        self.frames.write(raw)
        self.last_context = dict(native_sequence=value["sequence"],
            capture_monotonic_ns=int(value["capture_monotonic_ns"]), received_monotonic_ns=received_ns)
        receipt = dict(**self.last_context, validated_monotonic_ns=self.clock_ns(), eligible=False,
            eligibility_scope="observation age/clock only; physical guard/phase checked separately", physical_authority_claimed=False)
        try:
            observation = eligible_snapshot()
            receipt["eligible"] = True
            return observation
        except ObservationUnavailable as error:
            receipt["eligibility_error"] = str(error)[:256]
            return None
        finally:
            receipt["checked_monotonic_ns"] = self.clock_ns()
            trace = (json.dumps(receipt, separators=(",", ":"), allow_nan=False)+"\n").encode()
            self.receipt_bytes += len(trace)
            if self.receipt_bytes > self.max_receipt_bytes:
                raise ValueError("native receipt trace limit exceeded")
            self.receipts.write(trace)


def file_stats(paths):
    """Bind paths, symlinks and their targets without re-reading large files."""
    paths = tuple(str(Path(path).absolute()) for path in paths)
    if not paths or len(paths) > 2048 or len(set(paths)) != len(paths):
        raise ValueError("admission file inventory must be distinct and bounded")
    def identity(value):
        return [value.st_dev, value.st_ino, value.st_mode, value.st_size, value.st_mtime_ns, value.st_ctime_ns]
    return {path: [identity(Path(path).lstat()), identity(Path(path).stat())] for path in paths}


def _read_json(path, maximum=1024*1024):
    path = Path(path)
    if path.is_symlink() or not path.is_file() or path.stat().st_size > maximum:
        raise ValueError("read-only validation JSON is missing, linked or oversized")
    raw = path.read_bytes()
    if len(raw) > maximum:
        raise ValueError("read-only validation JSON exceeds its bound")
    value = json.loads(raw)
    if type(value) is not dict:
        raise ValueError("read-only validation JSON must be an object")
    return value


def _process_identity(pid):
    try:
        values = Path(f"/proc/{pid}/stat").read_text().rsplit(")",1)[1].split()
        return dict(pid=pid, starttime=int(values[19]))
    except (OSError,ValueError,IndexError):
        return None


def _member_identity(pid):
    try:
        values=Path(f"/proc/{pid}/stat").read_text().rsplit(")",1)[1].split()
        return dict(pid=pid,starttime=int(values[19]),ppid=int(values[1]),pgrp=int(values[2]),session=int(values[3]))
    except (OSError,ValueError,IndexError):
        return None


def _same_member(a,b):
    return a is not None and b is not None and all(a[k]==b[k] for k in ("pid","starttime","pgrp","session"))


def _group_gone(pgid):
    try:
        os.killpg(pgid,0)
        return False
    except ProcessLookupError:
        return True


class ReadOnlyAdmissionWorker:
    """One fixed read-only subprocess/session; Git children share its owner group.

    The ROS owner pumps while waiting. A cancelled/timed-out generation is
    permanently abandoned. No late proof can authorize work or another check.
    This subprocess owns no ROS objects and writes only its bounded private IPC.
    """
    def __init__(self, config, directory, *, clock=time.monotonic):
        self._clock = clock
        self.directory = Path(directory)
        self.directory.mkdir(mode=0o700)
        self._token = uuid.uuid4().hex
        self._generation = 0
        self._pending = None
        self._abandoned = self._closed = self._forced = False
        self._result_consumed = False
        self._group_retired=False
        self._ownership_error=None
        self._last_member_scan=float("-inf")
        body = dict(config, token=self._token, parent_identity=_process_identity(os.getpid()))
        raw = json.dumps(body,allow_nan=False).encode()
        if len(raw)>1024*1024:raise ValueError("read-only validation configuration exceeds its bound")
        configuration = self.directory/"config.json"
        configuration.write_bytes(raw)
        self.process = subprocess.Popen([sys.executable,"-m",__name__,"--identity-worker",str(configuration)],
            stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True,
            env=dict(os.environ,GIT_OPTIONAL_LOCKS="0"))
        self._leader=_member_identity(self.process.pid)
        self._members={} if self._leader is None else {self.process.pid:self._leader}

    def _remember_members(self, *, force=False):
        now=time.monotonic()
        if not force and now-self._last_member_scan < .05:
            return
        self._last_member_scan=now
        current=_member_identity(self.process.pid)
        if (not _same_member(current,self._leader) or current["ppid"]!=os.getpid()
                or current["pgrp"]!=self.process.pid or current["session"]!=self.process.pid):
            return
        for entry in Path("/proc").iterdir():
            if entry.name.isdigit():
                member=_member_identity(int(entry.name))
                if member is not None and member["pgrp"]==self.process.pid and member["session"]==self.process.pid:
                    self._members[member["pid"]]=member

    def _owned_group_gone(self):
        if self._group_retired:return True
        if _group_gone(self.process.pid):self._group_retired=True;return True
        return False

    def _signal_group(self,sig):
        if self._owned_group_gone():return
        self._remember_members(force=True)
        owned=any(_same_member(_member_identity(pid),identity) for pid,identity in self._members.items())
        if not owned:
            self._ownership_error="validator group identity no longer has a proven original member; signal refused"
            return
        try:os.killpg(self.process.pid,sig)
        except ProcessLookupError:self._group_retired=True

    def _interrupt(self):
        self._signal_group(signal.SIGINT)

    def check(self, *, pump, abort, timeout_sec=6.):
        if type(timeout_sec) not in (int,float) or not 0<timeout_sec<=30:
            raise ValueError("read-only admission timeout must be bounded")
        abort()
        if self._closed or self._abandoned or self._pending is not None:
            raise ValueError("read-only admission worker is closed, abandoned or busy")
        self._generation += 1
        if self._generation>128:raise ValueError("read-only admission request budget exceeded")
        generation=self._generation
        self._pending=generation
        self._result_consumed=False
        result_path=self.directory/f"proof-{generation:03}.json"
        deadline=self._clock()+timeout_sec
        try:
            request=json.dumps(dict(token=self._token,generation=generation)).encode()+b"\n"
            self.process.stdin.write(request);self.process.stdin.flush()
            while not result_path.exists():
                abort()
                self._remember_members()
                if self.process.poll() is not None:raise RuntimeError("read-only validation process exited before its proof")
                if self._clock()>=deadline:raise TimeoutError("read-only admission validation timed out")
                pump()
            abort()
            if self._clock()>=deadline:raise TimeoutError("read-only admission validation completed after its deadline")
            proof=_read_json(result_path)
            if proof.get("token")!=self._token or type(proof.get("generation")) is not int or proof["generation"]!=generation:
                raise ValueError("read-only admission proof generation or owner differs")
            if "error" in proof:raise ValueError("read-only validation failed: "+str(proof["error"])[:512])
            self._result_consumed=True
            self._pending=None
            return proof
        except BaseException:
            self._abandoned=True
            self._interrupt()
            raise

    def snapshot(self):
        self._remember_members()
        return dict(generation=self._generation,process_group=self.process.pid,
            process_alive=self.process.poll() is None,closed=self._closed,abandoned=self._abandoned,
            pending=self._pending is not None,late_completion_ignored=self._abandoned,
            queue_capacity=1,automatic_retry=False,ownership_error=self._ownership_error)

    def close(self, *, timeout_sec=.5):
        if type(timeout_sec) not in (int,float) or not 0<=timeout_sec<=1:
            raise ValueError("read-only worker close timeout must be bounded")
        self._closed=True
        try:self.process.stdin.close()
        except (OSError,ValueError):pass
        if self._abandoned:self._interrupt()
        deadline=time.monotonic()+timeout_sec
        self._remember_members()
        while time.monotonic()<deadline and (self.process.poll() is None or not self._owned_group_gone()):
            time.sleep(.005)
        for sig in (signal.SIGTERM,signal.SIGKILL):
            if self.process.poll() is not None and self._owned_group_gone():break
            self._forced=True
            self._signal_group(sig)
            try:self.process.wait(timeout=.25)
            except subprocess.TimeoutExpired:pass
            until=time.monotonic()+.25
            while not self._owned_group_gone() and time.monotonic()<until:time.sleep(.005)
        try:self.process.wait(timeout=.05)
        except subprocess.TimeoutExpired:pass
        result=self.snapshot()
        result.update(leader_return_code=self.process.returncode,process_group_gone=self._owned_group_gone(),forced=self._forced)
        result["quiescent"]=self.process.returncode is not None and result["process_group_gone"]
        return result


def _worker(configuration):
    # Fixed implementation, not an arbitrary command/callable execution API.
    from ..paths import ROOT
    from ..model_build.integrity import file_sha256
    from ..verification.reports import atomic_json
    from ..workcells.config import load_workcell
    from .ros_target import load_ros_grasp_target
    configuration=Path(configuration).resolve()
    if not configuration.is_relative_to((ROOT/"runtime").resolve()):raise ValueError("identity IPC must stay in this worktree runtime")
    config=_read_json(configuration)
    required={"token","parent_identity","target_directory","manifest_sha256","input_hashes","inventory"}
    if set(config)!=required:raise ValueError("unexpected read-only validator configuration")
    if (type(config["token"]) is not str or re.fullmatch(r"[0-9a-f]{32}",config["token"]) is None
        or type(config["manifest_sha256"]) is not str or re.fullmatch(r"[0-9a-f]{64}",config["manifest_sha256"]) is None
        or type(config["input_hashes"]) is not dict or not 1<=len(config["input_hashes"])<=128
        or type(config["inventory"]) is not list or not 1<=len(config["inventory"])<=2048):
        raise ValueError("invalid bounded validator identities or inventory")
    for path in [config["target_directory"],*config["inventory"],*config["input_hashes"]]:
        if type(path) is not str or not Path(path).is_absolute() or not Path(path).is_relative_to(ROOT):
            raise ValueError("validator paths must stay in its fixed worktree")
    if any(type(h) is not str or re.fullmatch(r"[0-9a-f]{64}",h) is None for h in config["input_hashes"].values()):
        raise ValueError("invalid validator source hashes")
    parent=config["parent_identity"]
    if parent is None or _process_identity(os.getppid())!=parent:raise ValueError("validation owner disappeared before startup")
    if os.getpid()!=os.getpgrp() or os.getpid()!=os.getsid(0):raise ValueError("validation process must own its isolated session")
    stop=threading.Event()
    def watch_parent():
        while not stop.wait(.05):
            if os.getppid()!=parent["pid"] or _process_identity(parent["pid"])!=parent:
                os.killpg(os.getpgrp(),signal.SIGKILL)
    watcher=threading.Thread(target=watch_parent,name="validation-parent-watch",daemon=True);watcher.start()
    try:
        target=None
        generation=0
        while True:
            line=sys.stdin.buffer.readline(257)
            if not line:break
            if len(line)>256:raise ValueError("validation request is oversized")
            request=json.loads(line)
            if (set(request)!={"token","generation"} or request["token"]!=config["token"]
                or type(request["generation"]) is not int or request["generation"]!=generation+1 or request["generation"]>128):
                raise ValueError("validation request owner or generation differs")
            generation=request["generation"]
            proof=dict(token=config["token"],generation=generation)
            try:
                before=file_stats(config["inventory"])
                if target is None:
                    target=load_ros_grasp_target(load_workcell("fr5_ur5e"),Path(config["target_directory"]),config["manifest_sha256"])
                checked=target.validate()
                proof.update(manifest_sha256=checked["manifest_sha256"],
                    observed_sha256={path:file_sha256(path) for path in config["input_hashes"]},
                    before=before,after=file_stats(config["inventory"]))
            except KeyboardInterrupt:
                break  # subprocess.check_output has reaped its interrupted Git child
            except BaseException as error:
                proof["error"]=dict(type=type(error).__name__,message=str(error)[:512])
            raw=json.dumps(proof,allow_nan=False).encode()
            if len(raw)>1024*1024:raise ValueError("validation proof exceeds its bound")
            atomic_json(configuration.parent/f"proof-{generation:03}.json",proof)
    finally:
        stop.set();watcher.join(timeout=.1)


if __name__=="__main__":
    if len(sys.argv)!=3 or sys.argv[1]!="--identity-worker":raise SystemExit("fixed identity worker mode required")
    try:_worker(sys.argv[2])
    except KeyboardInterrupt:pass
