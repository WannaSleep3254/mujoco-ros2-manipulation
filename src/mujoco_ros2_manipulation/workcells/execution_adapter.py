"""Transport-neutral ownership of submitted trajectory actions.

The caller supplies goal creation, bounded waiting and its simulation clock.
This module does not admit requests, plan, alter trajectories or import ROS.
The mutable ledger belongs to one dispatch. Its locks protect bookkeeping
against future callbacks; the supplied client itself must support the caller's
executor/thread model. No ledger lock is held while calling a client or waiting.
"""
from dataclasses import dataclass, field
from itertools import islice
from numbers import Integral
from threading import RLock


MAX_EXECUTION_GOALS = 32
MAX_CLEANUP_ERRORS = 512
ACCEPTANCE_TIMEOUT_SEC = 5
CANCEL_TIMEOUT_SEC = 5
RESULT_TIMEOUT_SEC = 45
SUCCEEDED = 4
TERMINAL_STATUSES = frozenset((4, 5, 6))


def _message_info(error):
    try:
        value = str(error)
        return value[:2000], len(value) > 2000
    except BaseException:
        return "<unprintable exception>", False


def _message(error):
    return _message_info(error)[0]


def _confirmed_uuid(handle):
    """Return only a validated 16-byte UUID, including ROS integral scalars."""
    try:
        values = handle.goal_id.uuid
        if len(values) != 16 or any(not isinstance(value, Integral) or isinstance(value, bool)
                                   or not 0 <= value <= 255 for value in values):
            return None
        return bytes(int(value) for value in values).hex()
    except Exception:
        return None


@dataclass(eq=False)
class ExecutionAttempt:
    arm: str
    controller: object = None
    submission_started: bool = False
    submission_uncertain: bool = False
    send_future: object = None
    accepted_handle: object = None
    accepted: object = None
    goal_uuid: object = None
    result_future: object = None
    cancel_future: object = None
    cancel_requested: bool = False
    cancel_acknowledged: bool = False  # Ack received; does not mean terminal.
    terminal_seen: bool = False
    terminal_status: object = None
    terminal_error_code: object = None
    result_reported: bool = False
    acceptance_reported: bool = False
    _result_acquiring: bool = field(default=False, repr=False)
    _cancel_sending: bool = field(default=False, repr=False)
    _accept_callback_registered: bool = field(default=False, repr=False)
    _owner: object = field(default=None, repr=False)


@dataclass
class ExecutionLedger:
    total_dispatched: int = 0
    handles: list = field(default_factory=list)
    attempts: list = field(default_factory=list)
    acceptance_margins: list = field(default_factory=list)
    cleanup_errors: list = field(default_factory=list)
    cleanup_errors_dropped: int = 0
    cleanup_requested: bool = False
    cleanup_uncertain: bool = False
    _lock: object = field(default_factory=RLock, repr=False, compare=False)

    def snapshot(self):
        """Bounded detached JSON data; action terminal is not physical stop."""
        truncated_strings = [0]
        def text(value, limit=128, already_truncated=False):
            if value is None:
                return None
            value = value if isinstance(value, str) else "<" + type(value).__name__ + ">"
            if len(value) > limit or already_truncated:
                truncated_strings[0] += 1
            return value[:limit]
        def integer(value):
            return int(value) if isinstance(value, Integral) and not isinstance(value, bool) else None
        with self._lock:
            attempts = []
            for attempt in self.attempts[:MAX_EXECUTION_GOALS]:
                owner = attempt._owner or attempt
                attempts.append(dict(arm=text(attempt.arm), controller=text(attempt.controller),
                    confirmed_goal_uuid=attempt.goal_uuid,
                    submission_started=attempt.submission_started,
                    submission_uncertain=attempt.submission_uncertain,
                    sent=attempt.send_future is not None, accepted=attempt.accepted,
                    owns_handle=bool(attempt.accepted),
                    cancel_requested=owner.cancel_requested,
                    cancel_acknowledged=owner.cancel_acknowledged,
                    terminal_seen=owner.terminal_seen,
                    terminal_status=integer(owner.terminal_status),
                    terminal_error_code=integer(owner.terminal_error_code)))
            errors = []
            for error in self.cleanup_errors[:MAX_CLEANUP_ERRORS]:
                message = error.get("message")
                message_truncated = bool(error.get("message_truncated", False)) or (
                    isinstance(message, str) and len(message) > 2000)
                errors.append(dict(arm=text(error.get("arm")), stage=text(error.get("stage")),
                    type=text(error.get("type")),
                    message=text(message, 2000, message_truncated),
                    message_truncated=message_truncated))
            return dict(schema_version=1,
                total_dispatched_cumulative=integer(self.total_dispatched),
                request_attempt_count=len(self.attempts),
                request_submission_started_count=sum(attempt.submission_started for attempt in self.attempts),
                request_sent_count=sum(attempt.send_future is not None for attempt in self.attempts),
                attempts=attempts, cleanup_requested=self.cleanup_requested,
                cleanup_uncertain=self.cleanup_uncertain,
                cleanup_errors=errors,
                cleanup_error_count=len(self.cleanup_errors) + self.cleanup_errors_dropped,
                truncation=dict(attempts=max(0, len(self.attempts) - MAX_EXECUTION_GOALS),
                    cleanup_errors=max(0, len(self.cleanup_errors) - MAX_CLEANUP_ERRORS)
                        + self.cleanup_errors_dropped, strings=truncated_strings[0]),
                limits=dict(attempts=MAX_EXECUTION_GOALS, cleanup_errors=MAX_CLEANUP_ERRORS,
                            identifier_characters=128, error_message_characters=2000),
                cancel_acknowledgement_semantics="Response received; does not confirm cancellation acceptance or a terminal result",
                terminal_semantics="Action protocol terminal only; does not prove physical stop or process cleanup")


class ExecutionCleanupError(RuntimeError):
    """Bounded cleanup could not be confirmed; primary remains the cause."""
    def __init__(self, ledger, primary=None):
        self.ledger = ledger
        self.primary = primary
        detail = [dict(item) for item in ledger.cleanup_errors]
        prefix = _message(primary) + "; " if primary is not None else ""
        super().__init__(prefix + "peer cleanup errors: " + str(detail))


class TrajectoryExecutionAdapter:
    def __init__(self, clients, goal_factory, wait_result, wait_until,
                 simulation_now, event=None, abort=None, on_sent=None):
        self.clients = clients
        self.goal_factory = goal_factory  # trajectory -> goal, without alteration
        self.wait_result = wait_result  # (future, seconds, description) -> value
        self.wait_until = wait_until  # (predicate, seconds, description) -> None
        self.simulation_now = simulation_now
        self.event = event  # (name, arm, **details), existing observer convention
        self.abort = abort
        self.on_sent = on_sent  # (ledger), after actual send/count, before a wait

    @staticmethod
    def _error(ledger, attempt, stage, error):
        message, truncated = _message_info(error)
        record = dict(arm=getattr(attempt, "arm", None), stage=stage,
                      type=type(error).__name__, message=message, message_truncated=truncated)
        with ledger._lock:
            if len(ledger.cleanup_errors) < MAX_CLEANUP_ERRORS:
                ledger.cleanup_errors.append(record)
            else:
                ledger.cleanup_errors_dropped += 1

    @staticmethod
    def _owner(attempt):
        return attempt._owner or attempt

    @staticmethod
    def _refresh_uncertainty(ledger):
        # Called only with the ledger lock held.
        if ledger.cleanup_requested:
            ledger.cleanup_uncertain = any(
                attempt.submission_uncertain or
                (attempt.send_future is not None and attempt.accepted is None) or
                (attempt.accepted and not (attempt._owner or attempt).terminal_seen)
                for attempt in ledger.attempts)

    def _emit(self, name, attempt, **details):
        if self.event is not None:
            if attempt.controller is not None:
                details["controller"] = attempt.controller
            if attempt.goal_uuid is not None:
                details["goal_uuid"] = attempt.goal_uuid
            self.event(name, attempt.arm, **details)

    def _safe_callback(self, ledger, attempt, stage, callback):
        def receive(future):
            try:
                callback(future)
            except BaseException as error:
                self._error(ledger, attempt, stage, error)
        return receive

    def _observe_terminal(self, ledger, attempt, future):
        answer = future.result()
        status = getattr(answer, "status", None)
        error_code = getattr(getattr(answer, "result", None), "error_code", None)
        if (not isinstance(status, Integral) or isinstance(status, bool)
                or status not in TERMINAL_STATUSES
                or not isinstance(error_code, Integral) or isinstance(error_code, bool)):
            raise RuntimeError("Action result does not confirm a terminal state")
        with ledger._lock:
            owner = self._owner(attempt)
            owner.terminal_seen = True
            owner.terminal_status = int(status)
            owner.terminal_error_code = int(error_code)
            self._refresh_uncertainty(ledger)
        return answer

    def _acquire_result(self, ledger, attempt):
        with ledger._lock:
            owner = self._owner(attempt)
            if owner.result_future is not None or owner._result_acquiring:
                return owner.result_future
            owner._result_acquiring = True
        try:
            future = owner.accepted_handle.get_result_async()
            if future is None:
                raise RuntimeError("Accepted action has no result future")
        except BaseException:
            with ledger._lock:
                owner._result_acquiring = False
            raise
        with ledger._lock:
            owner.result_future = future
            owner._result_acquiring = False
            if not any(handle is owner.accepted_handle for handle, _ in ledger.handles):
                ledger.handles.append((owner.accepted_handle, future))
        callback = getattr(future, "add_done_callback", None)
        if callable(callback):
            callback(self._safe_callback(ledger, owner, "terminal_callback",
                lambda done: self._observe_terminal(ledger, owner, done)))
        return future

    def _claim(self, ledger, attempt, handle):
        # Own the accepted handle before invoking observers/result factories.
        with ledger._lock:
            if attempt.accepted is None:
                attempt.accepted_handle = handle
                attempt.accepted = bool(handle.accepted)
                attempt.goal_uuid = _confirmed_uuid(handle) if attempt.accepted else None
                if attempt.accepted:
                    previous = next((item for item in ledger.attempts
                        if item is not attempt and item.accepted and item.accepted_handle is handle), None)
                    attempt._owner = self._owner(previous) if previous is not None else None
            self._refresh_uncertainty(ledger)
        if attempt.accepted:
            self._acquire_result(ledger, attempt)

    def _register_acceptance_callback(self, ledger, attempt):
        with ledger._lock:
            if attempt._accept_callback_registered or attempt.send_future is None:
                return
            callback = getattr(attempt.send_future, "add_done_callback", None)
            if not callable(callback):
                return
            attempt._accept_callback_registered = True

        def accepted(done):
            with ledger._lock:
                if not ledger.cleanup_requested:
                    return
            try:
                self._claim(ledger, attempt, done.result())
            except BaseException as error:
                # The handle was stored before result acquisition. A broken
                # result factory must not prevent requesting its cancellation.
                self._error(ledger, attempt, "late_result_acquisition", error)
            if attempt.accepted:
                # Never spin, wait, call abort, or emit an observer from this
                # callback. A terminal-result callback alone clears uncertainty.
                self._request_cancel(ledger, attempt)
        callback(self._safe_callback(ledger, attempt, "late_acceptance_callback", accepted))

    def _request_cancel(self, ledger, attempt):
        with ledger._lock:
            owner = self._owner(attempt)
            if owner.terminal_seen or owner.cancel_future is not None or owner._cancel_sending:
                return owner.cancel_future
            owner._cancel_sending = True
            owner.cancel_requested = True
        try:
            future = owner.accepted_handle.cancel_goal_async()
            if future is None:
                raise RuntimeError("Cancellation has no acknowledgement future")
        except BaseException:
            with ledger._lock:
                owner._cancel_sending = False
            raise
        with ledger._lock:
            owner.cancel_future = future
            owner._cancel_sending = False

        def acknowledged(done):
            if done.result() is None:
                raise RuntimeError("Cancellation acknowledgement is empty")
            with ledger._lock:
                owner.cancel_acknowledged = True
        callback = getattr(future, "add_done_callback", None)
        if callable(callback):
            callback(self._safe_callback(ledger, owner, "cancel_ack_callback", acknowledged))
        return future

    def _cancel_known(self, ledger):
        with ledger._lock:
            owners = []
            for attempt in ledger.attempts:
                if attempt.accepted and self._owner(attempt) not in owners:
                    owners.append(self._owner(attempt))
        for attempt in owners:
            with ledger._lock:
                if attempt.terminal_seen:
                    continue
                future = attempt.result_future
            try:
                if future is not None and future.done():
                    self._observe_terminal(ledger, attempt, future)
                    continue
            except BaseException as error:
                self._error(ledger, attempt, "existing_terminal", error)
            try:
                cancellation = self._request_cancel(ledger, attempt)
                if cancellation is not None:
                    self.wait_result(cancellation, CANCEL_TIMEOUT_SEC, "peer cancellation")
                    with ledger._lock:
                        attempt.cancel_acknowledged = True
            except BaseException as error:
                self._error(ledger, attempt, "cancel_acknowledgement", error)
            # An acknowledgement is not a terminal result. Continue this wait
            # even if sending/waiting for acknowledgement failed.
            try:
                future = self._acquire_result(ledger, attempt)
                if future is None:
                    raise RuntimeError("Accepted action result acquisition remains pending")
                self.wait_result(future, CANCEL_TIMEOUT_SEC, "peer cancellation result")
                self._observe_terminal(ledger, attempt, future)
            except BaseException as error:
                self._error(ledger, attempt, "cancel_terminal", error)

    def _cleanup(self, ledger):
        with ledger._lock:
            # Compatibility for a caller-owned legacy handles list. No action
            # is discovered through a client, service, topic or global registry.
            for index, (handle, future) in enumerate(ledger.handles):
                if not any(attempt.accepted_handle is handle for attempt in ledger.attempts):
                    ledger.attempts.append(ExecutionAttempt(
                        arm="owned_" + str(index), accepted_handle=handle,
                        accepted=True, goal_uuid=_confirmed_uuid(handle), result_future=future))
            ledger.cleanup_requested = True
            self._refresh_uncertainty(ledger)
            attempts = list(ledger.attempts)
        for attempt in attempts:
            try:
                self._register_acceptance_callback(ledger, attempt)
            except BaseException as error:
                self._error(ledger, attempt, "acceptance_callback_registration", error)
        self._cancel_known(ledger)
        for attempt in attempts:
            if attempt.send_future is None:
                continue
            try:
                handle = self.wait_result(attempt.send_future, ACCEPTANCE_TIMEOUT_SEC, "pending peer acceptance")
                self._claim(ledger, attempt, handle)
                with ledger._lock:
                    report = not attempt.acceptance_reported
                    attempt.acceptance_reported = True
                if report:
                    self._emit("goal_acceptance", attempt, accepted=attempt.accepted)
            except BaseException as error:
                self._error(ledger, attempt, "pending_acceptance", error)
        self._cancel_known(ledger)
        with ledger._lock:
            self._refresh_uncertainty(ledger)

    def cancel_actions(self, ledger):
        """Cancel/drain only this ledger's submitted actions, without abort hooks."""
        self._cleanup(ledger)
        if ledger.cleanup_errors or ledger.cleanup_uncertain:
            raise ExecutionCleanupError(ledger)
        return ledger

    def dispatch(self, trajectories, ledger, *, scheduled_start=None, inject_cancel_at=None):
        items = list(islice(iter(trajectories), MAX_EXECUTION_GOALS + 1))
        if not 1 <= len(items) <= MAX_EXECUTION_GOALS:
            raise ValueError("Expected 1..32 trajectory actions")
        with ledger._lock:
            if ledger.attempts or ledger.handles or ledger.cleanup_requested:
                raise ValueError("Dispatch requires a fresh execution ledger")
        try:
            for instance, trajectory in items:
                attempt = ExecutionAttempt(instance.id, getattr(instance, "controller", None))
                with ledger._lock:
                    ledger.attempts.append(attempt)
                goal = self.goal_factory(trajectory)
                self._emit("goal_sent", attempt)
                if self.abort is not None:
                    self.abort()
                client = self.clients[attempt.arm]
                with ledger._lock:
                    # Entering the transport may submit a goal even when it
                    # raises or supplies no future. Without an owned future we
                    # cannot discover/cancel that possible goal or prove none.
                    attempt.submission_started = True
                    attempt.submission_uncertain = True
                future = client.send_goal_async(goal)
                if future is None:
                    raise RuntimeError("Action client returned no acceptance future")
                with ledger._lock:
                    attempt.send_future = future
                    attempt.submission_uncertain = False
                    ledger.total_dispatched += 1
                if self.on_sent is not None:
                    self.on_sent(ledger)
                self._register_acceptance_callback(ledger, attempt)
            rejected = []
            for attempt in ledger.attempts:
                handle = self.wait_result(attempt.send_future, ACCEPTANCE_TIMEOUT_SEC, "arm goal acceptance")
                self._claim(ledger, attempt, handle)
                with ledger._lock:
                    attempt.acceptance_reported = True
                self._emit("goal_acceptance", attempt, accepted=attempt.accepted)
                if not attempt.accepted:
                    rejected.append(attempt.arm)
            if rejected:
                raise RuntimeError("Arm goal rejected: " + str(rejected))
            if scheduled_start is not None:
                margin = scheduled_start - self.simulation_now()
                if not margin > .1:
                    raise RuntimeError("Both goals must be accepted at least 0.1s before their start")
                ledger.acceptance_margins.append(margin)
            injected = False

            def finished():
                nonlocal injected
                if self.abort is not None:
                    self.abort()
                if inject_cancel_at is not None and not injected and self.simulation_now() >= inject_cancel_at:
                    self._request_cancel(ledger, ledger.attempts[0])
                    injected = True
                for attempt in ledger.attempts:
                    owner = self._owner(attempt)
                    if owner.result_future.done():
                        answer = self._observe_terminal(ledger, owner, owner.result_future)
                        if not attempt.result_reported:
                            attempt.result_reported = True
                            self._emit("goal_result", attempt, status=answer.status,
                                       error_code=answer.result.error_code)
                        if answer.status != SUCCEEDED or answer.result.error_code != 0:
                            raise RuntimeError("An arm failed; cancelling peers")
                return all(self._owner(attempt).result_future.done() for attempt in ledger.attempts)
            self.wait_until(finished, RESULT_TIMEOUT_SEC, "all arm results")
        except BaseException as primary:
            try:
                self._cleanup(ledger)
            except BaseException as error:
                self._error(ledger, None, "cleanup", error)
                with ledger._lock:
                    ledger.cleanup_uncertain = True
            if ledger.cleanup_errors or ledger.cleanup_uncertain:
                raise ExecutionCleanupError(ledger, primary) from primary
            raise
        return ledger
