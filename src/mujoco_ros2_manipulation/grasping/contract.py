"""Fail-closed task transitions; logical attachment never creates a grasp.

Evidence is supplied by the owner of a physics observation. This module does
not authenticate a caller, command hardware, or certify a payload. A stable
contact-only close is insufficient: lift and object/TCP stability are required.
"""
from dataclasses import dataclass, field
import math
import re


class GraspTransitionError(ValueError):
    pass


def _require(condition, message):
    if not condition:
        raise GraspTransitionError(message)


def _number(value, name):
    _require(type(value) in (int, float) and math.isfinite(value) and value >= 0,
             "Invalid nonnegative grasp observation: " + name)
    return float(value)


def _identity(value):
    _require(isinstance(value, str) and re.fullmatch(r"[0-9a-f]{64}", value) is not None,
             "Expected a complete fixture SHA-256")
    return value


@dataclass
class GraspLifecycle:
    fixture_sha256: str
    object_id: str = "block"
    owner: str = "synthetic_gripper"
    link: str = "carriage"
    stage: str = field(default="approaching", init=False)
    physical_grasp: str = field(default="unconfirmed", init=False)
    planning_attachment: str = field(default="world", init=False)
    events: list = field(default_factory=list, init=False)

    def __post_init__(self):
        _identity(self.fixture_sha256)
        for value in (self.object_id, self.owner, self.link):
            _require(isinstance(value, str) and re.fullmatch(r"[a-z][a-z0-9_]{0,63}", value) is not None,
                     "Invalid bounded grasp object/owner/link")
        self._event("started")

    def _event(self, name, **values):
        _require(len(self.events) < 32, "Grasp event bound exceeded")
        self.events.append(dict(event=name, stage=self.stage,
            physical_grasp=self.physical_grasp, planning_attachment=self.planning_attachment, **values))

    def _at(self, *stages):
        _require(self.stage in stages, "Invalid grasp transition from " + self.stage)
        _require(len(self.events) < 32, "Grasp event bound exceeded")

    def approach_complete(self):
        self._at("approaching")
        self.stage = "closing"
        self._event("approach_complete")

    def contacts_confirmed(self, *, left_force_n, right_force_n, observed_sec):
        self._at("closing")
        values = {name: _number(value, name) for name, value in dict(
            left_force_n=left_force_n, right_force_n=right_force_n, observed_sec=observed_sec).items()}
        _require(min(values["left_force_n"], values["right_force_n"]) >= .2
                 and values["observed_sec"] >= .1, "Bilateral sustained contact was not observed")
        self.stage = "extracting"
        self._event("bilateral_contact_confirmed", **values)

    def lift_confirmed(self, *, fixture_sha256, lift_m, stable_sec,
                       relative_position_error_m, relative_angle_error_rad, bilateral_fraction):
        self._at("extracting")
        _require(_identity(fixture_sha256) == self.fixture_sha256, "Grasp evidence fixture differs")
        values = {name: _number(value, name) for name, value in dict(lift_m=lift_m,
            stable_sec=stable_sec, relative_position_error_m=relative_position_error_m,
            relative_angle_error_rad=relative_angle_error_rad, bilateral_fraction=bilateral_fraction).items()}
        _require(values["lift_m"] >= .06 and values["stable_sec"] >= .2
            and values["relative_position_error_m"] <= .005
            and values["relative_angle_error_rad"] <= .15
            and .95 <= values["bilateral_fraction"] <= 1., "Lift or stable physical grasp was not observed")
        self.physical_grasp = "confirmed"
        self.stage = "holding"
        self._event("physical_grasp_confirmed", **values)

    def attach(self, *, object_id, owner, link, fixture_sha256):
        self._at("holding")
        _require((object_id, owner, link, _identity(fixture_sha256)) == (
            self.object_id, self.owner, self.link, self.fixture_sha256), "Attachment identity differs")
        _require(self.physical_grasp == "confirmed" and self.planning_attachment == "world",
                 "Attachment requires a new verified physical grasp")
        self.planning_attachment = "attached"
        self._event("logical_attachment_recorded", object_id=object_id, owner=owner, link=link)

    def carry(self):
        self._at("holding")
        _require(self.physical_grasp == "confirmed" and self.planning_attachment == "attached",
                 "Carry requires physical grasp and logical attachment")
        self.stage = "carrying"
        self._event("carry_started")

    def release(self):
        self._at("carrying")
        _require(self.physical_grasp == "confirmed" and self.planning_attachment == "attached",
                 "Release requires the carried object")
        self.stage = "releasing"
        self._event("release_started")

    def release_confirmed(self, *, fixture_sha256, bilateral_contact_absent_sec, open_gap_m):
        self._at("releasing")
        _require(_identity(fixture_sha256) == self.fixture_sha256, "Release evidence fixture differs")
        absent = _number(bilateral_contact_absent_sec, "bilateral_contact_absent_sec")
        gap = _number(open_gap_m, "open_gap_m")
        _require(absent >= .2 and .06 <= gap <= .08, "Physical release has not been observed")
        self.physical_grasp = "released"
        self.planning_attachment = "world"
        self.stage = "settling"
        self._event("physical_release_confirmed", bilateral_contact_absent_sec=absent, open_gap_m=gap)

    def settled(self, *, table_contact_sec, position_error_m, speed_m_s):
        self._at("settling")
        values = {name: _number(value, name) for name, value in dict(table_contact_sec=table_contact_sec,
            position_error_m=position_error_m, speed_m_s=speed_m_s).items()}
        _require(values["table_contact_sec"] >= .2 and values["position_error_m"] <= .01
            and values["speed_m_s"] <= .02, "Released object has not settled at the target")
        self.stage = "succeeded"
        self._event("placement_confirmed", **values)

    def fail(self, reason, *, slipped=False):
        self._at("approaching", "closing", "extracting", "holding", "carrying", "releasing", "settling")
        _require(isinstance(reason, str) and 0 < len(reason) <= 256, "Invalid failure reason")
        _require(type(slipped) is bool, "slipped must be boolean")
        if slipped:
            self.physical_grasp = "slipped"
        self.stage = "failed"
        # An unconfirmed logical detach would lie about planning-scene state.
        self._event("failed", reason=reason, explicit_recovery_required=self.planning_attachment == "attached")

    def cancel(self):
        self._at("approaching", "closing", "extracting", "holding", "carrying", "releasing", "settling")
        self.stage = "cancelled"
        self._event("cancelled", automatic_release=False, automatic_resume=False)

    def cancelled_observation(self, *, stop_observed, grasp_still_observed):
        """Record new hold observations without resuming or detaching a task.

        Cancellation delivery does not establish that the fixture stopped or
        retained the object. A logical attachment is deliberately left as-is.
        """
        self._at("cancelled")
        _require(type(stop_observed) is bool and type(grasp_still_observed) is bool,
                 "Cancellation observations must be explicit booleans")
        prior_confirmed = self.physical_grasp == "confirmed"
        self.physical_grasp = "confirmed" if grasp_still_observed and prior_confirmed else "unconfirmed"
        self._event("cancelled_hold_observed", stop_observed=stop_observed,
                    grasp_still_observed=grasp_still_observed and prior_confirmed)

    def failed_stop_observation(self, *, stop_observed):
        """A stopped failed run cannot renew an earlier grasp confirmation.

        Preserve a known slip or confirmed release, the original failure and
        logical attachment. There is no automatic release or recovery here.
        """
        self._at("failed")
        _require(stop_observed is True, "Failed-run stop must be explicitly observed")
        prior = self.physical_grasp
        if prior == "confirmed":
            self.physical_grasp = "unconfirmed"
        self._event("failed_stop_observed", stop_observed=True, physical_grasp_before_stop=prior)

    def snapshot(self):
        return dict(stage=self.stage, physical_grasp=self.physical_grasp,
            planning_attachment=self.planning_attachment, events=[dict(event) for event in self.events],
            attachment_authority="local logical lifecycle only; no MoveIt scene mutation")
