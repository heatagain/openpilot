import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.cereal import car, log
from openpilot.selfdrive.car.hyundai_mdps_recovery import MdpsRecovery, supported, warning_decision
from opendbc.car.hyundai.values import HyundaiFlags


def cp(**changes):
  values = {"brand": "hyundai", "carFingerprint": "HYUNDAI_ELANTRA", "steerControlType": "torque", "flags": int(HyundaiFlags.LEGACY)}
  return car.CarParams(**(values | changes))


class Feed:
  def __init__(self):
    self.monitor = MdpsRecovery()
    self.now = 1_000_000_000
    self.counter = 0

  def tick(self, *, dt=10, fault=False, active=True, bits=0, angle=0, request=True, protection=False,
           corrupt=None, counter_skip=False, valid=True, bus=0):
    self.now += dt * 1_000_000
    self.counter += 2 if counter_skip else 1
    x = 1024 | (int(fault) << 14) | (int(active) << 13) | (int(fault) << 15) | bits
    b = bytearray((x | ((self.counter % 256) << 16)).to_bytes(8, "little"))
    b[3] = sum(b) % 256
    lk = bytearray((1024 << 16 | int(request) << 27 | int(protection) << 28 | (self.counter % 16) << 36).to_bytes(8, "little"))
    lk[6] = (sum(lk[:6]) + lk[7]) % 256
    sas = int(round(angle * 10)).to_bytes(2, "little", signed=True) + bytes(3)
    if corrupt == "mdps":
      b[3] ^= 1
    if corrupt == "lkas":
      lk[6] ^= 1
    frames = [(688, sas, 0), (832, bytes(lk), 128), (593, bytes(b), bus)]
    return self.monitor.update([(self.now, frames)], self.now, valid)


@pytest.mark.parametrize("changes", [{"brand": "kia"}, {"carFingerprint": "HYUNDAI_SONATA"}, {"steerControlType": "angle"},
                                    {"flags": 0}, {"flags": int(HyundaiFlags.LEGACY | HyundaiFlags.CANFD)},
                                    {"flags": int(HyundaiFlags.LEGACY | HyundaiFlags.ANGLE_CONTROL)}])
def test_scope(changes):
  assert supported(cp())
  assert not supported(cp(**changes))


def test_torque_fault_is_not_debounced():
  f = Feed()
  f.tick()
  result = f.tick(fault=True, active=False)
  cs = car.CarState(steerFaultTemporary=True, canValid=True, hyundaiMdpsRecovery=result)
  assert warning_decision(cs, f.now).pending
  assert cs.steerFaultTemporary  # Consumer qualification does not mutate the safety bit.
  # Execute the real control gate without platform-only controlsd dependencies.
  p = Path(__file__).parents[2] / "controls/controlsd.py"
  node = next(n for n in ast.parse(p.read_text(encoding="utf8")).body if isinstance(n, ast.FunctionDef) and n.name == "lateral_control_allowed")
  ns = {}
  exec(compile(ast.Module(body=[node], type_ignores=[]), str(p), "exec"), ns)
  assert not ns["lateral_control_allowed"](True, False, True, cs.steerFaultTemporary, False, False, False, False)
  assert not ns["lateral_control_allowed"](False, True, True, cs.steerFaultTemporary, False, False, False, False)


@pytest.mark.parametrize("recovery_ms, expected", [(80, False), (100, False), (120, False), (121, True), (150, True)])
def test_recovery_deadline(recovery_ms, expected):
  f = Feed()
  f.tick()
  f.tick(fault=True, active=False)
  # Clear at 60 ms; active recovery remains a separate required observation.
  for _ in range(5):
    assert f.tick(fault=True, active=False)["pending"]
  assert f.tick(active=False)["pending"]
  elapsed = 60
  while elapsed + 10 < recovery_ms:
    f.tick(active=False)
    elapsed += 10
  result = f.tick(dt=recovery_ms - elapsed)
  assert result["warning"] == expected
  assert not result["pending"]


@pytest.mark.parametrize("kwargs", [{"bits": 1 << 12}, {"bits": 1 << 11}, {"bits": 1 << 37},
                                   {"angle": 85}, {"angle": -85}, {"protection": True},
                                   {"corrupt": "mdps"}, {"corrupt": "lkas"}, {"counter_skip": True}, {"valid": False}])
def test_guard_bypasses_grace(kwargs):
  f = Feed()
  f.tick()
  result = f.tick(fault=True, active=False, **kwargs)
  assert result["warning"] and not result["pending"]


def test_standalone_failstat_and_startup_fault():
  f = Feed()
  assert f.tick(fault=True, active=False)["warning"]
  f = Feed()
  f.tick()
  assert f.tick(bits=1 << 15)["warning"]


def test_lost_frames_and_consumer_watchdog():
  f = Feed()
  f.tick()
  status = f.tick(fault=True, active=False)
  cs = car.CarState(canValid=True, steerFaultTemporary=True, hyundaiMdpsRecovery=status)
  assert warning_decision(cs, f.now + 120_000_000).force_fault
  assert not warning_decision(cs, f.now + 120_000_000).pending
  assert f.monitor.update([], f.now + 100_000_000, True)["warning"]


def test_repeat_is_not_given_another_grace_until_healthy_horizon():
  f = Feed()
  f.tick()
  assert f.tick(fault=True, active=False)["pending"]
  assert not f.tick()["warning"]
  assert f.tick(fault=True, active=False)["warning"]
  for _ in range(303):
    f.tick()
  assert f.tick(fault=True, active=False)["pending"]


def test_wrong_bus_never_qualifies_and_old_schema_defaults_are_inert():
  f = Feed()
  f.tick(bus=2)
  result = f.tick(fault=True, active=False, bus=2)
  assert not result["pending"]
  assert not warning_decision(car.CarState(steerFaultTemporary=True), f.now).pending


def test_severe_after_pending_promotes_immediately():
  f = Feed()
  f.tick()
  assert f.tick(fault=True, active=False)["pending"]
  assert f.tick(bits=1 << 37)["warning"]


def test_same_batch_severe_dominates_recovery():
  f = Feed()
  f.tick()
  assert f.tick(fault=True, active=False)["pending"]
  f.now += 10_000_000
  b = bytearray((1024 | 1 << 37 | (f.counter + 1) << 16).to_bytes(8, "little"))
  b[3] = sum(b) % 256
  good = bytearray((1024 | 1 << 13 | (f.counter + 2) << 16).to_bytes(8, "little"))
  good[3] = sum(good) % 256
  assert f.monitor.update([(f.now, [(593, b, 0), (593, good, 0)])], f.now, True)["warning"]


def test_schema_roundtrip_and_pending_entry_block():
  cs = car.CarState(steerFaultTemporary=True, hyundaiMdpsRecovery={"available": True, "pending": True,
                         "faultStartMonoTime": 1_000_000_000, "sampleMonoTime": 1_010_000_000})
  with car.CarState.from_bytes(cs.to_bytes()) as decoded:
    assert decoded.steerFaultTemporary and decoded.hyundaiMdpsRecovery.pending
  assert log.OnroadEvent.EventName.steerTempUnavailablePending == 130


def load_state_machine():
  # Only platform imports are replaced. Execute the actual production class.
  p = Path(__file__).parents[2] / "selfdrived/state.py"
  tree = ast.parse(p.read_text(encoding="utf8"))
  nodes = [n for n in tree.body if not isinstance(n, (ast.Import, ast.ImportFrom))]
  import math
  ns = {"log": log, "Events": object, "DT_CTRL": .01, "math": math,
            "ET": SimpleNamespace(**{n: n for n in ["PERMANENT", "USER_DISABLE", "IMMEDIATE_DISABLE", "SOFT_DISABLE",
                    "OVERRIDE_LATERAL", "OVERRIDE_LONGITUDINAL", "ENABLE", "NO_ENTRY", "PRE_ENABLE", "WARNING"]})}
  exec(compile(ast.Module(body=nodes, type_ignores=[]), str(p), "exec"), ns)
  return ns["StateMachine"]


def test_preserves_original_disable_deadline():
  machine = load_state_machine()()
  machine.state = log.SelfdriveState.OpenpilotState.enabled
  events = SimpleNamespace(events=[], contains=lambda kind: kind == "SOFT_DISABLE")
  machine.update(events, soft_disable_elapsed=.12)
  assert machine.soft_disable_timer == 288
  for _ in range(288):
    machine.update(events)
  assert machine.state == log.SelfdriveState.OpenpilotState.disabled


def test_already_expired_disable_budget_and_unrelated_timer():
  machine = load_state_machine()()
  machine.state = log.SelfdriveState.OpenpilotState.enabled
  events = SimpleNamespace(events=[], contains=lambda kind: kind == "SOFT_DISABLE")
  machine.update(events, soft_disable_elapsed=3.1)
  assert machine.state == log.SelfdriveState.OpenpilotState.disabled
  machine = load_state_machine()()
  machine.state = log.SelfdriveState.OpenpilotState.enabled
  machine.update(events)
  assert machine.soft_disable_timer == 300


def event_helper(CP, now):
  """Run production event generation with only host/UI dependencies replaced."""
  from collections import deque
  from openpilot.selfdrive.car.hyundai_mdps_recovery import WarningDecision
  root = Path(__file__).parents[2]
  tree = ast.parse((root / "selfdrived/events.py").read_text(encoding="utf8"))
  event_dict = next(n.value for n in tree.body if isinstance(n, ast.AnnAssign) and
                    isinstance(n.target, ast.Name) and n.target.id == "EVENTS")
  # Read actual event-type mappings; alert rendering is not exercised on Windows.
  mapping = {getattr(log.OnroadEvent.EventName, key.attr): {t.attr for t in val.keys}
             for key, val in zip(event_dict.keys, event_dict.values, strict=True) if isinstance(key, ast.Attribute)}

  class RuntimeEvents:
    def __init__(self):
      self.events = []

    def add(self, event):
      self.events.append(event)

    def contains(self, kind):
      return any(kind in mapping.get(e, set()) for e in self.events)

  p = root / "car/car_specific.py"
  node = next(n for n in ast.parse(p.read_text(encoding="utf8")).body if isinstance(n, ast.ClassDef) and n.name == "CarSpecificEvents")
  ns = {"car": car, "structs": car, "deque": deque, "mdps_recovery_supported": supported, "warning_decision": warning_decision,
            "WarningDecision": WarningDecision, "Params": lambda: SimpleNamespace(), "HYUNDAI_PREV_BUTTON_SAMPLES": 8,
            "Events": RuntimeEvents, "EventName": log.OnroadEvent.EventName, "GearShifter": car.CarState.GearShifter,
            "ButtonType": car.CarState.ButtonEvent.Type, "MAX_CTRL_SPEED": 100, "BLUETOOTH_CANCEL": -2,
            "DT_CTRL": .01, "time": SimpleNamespace(monotonic_ns=lambda: now[0]), "ET": SimpleNamespace(NO_ENTRY="NO_ENTRY")}
  exec(compile(ast.Module(body=[node], type_ignores=[]), str(p), "exec"), ns)
  helper = ns["CarSpecificEvents"](CP)
  helper.silent_steer_warning = 0
  helper.steering_unpressed = 1000
  return helper


def state_with(status, **changes):
  return car.CarState(**({"canValid": True, "steerFaultTemporary": True, "seatbeltUnlatched": False, "gearShifter": "drive",
                            "cruiseState": {"available": True}, "hyundaiMdpsRecovery": status} | changes))


def test_pending_blocks_entry_but_does_not_soft_disable_then_times_out_even_with_driver_override():
  f = Feed()
  f.tick()
  status = f.tick(fault=True, active=False)
  now = [f.now]
  helper = event_helper(cp(), now)
  cs = state_with(status, activateCruise=1)
  ev = helper.create_common_events(cs, car.CarState(), pcm_enable=False)
  assert ev.contains("NO_ENTRY") and not ev.contains("SOFT_DISABLE")
  assert log.OnroadEvent.EventName.buttonEnable not in ev.events
  now[0] += 120_000_000
  cs.steeringPressed = True
  ev = helper.create_common_events(cs, cs, pcm_enable=False)
  assert ev.contains("SOFT_DISABLE") and ev.contains("NO_ENTRY")
  assert helper.steer_fault_elapsed == .12


def test_other_car_cannot_use_even_forged_pending_metadata():
  f = Feed()
  f.tick()
  status = f.tick(fault=True, active=False)
  helper = event_helper(cp(carFingerprint="HYUNDAI_SONATA"), [f.now])
  cs = state_with(status)
  ev = helper.create_common_events(cs, car.CarState(), pcm_enable=False)
  assert ev.contains("SOFT_DISABLE")
  assert log.OnroadEvent.EventName.steerTempUnavailablePending not in ev.events


def test_clear_without_active_recovery_cannot_silently_drop_the_episode():
  f = Feed()
  f.tick()
  f.tick(fault=True, active=False)
  for _ in range(12):
    status = f.tick(active=False)
  cs = state_with(status, steerFaultTemporary=False)
  helper = event_helper(cp(), [f.now])
  ev = helper.create_common_events(cs, cs, pcm_enable=False)
  assert ev.contains("SOFT_DISABLE")
  assert not cs.steerFaultTemporary


def test_recovered_episode_resets_event_history_and_leaves_no_warning():
  f = Feed()
  f.tick()
  f.tick(fault=True, active=False)
  status = f.tick()
  cs = state_with(status, steerFaultTemporary=False)
  helper = event_helper(cp(), [f.now])
  ev = helper.create_common_events(cs, cs, pcm_enable=False)
  assert log.OnroadEvent.EventName.steerTempUnavailable not in ev.events
  assert log.OnroadEvent.EventName.steerTempUnavailablePending not in ev.events


def test_protection_bypass_preserves_original_driver_and_clear_gates():
  f = Feed()
  f.tick(angle=90)
  status = f.tick(fault=True, active=False, angle=90)
  helper = event_helper(cp(), [f.now])
  cs = state_with(status, steeringPressed=True, steeringAngleDeg=90)
  ev = helper.create_common_events(cs, car.CarState(), pcm_enable=False)
  assert not ev.contains("SOFT_DISABLE")  # Original driver override handling.
  status = f.tick(active=False, angle=90)
  cs = state_with(status, steerFaultTemporary=False, steeringAngleDeg=90)
  helper = event_helper(cp(), [f.now])
  ev = helper.create_common_events(cs, cs, pcm_enable=False)
  assert not ev.contains("SOFT_DISABLE")  # No synthetic warning after baseline clear.


def test_can_invalid_replacing_pending_fault_keeps_original_deadline():
  f = Feed()
  f.tick()
  status = f.tick(fault=True, active=False)
  cs = state_with(status, canValid=False)
  decision = warning_decision(cs, f.now + 80_000_000)
  assert decision.force_fault and decision.elapsed == .08
  machine = load_state_machine()()
  machine.state = log.SelfdriveState.OpenpilotState.enabled
  can_error = SimpleNamespace(events=[], contains=lambda kind: kind == "SOFT_DISABLE")
  machine.update(can_error, soft_disable_elapsed=decision.elapsed)
  assert machine.soft_disable_timer == 292
