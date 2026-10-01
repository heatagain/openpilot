import copy
import json
import time
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from opendbc.can import CANPacker
from opendbc.can.parser import get_raw_value
from opendbc.car import Bus, structs
from opendbc.car.hyundai import carcontroller, hyundaican, syswarning_test as testtool
from opendbc.car.hyundai.values import CAR, HyundaiFlags


def decode(packer, message):
  return {key: get_raw_value(message[1], sig) * sig.factor + sig.offset
          for key, sig in packer.dbc.addr_to_msg[message[0]].sigs.items()}


def inputs():
  return (structs.CarControl(), structs.CarState(canValid=True, standstill=True, vEgo=0, vEgoRaw=0,
                                               gearShifter="park", parkingBrake=True))


def request(tool, value):
  testtool.main([str(value)], directory=tool.directory)
  tool.poll_once()


@pytest.mark.parametrize("value", range(7))
@pytest.mark.parametrize("dm", range(4))
@pytest.mark.parametrize("sys_state", [1, 3, 4, 5, 6])
def test_packed_display_only_and_off(value, dm, sys_state):
  packer = CANPacker("hyundai_kia_generic")
  stock = {key: 0 for key in packer.dbc.name_to_msg["LKAS11"].sigs}
  stock["CF_Lkas_HbaOpt"] = 1
  original = copy.deepcopy(stock)
  cp = NS(carFingerprint=CAR.HYUNDAI_ELANTRA, flags=CAR.HYUNDAI_ELANTRA.config.flags)
  args = (packer, 31, cp, 123, True, True, stock, True, sys_state, False, True, False, 2, 1, True)
  normal = hyundaican.create_lkas11(*args, dm_alert=dm)
  assert hyundaican.create_lkas11(*args, dm_alert=dm, syswarning_test_value=None) == normal
  msg = hyundaican.create_lkas11(*args, dm_alert=dm, syswarning_test_value=value)
  values, baseline = decode(packer, msg), decode(packer, normal)
  assert (msg[0], msg[2]) == (0x340, 0)
  assert values["CF_Lkas_SysWarning"] == value
  assert values["CF_Lkas_LdwsSysState"] == (3 if value else baseline["CF_Lkas_LdwsSysState"])
  changed = {"CF_Lkas_SysWarning", "CF_Lkas_LdwsSysState", "CF_Lkas_Chksum"}
  assert {k: v for k, v in values.items() if k not in changed} == {k: v for k, v in baseline.items() if k not in changed}
  assert values["CF_Lkas_Chksum"] == (sum(msg[1][:6]) + msg[1][7]) % 256
  assert values["CF_Lkas_MsgCount"] == 15
  assert stock == original


@pytest.mark.parametrize("fingerprint,flags", [(CAR.HYUNDAI_SONATA_LF, 0), (CAR.HYUNDAI_ELANTRA_GT_I30, 0),
                                              (CAR.HYUNDAI_ELANTRA, HyundaiFlags.SEND_LFA),
                                              (CAR.HYUNDAI_ELANTRA, HyundaiFlags.CANFD)])
def test_other_platforms_cannot_override(fingerprint, flags):
  packer = CANPacker("hyundai_kia_generic")
  cp = NS(carFingerprint=fingerprint, flags=flags)
  stock = {key: 0 for key in packer.dbc.name_to_msg["LKAS11"].sigs}
  args = (packer, 0, cp, 0, False, False, stock, False, 1, False, False, False, 0, 0, False)
  assert hyundaican.create_lkas11(*args, syswarning_test_value=6) == hyundaican.create_lkas11(*args)


@pytest.mark.parametrize("field,value", [("vEgo", 0.1), ("vEgo", -0.1), ("vEgoRaw", 0.1),
                                      ("vEgo", float("nan")), ("vEgoRaw", float("inf")),
                                      ("standstill", False), ("parkingBrake", False),
                                      ("gearShifter", "drive"), ("canValid", False), ("canTimeout", True),
                                      ("enabled", True), ("latActive", True), ("longActive", True)])
def test_gate_disarms_and_requires_new_command(tmp_path, field, value):
  tool = testtool.SysWarningTest(tmp_path, start_thread=False)
  cc, state = inputs()
  request(tool, 6)
  assert tool.get_override(cc, state) == 6
  target = cc if field in ("enabled", "latActive", "longActive") else state
  old = getattr(target, field)
  setattr(target, field, value)
  assert tool.get_override(cc, state) is None
  setattr(target, field, old)
  tool.poll_once()
  assert tool.get_override(cc, state) is None
  request(tool, 6)
  assert tool.get_override(cc, state) == 6


def test_live_off_zero_and_no_io_on_control_thread(tmp_path, monkeypatch):
  tool = testtool.SysWarningTest(tmp_path, start_thread=False)
  cc, state = inputs()
  assert tool.get_override(cc, state) is None
  for value in range(7):
    request(tool, value)
    with monkeypatch.context() as patch:
      patch.setattr(Path, "open", lambda *a, **kw: pytest.fail("File I/O on control thread"))
      for _ in range(100):
        assert tool.get_override(cc, state) == value
  testtool.main(["off"], directory=tmp_path)
  tool.poll_once()
  assert tool.get_override(cc, state) is None


@pytest.mark.parametrize("data", [None, [], {}, {"value": True}, {"value": -1}, {"value": 7},
                                {"value": "3"}, {"value": 3, "issued": float("nan"), "token": "a" * 32}])
def test_invalid_request_falls_back(tmp_path, data):
  tool = testtool.SysWarningTest(tmp_path, start_thread=False)
  (tmp_path / "request.json").write_text(json.dumps(data))
  tool.poll_once()
  assert tool.get_override(*inputs()) is None


def test_expiry_reader_stall_restart_and_missing_file(tmp_path, monkeypatch):
  now = [10.0]
  monkeypatch.setattr(testtool.time, "monotonic", lambda: now[0])
  tool = testtool.SysWarningTest(tmp_path, start_thread=False)
  cc, state = inputs()
  now[0] += 1
  request(tool, 3)
  assert tool.get_override(cc, state) == 3
  now[0] += testtool.READER_TIMEOUT
  assert tool.get_override(cc, state) is None
  tool.poll_once()
  assert tool.get_override(cc, state) is None
  request(tool, 3)
  assert tool.get_override(cc, state) == 3
  now[0] += testtool.LEASE_SECONDS
  tool.last_poll = now[0]
  assert tool.get_override(cc, state) is None
  request(tool, 3)
  now[0] += 0.01
  restarted = testtool.SysWarningTest(tmp_path, start_thread=False)
  restarted.poll_once()
  assert restarted.get_override(cc, state) is None
  request(restarted, 3)
  assert restarted.get_override(cc, state) == 3
  (tmp_path / "request.json").unlink()
  restarted.poll_once()
  assert restarted.get_override(cc, state) is None


def test_cli_status_reports_requested_and_controller_state(tmp_path, capsys):
  tool = testtool.SysWarningTest(tmp_path, start_thread=False)
  request(tool, 0)
  tool.get_override(*inputs())
  tool.poll_once()
  testtool.main(["status"], directory=tmp_path)
  output = capsys.readouterr().out
  assert "Requested value      : 0" in output
  assert "Controller state     : APPLYING" in output
  testtool.main(["off"], directory=tmp_path)
  tool.get_override(*inputs())
  tool.poll_once()
  testtool.main(["status"], directory=tmp_path)
  assert "Requested value      : OFF" in capsys.readouterr().out


def test_background_reader_picks_up_commands(tmp_path):
  tool = testtool.SysWarningTest(tmp_path)
  cc, state = inputs()

  def wait_for(expected):
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
      if tool.get_override(cc, state) == expected:
        return
      time.sleep(0.01)
    pytest.fail(f"Background reader did not apply {expected}")

  testtool.main(["0"], directory=tmp_path)
  wait_for(0)
  testtool.main(["6"], directory=tmp_path)
  wait_for(6)
  testtool.main(["off"], directory=tmp_path)
  wait_for(None)


def test_real_controller_live_transition(tmp_path, monkeypatch):
  tool = testtool.SysWarningTest(tmp_path, start_thread=False)
  monkeypatch.setattr(carcontroller, "SysWarningTest", lambda: tool)
  params = NS(get_int=lambda key: 0, get_bool=lambda key: False, get_float=lambda key: 0.0)
  monkeypatch.setattr(carcontroller, "Params", lambda: params)
  cp = structs.CarParams(carFingerprint=CAR.HYUNDAI_ELANTRA, wheelbase=2.7, steerRatio=15.4,
                        flags=int(CAR.HYUNDAI_ELANTRA.config.flags), openpilotLongitudinalControl=False)
  controller = carcontroller.CarController({Bus.pt: "hyundai_kia_generic"}, cp)
  controller.lkas11_active = True
  cc, state = inputs()
  stock = {key: 0 for key in controller.packer.dbc.name_to_msg["LKAS11"].sigs}
  cs = NS(out=state, lkas11=stock, modelV2=None, is_metric=True, clu11=None,
          paddle_button_prev=0, softHoldActive=0, mdps12=None)

  def sent_warning():
    _, messages = controller.update(cc.as_reader(), cs, controller.frame * 10_000_000)
    lkas = [msg for msg in messages if msg[0] == 0x340]
    assert len(lkas) == 1
    return decode(controller.packer, lkas[0])["CF_Lkas_SysWarning"]

  assert sent_warning() == 0
  for value in range(7):
    request(tool, value)
    assert sent_warning() == value
  state.vEgo = 0.1
  assert sent_warning() == 0
  state.vEgo = 0
  assert sent_warning() == 0
  request(tool, 5)
  assert sent_warning() == 5
  cc.latActive = True  # AlwaysLateral while not enabled also blocks the override.
  assert sent_warning() == 0
  cc.latActive = False
  assert sent_warning() == 0
  request(tool, 0)
  cc.hudControl.driverMonitoringAlert = 3
  assert sent_warning() == 0
  testtool.main(["off"], directory=tmp_path)
  tool.poll_once()
  assert sent_warning() == 3
