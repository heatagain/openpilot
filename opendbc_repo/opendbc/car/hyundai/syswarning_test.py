"""Temporary parked Elantra AD display test. No CAN transmitter or Params key."""
import argparse
import json
import math
import os
from pathlib import Path
import tempfile
import threading
import time
import uuid

DIRECTORY = Path("/dev/shm/openpilot_syswarning_test")
LEASE_SECONDS = 120.0
POLL_SECONDS = 0.5
READER_TIMEOUT = 2.0


def atomic_write(path, value):
  fd, name = tempfile.mkstemp(dir=path.parent, prefix=".tmp-")
  try:
    with os.fdopen(fd, "w") as f:
      json.dump(value, f, allow_nan=False)
    os.replace(name, path)
  finally:
    if os.path.exists(name):
      os.unlink(name)


def valid_request(data, now, started=0.0):
  try:
    value, issued, token = data["value"], data["issued"], data["token"]
    if (type(value) is int and 0 <= value <= 6 and type(issued) in (int, float)
        and math.isfinite(issued) and started <= issued <= now < issued + LEASE_SECONDS
        and isinstance(token, str) and len(token) == 32 and all(c in "0123456789abcdef" for c in token)):
      return value, issued, token
  except (KeyError, TypeError):
    pass
  return None


def parked_gate(CC, state):
  # Check both active flags: AlwaysLateral can be active while CC.enabled is false.
  return (state.canValid and not state.canTimeout and state.standstill
          and math.isfinite(state.vEgo) and abs(state.vEgo) < 0.1
          and math.isfinite(state.vEgoRaw) and abs(state.vEgoRaw) < 0.1
          and state.gearShifter == "park" and state.parkingBrake
          and not CC.enabled and not CC.latActive and not CC.longActive)


class SysWarningTest:
  def __init__(self, directory=DIRECTORY, *, start_thread=True):
    self.directory = Path(directory)
    self.started = time.monotonic()
    self.request = None
    self.last_poll = -math.inf
    self.blocked_token = None
    self.status = {"state": "OFF", "value": None, "token": None, "updated": self.started}
    if start_thread:
      try:
        threading.Thread(target=self._run, name="lkas-warning-test", daemon=True).start()
      except RuntimeError:
        pass  # Unable to start the optional reader: remain OFF.

  def poll_once(self):
    # Called exclusively by the background reader, never by the 100 Hz controller.
    now = time.monotonic()
    try:
      with (self.directory / "request.json").open() as f:
        data = json.loads(f.read(1024))
      request = valid_request(data, now, self.started)
    except (OSError, ValueError):
      request = None
    self.request = request
    self.last_poll = now
    try:
      atomic_write(self.directory / "status.json", self.status)
    except OSError:
      pass

  def _run(self):
    while True:
      self.poll_once()
      time.sleep(POLL_SECONDS)

  def get_override(self, CC, state):
    # Only cached memory + monotonic clock on the control thread. Fail closed.
    now = time.monotonic()
    request = self.request
    value, token, mode = None, None, "OFF"
    if request is not None:
      requested, issued, token = request
      if now >= issued + LEASE_SECONDS or now - self.last_poll >= READER_TIMEOUT:
        self.blocked_token = token
        mode = "EXPIRED_OR_READER_STALE"
      elif not parked_gate(CC, state):
        self.blocked_token = token
        mode = "GATE_BLOCKED_REISSUE_COMMAND"
      elif token == self.blocked_token:
        mode = "DISARMED_REISSUE_COMMAND"
      else:
        value, mode = requested, "APPLYING"
    self.status = {"state": mode, "value": value, "token": token, "updated": now}
    return value


def main(argv=None, *, directory=DIRECTORY):
  parser = argparse.ArgumentParser(description="Parked Elantra AD LKAS11 display test (120-second lease).")
  parser.add_argument("command", choices=["off", "status", *map(str, range(7))])
  args = parser.parse_args(argv)
  directory = Path(directory)
  path = directory / "request.json"
  if args.command == "off":
    path.unlink(missing_ok=True)
    print("SysWarning test mode : OFF (requested)")
    print("Production fallback : next reader poll (normally <=0.5 seconds; stale-reader gate at 2 seconds)")
    return 0
  if args.command == "status":
    now = time.monotonic()
    try:
      with path.open() as f:
        request = valid_request(json.loads(f.read(1024)), now)
    except (OSError, ValueError):
      request = None
    print(f"SysWarning test mode : {'ON (requested)' if request else 'OFF'}")
    print(f"Requested value      : {request[0] if request else 'OFF'}")
    if request:
      print(f"Lease remaining      : {max(0, int(request[1] + LEASE_SECONDS - now))} seconds")
    print("Vehicle gate         : Park + parking brake + stopped + valid CAN + all controls inactive")
    print("Production fallback  : enabled")
    try:
      with (directory / "status.json").open() as f:
        status = json.loads(f.read(1024))
      if (0 <= now - status["updated"] < READER_TIMEOUT
          and status["token"] == (request[2] if request else None)):
        print(f"Controller state     : {status['state']} (cached; not a wire/cluster confirmation)")
      else:
        print("Controller state     : UNKNOWN (waiting/stale/different request)")
    except (OSError, ValueError, KeyError, TypeError):
      print("Controller state     : UNKNOWN (card/Elantra path not confirmed)")
    return 0
  directory.mkdir(mode=0o700, parents=True, exist_ok=True)
  atomic_write(path, {"value": int(args.command), "issued": time.monotonic(), "token": uuid.uuid4().hex})
  print(f"CF_Lkas_SysWarning test = {args.command}")
  print("Stationary-only test requested (120 seconds; refresh by repeating command)")
  print("Park + parking brake + openpilot/AlwaysLateral inactive required")
  print("Check status after 1 second; moving/engaging disarms until a new command")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
