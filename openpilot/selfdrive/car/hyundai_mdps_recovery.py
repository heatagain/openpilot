"""Elantra legacy EPS warning qualification. Never modifies actuator eligibility.

Receive timestamps and original CAN payloads are used before parser coalescing.
The 120 ms deadline is fixed. Extra guards deliberately reject ambiguous cases.
"""
from dataclasses import dataclass

RECOVERY_NS = 120_000_000
FRESH_NS = 100_000_000  # Existing 100 Hz parser's ten-period freshness limit.
PROTECTION_HOLD_NS = 500_000_000
REARM_NS = 3_000_000_000  # One uninterrupted healthy soft-disable horizon.


def supported(CP) -> bool:
  from opendbc.car.hyundai.values import CAR, HyundaiFlags
  return (CP.brand == "hyundai" and CP.carFingerprint == CAR.HYUNDAI_ELANTRA and
          str(CP.steerControlType) == "torque" and bool(CP.flags & HyundaiFlags.LEGACY) and
          not CP.flags & (HyundaiFlags.CANFD | HyundaiFlags.ANGLE_CONTROL))


@dataclass(frozen=True)
class WarningDecision:
  pending: bool = False
  force_fault: bool = False
  elapsed: float = 0.0


def warning_decision(CS, now_ns: int) -> WarningDecision:
  """Consumer watchdog: stale/missing qualification must never extend suppression."""
  status = CS.hyundaiMdpsRecovery
  if not status.available or not status.faultStartMonoTime:
    return WarningDecision()
  elapsed_ns = now_ns - status.faultStartMonoTime
  fresh = 0 <= now_ns - status.sampleMonoTime < FRESH_NS
  safe_to_wait = (fresh and CS.canValid and not CS.canTimeout and not CS.steerFaultPermanent and
                  not CS.vehicleSensorsInvalid and abs(CS.steeringAngleDeg) < 85)
  pending = (status.pending and safe_to_wait and 0 <= elapsed_ns < RECOVERY_NS)
  force = status.forceWarning or (status.pending and not pending)
  return WarningDecision(pending, bool(force), max(0, elapsed_ns) / 1e9 if status.delayed else 0.0)


class MdpsRecovery:
  def __init__(self):
    self.onset = 0
    self.pending = False
    self.warning = False
    self.delayed = False
    self.force_warning = False
    self.last_now = 0
    self.last_guard = -PROTECTION_HOLD_NS
    self.healthy_since = 0
    self.used_grace = False
    self.was_healthy = False
    self.seen: dict[int, int] = {}
    self.counters: dict[int, int] = {}
    self.toi_fault = False
    self.severe = False
    self.active = False
    self.angle = 0.0
    self.request = False
    self.protection_request = False

  def _fresh(self, now: int) -> bool:
    return all(addr in self.seen and 0 <= now - self.seen[addr] < FRESH_NS for addr in (593, 688, 832))

  def _promote(self):
    self.pending = False
    self.warning = bool(self.onset)

  def _advance(self, now: int):
    if self.pending and now - self.onset >= RECOVERY_NS:
      self._promote()

  def update(self, packets, now_ns: int, can_valid: bool) -> dict:
    # A promoted episode is only released on a later update, after actual recovery.
    if self.warning and not self.toi_fault and not self.severe and self.active and self._fresh(now_ns) and can_valid:
      self.warning = False
      self.onset = 0
      self.delayed = False
      self.force_warning = False
    bad = not can_valid or (self.last_now != 0 and now_ns <= self.last_now)
    self.last_now = now_ns
    # Expire before processing late-delivered observations: no retroactive success.
    self._advance(now_ns - 1)  # A recovery received exactly at the deadline may win.
    for timestamp, frames in packets:
      relevant = [(addr, bytes(data)) for addr, data, bus in frames
                  if (addr in (593, 688) and bus == 0) or (addr == 832 and bus == 128)]
      if not relevant:
        continue
      previous_healthy = self.was_healthy
      batch_bad = timestamp > now_ns or now_ns - timestamp >= FRESH_NS
      fault_seen = False
      # Inspect the entire receive batch before accepting recovery. A severe or
      # invalid frame in the same batch dominates any apparent healthy frame.
      for addr, data in relevant:
        size = 5 if addr == 688 else 8
        if len(data) != size or timestamp < self.seen.get(addr, 0):
          batch_bad = True
          continue
        self.seen[addr] = timestamp
        value = int.from_bytes(data, "little")
        if addr == 688:
          self.angle = int.from_bytes(data[:2], "little", signed=True) * 0.1
          if abs(self.angle) >= 85:
            self.last_guard = timestamp
          continue
        count = data[2] if addr == 593 else (value >> 36) & 15
        modulo = 256 if addr == 593 else 16
        checksum = (sum(data) - data[3]) % 256 if addr == 593 else (sum(data[:6]) + data[7]) % 256
        received = data[3] if addr == 593 else data[6]
        if checksum != received or (addr in self.counters and (count - self.counters[addr]) % modulo != 1):
          batch_bad = True
        self.counters[addr] = count
        if addr == 593:
          self.toi_fault = bool(value & (1 << 14))
          self.active = bool(value & (1 << 13))
          # FailStat accompanying ToiFlt is observed; standalone/persistent
          # FailStat is not established as a recoverable transient.
          self.severe = bool(value & ((1 << 11) | (1 << 12) | (1 << 37))) or bool(value & (1 << 15) and not self.toi_fault)
          batch_bad |= self.severe
          fault_seen |= self.toi_fault or self.severe
        else:
          self.request = bool(value & (1 << 27))
          self.protection_request = bool(value & (1 << 28))
          if self.protection_request:
            self.last_guard = timestamp
      bad |= batch_bad
      if batch_bad:
        self.last_guard = now_ns
      guarded = bad or not self._fresh(timestamp) or timestamp - self.last_guard <= PROTECTION_HOLD_NS
      if fault_seen and not self.onset:
        self.onset = timestamp
        self.pending = previous_healthy and not guarded and not self.used_grace
        self.delayed = self.pending
        self.warning = not self.pending
        self.force_warning = batch_bad
        self.used_grace = True
        self.healthy_since = 0
      if self.pending:
        if guarded:
          self._promote()
        elif timestamp - self.onset > RECOVERY_NS:
          self._promote()
        elif not self.toi_fault and not self.severe and self.active:
          self.pending = False
          self.onset = 0
          self.delayed = False
        else:
          self._advance(timestamp)
      if self.warning and batch_bad:
        self.force_warning = True
      self.was_healthy = (not guarded and not self.toi_fault and not self.severe and
                          self.active and self.request and not self.protection_request)

    healthy = can_valid and not bad and self._fresh(now_ns) and not self.toi_fault and not self.severe and self.active
    if not healthy:
      self.healthy_since = 0
      self.was_healthy = False
      if bad or not self._fresh(now_ns):
        self.last_guard = now_ns
        if self.pending:
          self._promote()
    else:
      if not self.healthy_since:
        self.healthy_since = now_ns
      if now_ns - self.healthy_since >= REARM_NS:
        self.used_grace = False
    self._advance(now_ns)
    return {"available": True, "pending": self.pending, "warning": self.warning,
            "faultStartMonoTime": self.onset, "sampleMonoTime": now_ns, "delayed": self.delayed,
            "forceWarning": self.warning and (self.delayed or self.force_warning)}
