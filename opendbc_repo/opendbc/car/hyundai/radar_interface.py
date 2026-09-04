import copy
import math
import os
import time
from collections import Counter, deque
from dataclasses import dataclass, field
from itertools import combinations
from numbers import Integral, Real
from typing import Sequence

import numpy as np

from opendbc import DBC_PATH
from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.carlog import carlog
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.hyundai.values import DBC, HyundaiFlags, HyundaiExtFlags
from openpilot.common.params import Params
from opendbc.car.hyundai.hyundaicanfd import CanBus

SCC_TID = 0
RADAR_START_ADDR = 0x500
RADAR_MSG_COUNT = 64
RADAR_REQUIRED_MSG_COUNT = 32
RADAR_MSG_COUNT4 = 8
RADAR_GROUP4_MAX_LONG_DIST = 325.0
RADAR_GROUP4_MAX_YREL = 6.0
RADAR_START_ADDR_CANFD1 = 0x210
RADAR_MSG_COUNT1 = 16
RADAR_START_ADDR_CANFD2 = 0x3A5  # Group 2; Group 1 uses two 0x210 messages. Pending validation.
RADAR_MSG_COUNT2 = 32
RADAR_START_ADDR_CANFD3 = 0x400
RADAR_MSG_COUNT3 = 30
CORNER_OBJECT_235_START_ADDR = 0x235
CORNER_OBJECT_235_MSG_COUNT = 20
CORNER_OBJECT_235_TRACK_ID_OFFSET = 200
CORNER_OBJECT_235_DBC = 'hyundai_canfd_corner_radar_235_generated'
CORNER_OBJECT_180_START_ADDR = 0x180
CORNER_OBJECT_180_MSG_COUNT = 5
CORNER_OBJECT_180_SLOTS_PER_MSG = 2
CORNER_OBJECT_180_TRACK_ID_OFFSET = 240
CORNER_OBJECT_180_DBC = 'hyundai_canfd_corner_radar_180_generated'
CORNER_OBJECT_430_LEFT_START_ADDR = 0x430
CORNER_OBJECT_430_RIGHT_START_ADDR = 0x440
CORNER_OBJECT_430_MSG_COUNT_PER_SIDE = 8
CORNER_OBJECT_430_SLOTS_PER_MSG = 7
CORNER_OBJECT_430_TRACK_ID_OFFSET = 300
CORNER_OBJECT_430_DBC = 'hyundai_canfd_corner_radar_430_generated'


def canfd_group2_track_status(msg):
  """Return the existing age gate and the radar-native object state."""
  return msg['VALID_CNT'] > 10, int(msg['VALID'])


CORNER_OBJECT_430_EMPTY_RAW_VALUES = (0x010d1f40, 0x00010d1f)
CORNER_OBJECT_430_DEFAULT_DISTANCE_RAW_MIN = 2520  # 126.0 m
CORNER_OBJECT_430_DEFAULT_DISTANCE_RAW_MAX = 2600  # 130.0 m
CORNER_OBJECT_430_MAX_DREL = 120.0
CORNER_OBJECT_430_MAX_TRACKS_PER_SIDE = 4
CORNER_OBJECT_430_DT = 0.05
CORNER_OBJECT_430_MAX_DREL_DELTA = 1.5
CORNER_OBJECT_430_CANDIDATE_META_BYTE_3 = (2,)
CORNER_OBJECT_430_CANDIDATE_EXCLUDED_SLOTS = (1,)
CORNER_OBJECT_430_CANDIDATE_RAW_DELTA = 200
CORNER_OBJECT_430_STRONG_META_BYTE_2 = (10,)
CORNER_OBJECT_430_WEAK_META_BYTE_2 = (5, 6, 7, 8, 9)
CORNER_OBJECT_430_STRONG_MIN_SUPPORT = 2
CORNER_OBJECT_430_WEAK_MIN_SUPPORT = 3
CORNER_OBJECT_430_CLUSTER_RAW_GAP = 200
CORNER_OBJECT_430_TRACK_MATCH_MAX_DREL_DELTA = 3.0
CORNER_OBJECT_430_MAX_ABS_VREL = 20.0
CORNER_OBJECT_430_MAX_ABS_YVREL = 3.0
CORNER_OBJECT_430_VREL_ALPHA = 0.35
CORNER_OBJECT_430_YVREL_ALPHA = 0.35
CORNER_OBJECT_430_LATERAL_CELL_MSG_WEIGHT = 0.35
CORNER_OBJECT_430_LATERAL_CELL_SLOT_WEIGHT = 0.65
CORNER_OBJECT_430_YREL_OFFSET = 5.8
CORNER_OBJECT_430_YREL_SCALE = 1.1
CORNER_OBJECT_430_RIGHT_CELL_MIRROR = 7.0
CORNER_OBJECT_430_MIN_ABS_YREL = 0.8
CORNER_OBJECT_430_MAX_ABS_YREL = 4.2
CORNER_OBJECT_430_HISTORY_SIZE = 8
CORNER_OBJECT_430_MIN_HISTORY = 5
CORNER_OBJECT_430_MIN_INWARD_YREL_DELTA = 0.35
CORNER_OBJECT_430_MIN_RECENT_INWARD_YREL_DELTA = 0.05
CORNER_OBJECT_430_MIN_INWARD_RATIO = 0.65
CORNER_OBJECT_430_INWARD_CENTER_ABS_YREL = 1.55
CORNER_OBJECT_430_INWARD_KEEP_YVREL_ABS_YREL = 2.2
CORNER_OBJECT_430_EARLY_INWARD_NONCENTER_FRAMES = 2
CORNER_OBJECT_430_SIDE_KEEP_ABS_YREL = 2.0
CORNER_OBJECT_STABLE_TRACK_ID_START = 1000
CORNER_OBJECT_IDENTITY_STALE_CYCLES = 3
CORNER_OBJECT_IDENTITY_MAX_DREL_DELTA = 7.0
CORNER_OBJECT_IDENTITY_MAX_YREL_DELTA = 3.2
CORNER_OBJECT_HANDOFF_MAX_DREL_DELTA = 2.0
CORNER_OBJECT_HANDOFF_MAX_YREL_DELTA = 1.0
CORNER_OBJECT_HANDOFF_MAX_VREL_DELTA = 3.0
CORNER_SIDE_OBJECT_MAX_DREL = 0.2
CORNER_SIDE_OBJECT_MIN_ABS_YREL = 1.4
CORNER_SIDE_OBJECT_MAX_ABS_YREL = 4.5

# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/


class CornerObjectTrackIdManager:
  def __init__(self):
    self.next_track_id = CORNER_OBJECT_STABLE_TRACK_ID_START
    self.source_cycles: dict[str, int] = {}
    self.track_states: dict[tuple[str, int], tuple[int, int, int, float, float, int]] = {}

  def clear_source(self, source: str):
    self.track_states = {key: value for key, value in self.track_states.items() if key[0] != source}
    self.source_cycles.pop(source, None)

  def get_track_ids(self, source: str, candidates) -> dict[int, int]:
    cycle = self.source_cycles.get(source, 0) + 1
    self.source_cycles[source] = cycle
    previous = {
      track_id: state for (state_source, track_id), state in self.track_states.items()
      if state_source == source and cycle - state[5] <= CORNER_OBJECT_IDENTITY_STALE_CYCLES
    }
    used_track_ids = set()
    assignments = {}

    # Prefer the previous CAN slot, then permit a physically continuous slot
    # handoff. Object IDs are not globally unique: two distant objects can use
    # the same ID at the same time.
    for candidate in candidates:
      slot_id, object_id, age, _, d_rel, y_rel, *_ = candidate
      matches = []
      for track_id, state in previous.items():
        previous_slot, previous_object_id, previous_age, previous_d_rel, previous_y_rel, _ = state
        if track_id in used_track_ids or object_id != previous_object_id or age < previous_age:
          continue
        d_delta = abs(d_rel - previous_d_rel)
        y_delta = abs(y_rel - previous_y_rel)
        if d_delta > CORNER_OBJECT_IDENTITY_MAX_DREL_DELTA or y_delta > CORNER_OBJECT_IDENTITY_MAX_YREL_DELTA:
          continue
        matches.append((previous_slot != slot_id, d_delta + y_delta * 1.5, track_id))

      if matches:
        track_id = min(matches)[2]
      else:
        track_id = self.next_track_id
        self.next_track_id += 1
      assignments[slot_id] = track_id
      used_track_ids.add(track_id)
      self.track_states[(source, track_id)] = (slot_id, object_id, age, d_rel, y_rel, cycle)

    self.track_states = {
      key: state for key, state in self.track_states.items()
      if key[0] != source or cycle - state[5] <= CORNER_OBJECT_IDENTITY_STALE_CYCLES
    }
    return assignments


def deduplicate_corner_candidates(candidates):
  objects = []
  for candidate in candidates:
    _, object_id, age, quality, d_rel, y_rel, v_rel, *_ = candidate
    duplicate_index = None
    for index, previous in enumerate(objects):
      if object_id != previous[1]:
        continue
      if (abs(d_rel - previous[4]) <= CORNER_OBJECT_HANDOFF_MAX_DREL_DELTA and
          abs(y_rel - previous[5]) <= CORNER_OBJECT_HANDOFF_MAX_YREL_DELTA and
          abs(v_rel - previous[6]) <= CORNER_OBJECT_HANDOFF_MAX_VREL_DELTA):
        duplicate_index = index
        break
    if duplicate_index is None:
      objects.append(candidate)
    elif (age, quality) > (objects[duplicate_index][2], objects[duplicate_index][3]):
      objects[duplicate_index] = candidate
  return objects


def corner_object_position_valid(d_rel: float, y_rel: float) -> bool:
  normal_object = 0.2 < d_rel < 180.0
  clipped_side_object = (
    0.0 <= d_rel <= CORNER_SIDE_OBJECT_MAX_DREL and
    CORNER_SIDE_OBJECT_MIN_ABS_YREL <= abs(y_rel) <= CORNER_SIDE_OBJECT_MAX_ABS_YREL
  )
  return (normal_object or clipped_side_object) and abs(y_rel) < 40.0


def get_radar_can_parser(CP, radar_tracks, msg_start_addr, msg_count, required_msg_count, radar_group4=False):
  if not radar_tracks:
    return None
  #if Bus.radar not in DBC[CP.carFingerprint]:
  #  return None
  print("RadarInterface: RadarTracks...")

  if CP.flags & HyundaiFlags.CANFD:
    CAN = CanBus(CP)
    messages = [(f"RADAR_TRACK_{addr:x}", 20) for addr in range(msg_start_addr, msg_start_addr + msg_count)]
    return CANParser('hyundai_canfd_radar_generated', messages, CAN.ACAN)
  else:
    # Legacy Mando radars expose either 32 or 64 consecutive slots. Keep the
    # first 32 mandatory for timing/CAN validity and accept the upper bank when
    # present, so a 32-slot radar remains fully compatible.
    messages = [(f"RADAR_TRACK_{addr:x}", 20 if index < required_msg_count else math.nan)
                for index, addr in enumerate(range(msg_start_addr, msg_start_addr + msg_count))]
  #return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 1)
    dbc_name = 'hyundai_kia_denso_front_radar_generated' if radar_group4 else 'hyundai_kia_mando_front_radar_generated'
    return CANParser(dbc_name, messages, 1)

def get_corner_object_can_parser(CP, enabled):
  if not enabled or not (CP.flags & HyundaiFlags.CANFD):
    return None

  dbc_path = os.path.join(DBC_PATH, f"{CORNER_OBJECT_235_DBC}.dbc")
  if not os.path.exists(dbc_path):
    print(f"RadarInterface: missing {CORNER_OBJECT_235_DBC}.dbc, 0x235 corner radar disabled")
    return None

  CAN = CanBus(CP)
  messages = [(f"CORNER_RADAR_235_OBJECTS_{addr:x}", 33) for addr in range(CORNER_OBJECT_235_START_ADDR, CORNER_OBJECT_235_START_ADDR + CORNER_OBJECT_235_MSG_COUNT)]
  return CANParser(CORNER_OBJECT_235_DBC, messages, CAN.ACAN)

def get_corner_object_180_can_parser(CP, enabled):
  if not enabled or not (CP.flags & HyundaiFlags.CANFD):
    return None

  dbc_path = os.path.join(DBC_PATH, f"{CORNER_OBJECT_180_DBC}.dbc")
  if not os.path.exists(dbc_path):
    print(f"RadarInterface: missing {CORNER_OBJECT_180_DBC}.dbc, 0x180 corner radar disabled")
    return None

  CAN = CanBus(CP)
  messages = [(f"CORNER_RADAR_180_OBJECTS_{addr:x}", 33) for addr in range(CORNER_OBJECT_180_START_ADDR, CORNER_OBJECT_180_START_ADDR + CORNER_OBJECT_180_MSG_COUNT)]
  return CANParser(CORNER_OBJECT_180_DBC, messages, CAN.ACAN)

def get_corner_object_430_can_parser(CP, enabled):
  if not enabled or not (CP.flags & HyundaiFlags.CANFD):
    return None

  dbc_path = os.path.join(DBC_PATH, f"{CORNER_OBJECT_430_DBC}.dbc")
  if not os.path.exists(dbc_path):
    print(f"RadarInterface: missing {CORNER_OBJECT_430_DBC}.dbc, 0x430/0x440 corner radar disabled")
    return None

  CAN = CanBus(CP)
  messages = [(f"CORNER_RADAR_430_OBJECTS_{addr:x}", 33) for addr in range(CORNER_OBJECT_430_LEFT_START_ADDR, CORNER_OBJECT_430_LEFT_START_ADDR + CORNER_OBJECT_430_MSG_COUNT_PER_SIDE)]
  messages += [(f"CORNER_RADAR_430_OBJECTS_{addr:x}", 33) for addr in range(CORNER_OBJECT_430_RIGHT_START_ADDR, CORNER_OBJECT_430_RIGHT_START_ADDR + CORNER_OBJECT_430_MSG_COUNT_PER_SIDE)]
  return CANParser(CORNER_OBJECT_430_DBC, messages, CAN.ACAN)

def get_radar_can_parser_scc(CP):
  CAN = CanBus(CP)
  if CP.flags & HyundaiFlags.CANFD:
    messages = [("SCC_CONTROL", 50)]
    bus = CAN.ECAN
  else:
    messages = [("SCC11", 50)]
    bus = CAN.ECAN

  print("$$$$$$$$ ECAN = ", CAN.ECAN)    
  bus = CAN.CAM if CP.flags & HyundaiFlags.CAMERA_SCC else bus
  return CANParser(DBC[CP.carFingerprint][Bus.pt], messages, bus)

# Bosch MRRevo14F passive radar
# Raw return IDs and physical object IDs are distinct. Only physical
# observations are emitted; 0x601 supplies selected-member metadata.

BOSCH_INACTIVE_WORD = 0x40100000
BOSCH_MAX_RAW_TRACK_ID = 2**31 - 1
BOSCH_TRACK_ADDRESSES = frozenset(range(0x602, 0x612))
BOSCH_WINDOW_NS = 20_000_000
BOSCH_STALE_NS = 300_000_000
BOSCH_OUTPUT_INTERVAL_NS = 100_000_000
BOSCH_SAMPLE_HOLD_NS = 150_000_000  # one 10 Hz observation period plus one SCC publication period


def bosch_numpy_linear_sum_assignment(cost_matrix):
  cost = np.asarray(cost_matrix, dtype=float)
  if cost.ndim != 2:
    raise ValueError('expected a matrix')
  if np.isnan(cost).any() or np.isneginf(cost).any():
    raise ValueError('matrix contains invalid numeric entries')
  transposed = cost.shape[0] > cost.shape[1]
  if transposed:
    cost = cost.T
  n, m = cost.shape
  if not n:
    return np.empty(0, dtype=int), np.empty(0, dtype=int)
  u, v = np.zeros(n + 1), np.zeros(m + 1)
  p, way = np.zeros(m + 1, dtype=int), np.zeros(m + 1, dtype=int)
  for row in range(1, n + 1):
    p[0] = row
    minimum = np.full(m + 1, np.inf)
    used = np.zeros(m + 1, dtype=bool)
    column = 0
    while True:
      used[column] = True
      current_row = p[column]
      remaining = np.flatnonzero(~used[1:]) + 1
      reduced = cost[current_row - 1, remaining - 1] - u[current_row] - v[remaining]
      improve = reduced < minimum[remaining]
      improved_columns = remaining[improve]
      minimum[improved_columns] = reduced[improve]
      way[improved_columns] = column
      next_column = remaining[np.argmin(minimum[remaining])]
      delta = minimum[next_column]
      if not np.isfinite(delta):
        raise ValueError('cost matrix is infeasible')
      u[p[used]] += delta
      v[used] -= delta
      minimum[~used] -= delta
      column = next_column
      if p[column] == 0:
        break
    while column:
      previous_column = way[column]
      p[column] = p[previous_column]
      column = previous_column
  columns = np.flatnonzero(p[1:])
  rows = p[columns + 1] - 1
  if transposed:
    rows, columns = columns, rows
  order = np.argsort(rows)
  return rows[order], columns[order]


try:
  from scipy.optimize import linear_sum_assignment as bosch_linear_sum_assignment
except ImportError:
  bosch_linear_sum_assignment = bosch_numpy_linear_sum_assignment


def _bosch_integer(value: object, label: str, minimum: int, maximum: int | None = None) -> None:
  if isinstance(value, bool) or not isinstance(value, Integral) or value < minimum or (maximum is not None and value > maximum):
    raise ValueError(f"{label} must be an integer in [{minimum}, {maximum}]")


def _bosch_finite(value: object, label: str) -> None:
  if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
    raise ValueError(f"{label} must be finite")


@dataclass(frozen=True)
class BoschRawDetection:
  timestamp_ns: int
  slot: int
  d_rel: float
  y_rel: float
  v_rel: float
  raw_word: int = 0

  def __post_init__(self) -> None:
    _bosch_integer(self.timestamp_ns, "timestamp_ns", 0)
    _bosch_integer(self.slot, "slot", 0, 31)
    _bosch_integer(self.raw_word, "raw_word", 0, 2**32 - 1)
    for name in ("d_rel", "y_rel", "v_rel"):
      _bosch_finite(getattr(self, name), name)
    if not 0 <= self.d_rel <= 255.75:
      raise ValueError("d_rel outside the supported Bosch decoded range")
    if not -32 <= self.y_rel <= 31.96875:
      raise ValueError("y_rel outside the supported Bosch decoded range")
    if not -128 <= self.v_rel <= 127.75:
      raise ValueError("v_rel outside the supported Bosch decoded range")
    if self.raw_word == BOSCH_INACTIVE_WORD or self.raw_word & (1 << 31):
      raise ValueError("empty or unsupported bit31 record is not a RawDetection")

  @property
  def address(self) -> int:
    return 0x602 + self.slot // 2

  @property
  def half(self) -> int:
    return self.slot % 2


@dataclass(frozen=True)
class BoschRawTrackingConfig:
  distance_gate_m: float = 3.5
  speed_gate_mps: float = 2.0
  bearing_near_deg: float = 12.0
  bearing_mid_deg: float = 5.0
  bearing_far_deg: float = 2.5
  bearing_distant_deg: float = 2.0
  coast_s: float = 0.3
  same_slot_bonus: float = 0.03
  unmatched_cost: float = 0.65

  def __post_init__(self) -> None:
    for name in self.__dataclass_fields__:
      value = getattr(self, name)
      _bosch_finite(value, name)
      if name == "same_slot_bonus":
        if not 0 <= value <= 0.1:
          raise ValueError("same_slot_bonus must remain a weak preference in [0, 0.1]")
      elif value <= 0:
        raise ValueError(f"{name} must be positive")
    for name in ("bearing_near_deg", "bearing_mid_deg", "bearing_far_deg", "bearing_distant_deg"):
      if getattr(self, name) > 180:
        raise ValueError(f"{name} must be <= 180 degrees")

  def bearing_gate_rad(self, distance_m: float) -> float:
    if distance_m < 15:
      degrees = self.bearing_near_deg
    elif distance_m < 30:
      degrees = self.bearing_mid_deg
    elif distance_m < 60:
      degrees = self.bearing_far_deg
    else:
      degrees = self.bearing_distant_deg
    return math.radians(degrees)


@dataclass(frozen=True)
class BoschRawTrack:
  raw_track_id: int
  detection: BoschRawDetection
  age_scans: int
  recovered: bool = False
  previous_slot: int | None = None

  @property
  def timestamp_ns(self) -> int:
    return self.detection.timestamp_ns

  @property
  def slot(self) -> int:
    return self.detection.slot

  @property
  def address(self) -> int:
    return self.detection.address

  @property
  def half(self) -> int:
    return self.detection.half

  @property
  def d_rel(self) -> float:
    return self.detection.d_rel

  @property
  def y_rel(self) -> float:
    return self.detection.y_rel

  @property
  def v_rel(self) -> float:
    return self.detection.v_rel


@dataclass
class _BoschRawState:
  track: BoschRawTrack
  # Association-only state aligned to the last update's scan timestamp.
  x: float
  y: float
  last_seen_scan_ns: int
  last_seen_update: int


def _bosch_advance(x: float, y: float, velocity: float, dt: float, yaw_rate: float | None) -> tuple[float, float]:
  x += velocity * dt
  angle = 0.0 if yaw_rate is None else yaw_rate * dt
  c, s = math.cos(angle), math.sin(angle)
  return c * x + s * y, -s * x + c * y


class BoschRawTrackManager:
  """Optimal one-to-one association across all slots with internal coasting.

  update() requires strictly increasing scan timestamps. Each detection may be
  slightly older than its scan availability timestamp, but must be newer than
  the preceding scan. Its state is projected to scan time only for association;
  the returned detection always contains the original, unsmoothed observation.
  age_scans counts matched observations (not elapsed or coasted scans).

  Missing tracks remain eligible through coast_s inclusive; they are never
  emitted. Slots influence cost weakly and cannot override any hard gate.
  Rejected arguments and ID exhaustion leave manager state unchanged.
  """

  def __init__(self, config: BoschRawTrackingConfig | None = None, *, first_id: int = 1) -> None:
    self.config = config if config is not None else BoschRawTrackingConfig()
    if not isinstance(self.config, BoschRawTrackingConfig):
      raise TypeError("config must be RawTrackingConfig")
    _bosch_integer(first_id, "first_id", 1, BOSCH_MAX_RAW_TRACK_ID)
    self.next_id = int(first_id)
    self.last_timestamp_ns: int | None = None
    self._update_count = 0
    self._states: dict[int, _BoschRawState] = {}
    self.stats = {name: 0 for name in (
      "updates", "detections", "created", "deleted", "assignments",
      "cross_slot", "recovered", "coasted_track_scans", "max_active",
      "gated_pairs", "distance_rejections", "speed_rejections", "bearing_rejections",
      "yaw_compensated_updates", "yaw_unavailable_updates",
    )}

  @property
  def active_count(self) -> int:
    """Internal live plus coasted return hypotheses; not an output count."""
    return len(self._states)

  def update(self, timestamp_ns: int, detections: Sequence[BoschRawDetection], yaw_rate: float | None = None) -> tuple[BoschRawTrack, ...]:
    _bosch_integer(timestamp_ns, "timestamp_ns", 0)
    if self.last_timestamp_ns is not None and timestamp_ns <= self.last_timestamp_ns:
      raise ValueError("scan timestamps must be strictly increasing")
    if yaw_rate is not None:
      _bosch_finite(yaw_rate, "yaw_rate")
    current = tuple(detections)
    if len(current) > 32:
      raise ValueError("a Bosch scan cannot contain more than 32 detections")
    slots = set()
    for detection in current:
      if not isinstance(detection, BoschRawDetection):
        raise TypeError("detections must contain RawDetection instances")
      if detection.timestamp_ns > timestamp_ns:
        raise ValueError("detection timestamp is later than scan availability")
      if self.last_timestamp_ns is not None and detection.timestamp_ns <= self.last_timestamp_ns:
        raise ValueError("stale/repeated detection cannot be tracked as fresh")
      if detection.slot in slots:
        raise ValueError("a scan must contain at most one detection per slot")
      slots.add(detection.slot)
    current = tuple(sorted(current, key=lambda point: point.slot))
    config = self.config
    coast_ns = round(config.coast_s * 1e9)
    retained = [state for _, state in sorted(self._states.items())
          if timestamp_ns - state.last_seen_scan_ns <= coast_ns]
    expired_count = len(self._states) - len(retained)
    dt = 0.0 if self.last_timestamp_ns is None else (timestamp_ns - self.last_timestamp_ns) / 1e9
    predicted = [_bosch_advance(state.x, state.y, state.track.v_rel, dt, yaw_rate) for state in retained]
    observed = [_bosch_advance(point.d_rel, point.y_rel, point.v_rel,
              (timestamp_ns - point.timestamp_ns) / 1e9, yaw_rate) for point in current]
    n, m = len(retained), len(current)
    # Explicit births and misses keep a feasible solution even if every real
    # edge is forbidden. Never solve then discard ungated forced matches.
    costs = np.full((n + m, n + m), np.inf)
    pending_stats = {"gated_pairs": 0, "distance_rejections": 0, "speed_rejections": 0, "bearing_rejections": 0}
    for row, (state, (pred_x, pred_y)) in enumerate(zip(retained, predicted)):
      costs[row, m + row] = config.unmatched_cost
      for col, (point, (obs_x, obs_y)) in enumerate(zip(current, observed)):
        delta_d = abs(obs_x - pred_x)
        delta_v = abs(point.v_rel - state.track.v_rel)
        angle = math.atan2(obs_y, obs_x) - math.atan2(pred_y, pred_x)
        delta_bearing = abs(math.atan2(math.sin(angle), math.cos(angle)))
        gate = config.bearing_gate_rad(max(0.0, min(pred_x, obs_x)))
        if delta_d > config.distance_gate_m:
          pending_stats["distance_rejections"] += 1
        elif delta_v > config.speed_gate_mps:
          pending_stats["speed_rejections"] += 1
        elif delta_bearing > gate:
          pending_stats["bearing_rejections"] += 1
        else:
          pending_stats["gated_pairs"] += 1
          cost = ((delta_d / config.distance_gate_m)**2 + (delta_v / config.speed_gate_mps)**2 + (delta_bearing / gate)**2) / 3
          if state.track.slot == point.slot:
            cost = max(0.0, cost - config.same_slot_bonus)
          costs[row, col] = cost
    for col in range(m):
      costs[n + col, col] = config.unmatched_cost
    costs[n:, m:] = 0.0
    assignment = {}
    if n + m:
      rows, columns = bosch_linear_sum_assignment(costs)
      assignment = {int(col): int(row) for row, col in zip(rows, columns) if row < n and col < m}
    births = m - len(assignment)
    if self.next_id + births - 1 > BOSCH_MAX_RAW_TRACK_ID:
      raise OverflowError("raw return Int32 ID space exhausted; IDs must not wrap or be reused")

    # Commit only after all validation and assignment have succeeded.
    self._update_count += 1
    self._states = {}
    for state, (x, y) in zip(retained, predicted):
      self._states[state.track.raw_track_id] = _BoschRawState(state.track, x, y, state.last_seen_scan_ns, state.last_seen_update)
    output = []
    for col, (point, (x, y)) in enumerate(zip(current, observed)):
      if col in assignment:
        old = retained[assignment[col]]
        recovered = old.last_seen_update != self._update_count - 1
        track = BoschRawTrack(old.track.raw_track_id, point, old.track.age_scans + 1, recovered, old.track.slot)
        self.stats["assignments"] += 1
        self.stats["cross_slot"] += int(point.slot != old.track.slot)
        self.stats["recovered"] += int(recovered)
      else:
        track = BoschRawTrack(self.next_id, point, 1)
        self.next_id += 1
        self.stats["created"] += 1
      output.append(track)
      self._states[track.raw_track_id] = _BoschRawState(track, x, y, timestamp_ns, self._update_count)
    self.last_timestamp_ns = int(timestamp_ns)
    for key, value in pending_stats.items():
      self.stats[key] += value
    self.stats["updates"] += 1
    self.stats["detections"] += m
    self.stats["deleted"] += expired_count
    self.stats["coasted_track_scans"] += n - len(assignment)
    self.stats["max_active"] = max(self.stats["max_active"], len(self._states))
    self.stats["yaw_compensated_updates" if yaw_rate is not None else "yaw_unavailable_updates"] += 1
    return tuple(output)


@dataclass(frozen=True)
class BoschGroupingConfig:
  distance_diameter_m: float = 3.0
  lateral_diameter_m: float = 1.5
  velocity_diameter_mps: float = 1.5
  min_pair_observations: int = 3
  min_pair_span_s: float = .18
  pair_max_gap_s: float = .16
  evidence_window_s: float = .8
  # Reject expanding separation, not a return settling toward an existing one.
  max_relative_distance_growth_m: float = 1.0
  max_relative_lateral_growth_m: float = .75
  stationary_speed_mps: float = .6
  moving_speed_mps: float = 1.4
  max_members: int = 8
  coast_s: float = .3
  first_physical_id: int = 1_000_000

  def __post_init__(self):
    for name, value in vars(self).items():
      if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be positive and finite')
    if self.min_pair_observations < 2 or int(self.min_pair_observations) != self.min_pair_observations:
      raise ValueError('at least two integer observations are required')
    if not 2 <= self.max_members <= 32 or int(self.max_members) != self.max_members:
      raise ValueError('max_members must be an integer in [2,32]')
    if self.stationary_speed_mps >= self.moving_speed_mps:
      raise ValueError('kinematic stationary and moving bands must be disjoint')
    if self.min_pair_span_s > self.evidence_window_s:
      raise ValueError('evidence window must contain the minimum pair span')
    if not isinstance(self.first_physical_id, int) or self.first_physical_id >= 2**31:
      raise ValueError('physical IDs must fit positive downstream Int32')


@dataclass(frozen=True)
class BoschVisionCue:
  """Association support only. No model horizon is treated as a new object ID."""
  d_rel: float
  y_rel: float
  probability: float
  distance_tolerance_m: float = 8.
  lateral_tolerance_m: float = 1.5


@dataclass(frozen=True)
class BoschPhysicalObject:
  physical_track_id: int
  timestamp_ns: int
  members: tuple[BoschRawTrack, ...]
  representative_raw_track_id: int
  d_rel: float
  y_rel: float
  v_rel: float
  oem_selected: bool
  vision_supported: bool
  age_scans: int
  grouping_evidence: str

  @property
  def member_slots(self):
    return tuple(m.slot for m in self.members)


@dataclass
class _BoschPairEvidence:
  samples: deque = field(default_factory=deque)


@dataclass
class _BoschPhysicalState:
  observation: BoschPhysicalObject
  # Includes temporarily missing raw members, for internal coasting only.
  member_last_seen: dict[int, int]


class BoschObjectGroupManager:
  """Temporal complete-link grouping followed by global physical-ID assignment.

  Every pair in a cluster must satisfy the full geometry, motion-band and
  temporal evidence constraints. There is no transitive single-link merge.
  Existing raw membership and representative continuity preserve the physical
  ID. OEM selection never establishes identity or creates an extra point.
  """
  def __init__(self, config: BoschGroupingConfig | None = None):
    self.config = config or BoschGroupingConfig()
    self.next_id = self.config.first_physical_id
    self.now_ns = None
    self.pairs: dict[tuple[int, int], _BoschPairEvidence] = {}
    self.states: dict[int, _BoschPhysicalState] = {}
    self.stats = Counter()
    self.last_diagnostics = []

  def _geometry(self, a, b, v_ego):
    c = self.config
    if abs(a.d_rel - b.d_rel) > c.distance_diameter_m:
      return False, 'distance_diameter'
    if abs(a.y_rel - b.y_rel) > c.lateral_diameter_m:
      return False, 'lateral_diameter'
    if abs(a.v_rel - b.v_rel) > c.velocity_diameter_mps:
      return False, 'velocity_diameter'
    if math.isfinite(v_ego):
      speeds = sorted((abs(a.v_rel + v_ego), abs(b.v_rel + v_ego)))
      if speeds[0] <= c.stationary_speed_mps and speeds[1] >= c.moving_speed_mps:
        return False, 'stationary_moving_conflict'
    return True, 'geometry'

  @staticmethod
  def _vision_support(member, vision):
    return any(math.isfinite(c.d_rel) and math.isfinite(c.y_rel) and c.probability >= .7
         and abs(member.d_rel - c.d_rel) <= c.distance_tolerance_m
         and abs(member.y_rel - c.y_rel) <= c.lateral_tolerance_m for c in vision)

  def update(self, timestamp_ns: int, raw_tracks: Sequence[BoschRawTrack], *,
       yaw_rate: float | None = None, v_ego: float = math.nan,
       oem_slot: int | None = None, vision: Sequence[BoschVisionCue] = ()) -> tuple[BoschPhysicalObject, ...]:
    if isinstance(timestamp_ns, bool) or not isinstance(timestamp_ns, int) or timestamp_ns < 0:
      raise ValueError('timestamp must be nonnegative integer nanoseconds')
    if self.now_ns is not None and timestamp_ns <= self.now_ns:
      raise ValueError('group updates require strictly increasing scan timestamps')
    if yaw_rate is not None and (isinstance(yaw_rate, bool) or not math.isfinite(yaw_rate)):
      raise ValueError('yaw_rate must be finite or None')
    if isinstance(v_ego, bool) or not isinstance(v_ego, (int, float)) or math.isinf(v_ego):
      raise ValueError('v_ego must be finite or NaN unknown')
    raw_tracks = tuple(raw_tracks)
    if len(raw_tracks) > 32 or not all(isinstance(r, BoschRawTrack) for r in raw_tracks):
      raise ValueError('at most 32 fresh RawTrack observations are required')
    if len({r.slot for r in raw_tracks}) != len(raw_tracks):
      raise ValueError('duplicate raw slot')
    for r in raw_tracks:
      if (r.timestamp_ns > timestamp_ns or
        timestamp_ns-r.timestamp_ns > round(self.config.coast_s*1e9) or
        (self.now_ns is not None and r.timestamp_ns <= self.now_ns)):
        raise ValueError('stale, repeated or future raw observation')
    if len({r.raw_track_id for r in raw_tracks}) != len(raw_tracks):
      raise ValueError('duplicate raw track ID')
    # ID exhaustion is exceptional. Stage that rare update separately so an
    # overflow cannot partially alter history, ownership, timestamps or stats.
    if self.next_id+len(raw_tracks) > 2**31:
      staged = copy.deepcopy(self)
      result = staged._update(timestamp_ns, raw_tracks, yaw_rate=yaw_rate, v_ego=v_ego,
                  oem_slot=oem_slot, vision=vision)
      self.__dict__.update(staged.__dict__)
      return result
    return self._update(timestamp_ns, raw_tracks, yaw_rate=yaw_rate, v_ego=v_ego,
              oem_slot=oem_slot, vision=vision)

  def _update(self, timestamp_ns, raw_tracks, *, yaw_rate=None, v_ego=math.nan, oem_slot=None, vision=()):
    raw_tracks = tuple(sorted(raw_tracks, key=lambda r: r.raw_track_id))
    self.now_ns = timestamp_ns
    c = self.config
    for pid in list(self.states):
      state = self.states[pid]
      if timestamp_ns - state.observation.timestamp_ns > round(c.coast_s * 1e9):
        del self.states[pid]
        self.stats['deleted'] += 1
      else:
        state.member_last_seen = {rid: ns for rid, ns in state.member_last_seen.items()
                     if timestamp_ns - ns <= round(c.coast_s * 1e9)}
    for pair in list(self.pairs):
      if timestamp_ns - self.pairs[pair].samples[-1][0] > c.evidence_window_s * 1e9:
        del self.pairs[pair]

    compatible = set()
    self.last_diagnostics = []
    for a, b in combinations(raw_tracks, 2):
      key = (a.raw_track_id, b.raw_track_id)
      geometry, reason = self._geometry(a, b, v_ego)
      if not geometry:
        self.pairs.pop(key, None)
        self.stats['pair_rejected_' + reason] += 1
        continue
      evidence = self.pairs.setdefault(key, _BoschPairEvidence())
      if evidence.samples and timestamp_ns - evidence.samples[-1][0] > c.pair_max_gap_s * 1e9:
        evidence.samples.clear()
      evidence.samples.append((timestamp_ns, a.d_rel-b.d_rel, a.y_rel-b.y_rel))
      while timestamp_ns - evidence.samples[0][0] > c.evidence_window_s * 1e9:
        evidence.samples.popleft()
      ds = [s[1] for s in evidence.samples]
      ys = [s[2] for s in evidence.samples]
      # The segment15 flatbed's new return converges from 1.25m to .4m
      # lateral separation. A max-minus-min history test split it because
      # of OLD larger separation despite current geometry improving.
      # Test widening from a recent minimum instead; complete-link still
      # imposes the absolute d/y/v diameter on every current member pair.
      stable = (abs(ds[-1])-min(abs(x) for x in ds) <= c.max_relative_distance_growth_m and
           abs(ys[-1])-min(abs(x) for x in ys) <= c.max_relative_lateral_growth_m)
      mature = (len(evidence.samples) >= c.min_pair_observations and
           (timestamp_ns-evidence.samples[0][0])/1e9 + 1e-6 >= c.min_pair_span_s)
      if stable and mature:
        compatible.add(key)
      else:
        reason = 'relative_motion_drift' if not stable else 'insufficient_temporal_evidence'
      self.last_diagnostics.append(dict(a=key[0], b=key[1], compatible=stable and mature, reason=reason))

    owner = {rid: pid for pid, state in self.states.items() for rid in state.member_last_seen}
    clusters = [[r] for r in raw_tracks]
    while True:
      choices = []
      for i, a in enumerate(clusters):
        for j in range(i+1, len(clusters)):
          b = clusters[j]
          if len(a)+len(b) > c.max_members:
            continue
          if not all(tuple(sorted((x.raw_track_id, y.raw_track_id))) in compatible for x in a for y in b):
            continue
          # Prefer existing membership; then the tightest complete-link pair.
          continuity = any(owner.get(x.raw_track_id) is not None and owner.get(x.raw_track_id) == owner.get(y.raw_track_id)
                  for x in a for y in b)
          cost = max(abs(x.d_rel-y.d_rel)/c.distance_diameter_m +
               abs(x.y_rel-y.y_rel)/c.lateral_diameter_m +
               abs(x.v_rel-y.v_rel)/c.velocity_diameter_mps for x in a for y in b)
          choices.append((not continuity, cost, i, j))
      if not choices:
        break
      _, _, i, j = min(choices)
      clusters[i] += clusters.pop(j)

    previous = sorted(self.states)
    assigned = {}
    if previous and clusters:
      # Only membership-supported physical associations are allowed. Raw
      # tracking already handles cross-slot prediction/kinematic matching.
      scores = np.zeros((len(clusters), len(previous)+len(clusters)))
      for i, cluster in enumerate(clusters):
        members = {m.raw_track_id for m in cluster}
        for j, pid in enumerate(previous):
          state = self.states[pid]
          overlap = members.intersection(state.member_last_seen)
          if overlap:
            scores[i, j] = (10*len(overlap) +
                    3*(state.observation.representative_raw_track_id in members) +
                    min(state.observation.age_scans, 1000)*1e-5 + 1/(pid+1))
      ri, ci = bosch_linear_sum_assignment(-scores)
      assigned = {int(i): previous[int(j)] for i, j in zip(ri, ci) if j < len(previous) and scores[i, j] > 0}

    result = []
    current_ids = {r.raw_track_id for r in raw_tracks}
    for i, cluster in enumerate(clusters):
      pid = assigned.get(i)
      old = self.states.get(pid)
      prior = old.observation if old else None
      if pid is None:
        if self.next_id >= 2**31:
          raise OverflowError('physical Int32 ID space exhausted; no reuse')
        pid, self.next_id = self.next_id, self.next_id+1
        self.stats['created'] += 1
      candidates = sorted(cluster, key=lambda m: m.raw_track_id)
      def representative_cost(m):
        if prior:
          dt = (timestamp_ns-prior.timestamp_ns)/1e9
          angle = -(yaw_rate or 0.)*dt
          dx = prior.d_rel + prior.v_rel*dt
          px = dx*math.cos(angle)-prior.y_rel*math.sin(angle)
          py = dx*math.sin(angle)+prior.y_rel*math.cos(angle)
          continuity = abs(m.d_rel-px) + .5*abs(m.y_rel-py) + .5*abs(m.v_rel-prior.v_rel)
          # Observed state wins over OEM changes; these are only ties.
          return (continuity, m.raw_track_id != prior.representative_raw_track_id,
              not self._vision_support(m, vision), m.slot != oem_slot, -m.age_scans, m.raw_track_id)
        # Initially prefer a robust actual member near the group median.
        median = float(np.median([x.d_rel for x in candidates]))
        return (abs(m.d_rel-median), False, not self._vision_support(m, vision), m.slot != oem_slot, -m.age_scans, m.raw_track_id)
      rep = min(candidates, key=representative_cost)
      obj = BoschPhysicalObject(pid, timestamp_ns, tuple(candidates), rep.raw_track_id,
                rep.d_rel, rep.y_rel, rep.v_rel, any(m.slot == oem_slot for m in candidates),
                any(self._vision_support(m, vision) for m in candidates),
                (prior.age_scans+1 if prior else 1),
                'temporal_complete_link' if len(candidates)>1 else 'single_return')
      last_seen = dict(old.member_last_seen) if old else {}
      # A raw member currently assigned elsewhere cannot belong to this ID.
      for rid in list(last_seen):
        if rid in current_ids and rid not in {m.raw_track_id for m in candidates}:
          del last_seen[rid]
      last_seen.update({m.raw_track_id: timestamp_ns for m in candidates})
      self.states[pid] = _BoschPhysicalState(obj, last_seen)
      if prior and prior.representative_raw_track_id != rep.raw_track_id:
        self.stats['representative_changes'] += 1
      old_owners = {owner[m.raw_track_id] for m in candidates if m.raw_track_id in owner}
      self.stats['membership_merge_events'] += max(0, len(old_owners)-1)
      if old is not None and all(m.recovered for m in candidates):
        self.stats['coasting_recoveries'] += 1
      self.stats['multi_member_groups'] += int(len(candidates)>1)
      self.stats['vision_supported_groups'] += int(obj.vision_supported)
      self.stats['stationary_groups'] += int(math.isfinite(v_ego) and abs(obj.v_rel+v_ego) <= c.stationary_speed_mps)
      self.stats['max_group_size'] = max(self.stats['max_group_size'], len(candidates))
      result.append(obj)

    # Retire absorbed physical IDs immediately, retaining only missing members
    # for genuine coasting. Prevent two IDs from owning the same raw member.
    live_owners = {m.raw_track_id: obj.physical_track_id for obj in result for m in obj.members}
    output_ids = {obj.physical_track_id for obj in result}
    for pid in list(self.states):
      state = self.states[pid]
      state.member_last_seen = {rid: ns for rid, ns in state.member_last_seen.items()
                   if rid not in live_owners or live_owners[rid] == pid}
      if not state.member_last_seen and pid not in output_ids:
        del self.states[pid]
        self.stats['absorbed'] += 1
    self.stats['scans'] += 1
    self.stats['output_objects'] += len(result)
    return tuple(sorted(result, key=lambda obj: obj.physical_track_id))


class BoschPhysicalTracker:
  def __init__(self, raw_config: BoschRawTrackingConfig | None = None, group_config: BoschGroupingConfig | None = None):
    self.raw_manager = BoschRawTrackManager(raw_config)
    self.group_manager = BoschObjectGroupManager(group_config)

  def update(self, timestamp_ns, detections, *, yaw_rate=None, v_ego=math.nan, oem_slot=None, vision=()):
    raw = self.raw_manager.update(timestamp_ns, detections, yaw_rate=yaw_rate)
    return self.group_manager.update(timestamp_ns, raw, yaw_rate=yaw_rate, v_ego=v_ego,
                    oem_slot=oem_slot, vision=vision)


def bosch_to_native_radar_data(objects: Sequence[BoschPhysicalObject], timestamp_ns: int, *, v_ego=math.nan,
            data_type=None, complete=True, unsupported=False):
  """Serialize fresh physical hypotheses. Unknown native fields remain unknown.

  trackState is an integer in the existing schema, so its default 0 is used
  as UNKNOWN (not Bosch moving/confirmed state). measured is receipt only.
  No coasting state is ever serialized. No message is published by this API.
  """
  if data_type is None:
    from opendbc.car import structs
    data_type = structs.RadarData
  if any(obj.timestamp_ns != timestamp_ns for obj in objects):
    raise ValueError('stale physical observation cannot be serialized as fresh')
  if len({obj.physical_track_id for obj in objects}) != len(objects):
    raise ValueError('duplicate physical ID')
  radar = data_type.new_message()
  radar.errors.canError = not complete
  radar.errors.wrongConfig = unsupported
  for p, obj in zip(radar.init('points', len(objects)), objects):
    p.trackId = obj.physical_track_id
    p.dRel, p.yRel, p.vRel = obj.d_rel, obj.y_rel, obj.v_rel
    p.aRel = p.yvRel = p.aLead = p.jLead = math.nan
    p.vLead = v_ego+obj.v_rel
    p.radarSource = 'frontRadar'
    p.trackState = 0
    p.measured = True
  return radar


@dataclass(frozen=True)
class _BoschCanFrame:
  timestamp_ns: int
  address: int
  payload: bytes
  order: int


def bosch_decode_frame(timestamp_ns: int, address: int, payload: bytes):
  """Decode two independent 32-bit records; 0x601 is never a detection."""
  if address not in BOSCH_TRACK_ADDRESSES or len(payload) != 8:
    raise ValueError('expected an eight-byte Bosch raw track frame')
  detections = []
  unsupported = False
  for half in range(2):
    word = int.from_bytes(payload[half * 4:half * 4 + 4], 'little')
    if word == BOSCH_INACTIVE_WORD:
      continue
    if word & (1 << 31):
      unsupported = True
      continue
    detections.append(BoschRawDetection(timestamp_ns, (address - 0x602) * 2 + half,
                                   (word & 0x3FF) * .25,
                                   ((word >> 10) & 0x7FF) * .03125 - 32,
                                   ((word >> 21) & 0x3FF) * .25 - 128, word))
  return tuple(detections), unsupported


def bosch_make_points(objects, v_ego=math.nan):
  """Native physical IDs and original observations; unknown fields stay NaN."""
  if not objects:
    return []
  data = bosch_to_native_radar_data(objects, objects[0].timestamp_ns, v_ego=v_ego)
  return list(data.points)


class BoschRadarProvider:
  def __init__(self, bus: int):
    self.bus = bus
    self.tracker = BoschPhysicalTracker()
    self.can_error = False
    self.wrong_config = False
    self.last_scan_timestamp_ns = None
    self.debug_snapshot = {}
    self.slot_to_ids = {}
    self._frames = []
    self._anchors = []
    self._order = 0
    self._start_ns = None
    self._last_now_ns = None
    self._last_closed_anchor_ns = None
    self._last_output_ns = None
    self._pending_error = False

  def update(self, can_packets, now_ns: int, v_ego: float, yaw_rate_left=None, vision=()):
    """Consume (receive_ns, [(address, payload, src), ...]) CAN packets.

    Return None when no window has closed, otherwise only freshly observed
    physical objects. Empty/error updates at 10 Hz after 0.3 s without a scan
    clear downstream points even when CAN reception has stopped completely.
    All closed scans in a batch update association; only the latest is emitted.
    """
    if self._last_now_ns is not None and now_ns < self._last_now_ns:
      raise ValueError('provider receive clock must not regress')
    self._last_now_ns = now_ns
    if self._start_ns is None:
      self._start_ns = now_ns
    for timestamp_ns, messages in can_packets:
      for address, payload, source in messages:
        if source != self.bus or not 0x601 <= address <= 0x612:
          continue
        if timestamp_ns > now_ns:
          self._pending_error = True
          continue
        payload = bytes(payload)
        if address == 0x612:
          if len(payload) != 8:
            self._pending_error = True
            continue
          tick = int.from_bytes(payload[1:4], 'little')
          if tick % 10 == 0:
            if self._last_closed_anchor_ns is not None and timestamp_ns <= self._last_closed_anchor_ns:
              continue
            if not any(ns == timestamp_ns for ns, _ in self._anchors):
              self._anchors.append((timestamp_ns, tick))
        else:
          self._frames.append(_BoschCanFrame(timestamp_ns, address, payload, self._order))
          self._order += 1
    self._anchors.sort()
    output = None
    while self._anchors and self._anchors[0][0] + BOSCH_WINDOW_NS <= now_ns:
      phase_ns, tick = self._anchors[0]
      assigned, retained = [], []
      for frame in self._frames:
        closest = min(self._anchors, key=lambda anchor: abs(anchor[0] - frame.timestamp_ns))[0]
        if closest == phase_ns and abs(phase_ns - frame.timestamp_ns) <= BOSCH_WINDOW_NS:
          assigned.append(frame)
        else:
          retained.append(frame)
      self._frames = retained
      self._anchors.pop(0)
      self._last_closed_anchor_ns = phase_ns
      output = self._finish_scan(phase_ns, tick, assigned, v_ego, yaw_rate_left, vision)
      self._last_output_ns = now_ns

    # Preserve the pre-anchor half-window. Pending future anchors may already
    # own older frames when update() receives a large batch.
    oldest = min([now_ns] + [ns for ns, _ in self._anchors]) - BOSCH_WINDOW_NS
    self._frames = [frame for frame in self._frames if frame.timestamp_ns >= oldest]
    since = self.last_scan_timestamp_ns if self.last_scan_timestamp_ns is not None else self._start_ns
    if now_ns - since >= BOSCH_STALE_NS and (output is not None or self._last_output_ns is None or now_ns - self._last_output_ns >= BOSCH_OUTPUT_INTERVAL_NS):
      self.can_error = True
      self.slot_to_ids = {}
      self.debug_snapshot = {'bus': self.bus, 'timeout': True, 'last_scan_timestamp_ns': self.last_scan_timestamp_ns,
                             'objects': [], 'slot_to_ids': {}}
      self._last_output_ns = now_ns
      return ()
    return output

  def _finish_scan(self, phase_ns, tick, frames, v_ego, yaw_rate_left, vision):
    bucket = {}
    malformed = self._pending_error or any(len(frame.payload) != 8 for frame in frames)
    self._pending_error = False
    for frame in sorted(frames, key=lambda frame: (frame.timestamp_ns, frame.order)):
      if len(frame.payload) != 8:
        continue
      previous = bucket.get(frame.address)
      if previous is None or abs(frame.timestamp_ns - phase_ns) < abs(previous.timestamp_ns - phase_ns):
        bucket[frame.address] = frame
    complete = BOSCH_TRACK_ADDRESSES.issubset(bucket)
    availability_ns = max([phase_ns] + [frame.timestamp_ns for address, frame in bucket.items() if address in BOSCH_TRACK_ADDRESSES])
    if not complete:
      availability_ns = max(availability_ns, phase_ns + BOSCH_WINDOW_NS)
    detections = []
    unsupported = False
    for frame in sorted(bucket.values(), key=lambda frame: (frame.timestamp_ns, frame.order)):
      if frame.address not in BOSCH_TRACK_ADDRESSES:
        continue
      decoded, unknown = bosch_decode_frame(frame.timestamp_ns, frame.address, frame.payload)
      detections.extend(decoded)
      unsupported |= unknown
    # A delayed or overlapping scan must not turn an old sample into a fresh
    # measurement. The normal 100 ms phase cadence has disjoint 40 ms windows.
    previous_ns = self.tracker.raw_manager.last_timestamp_ns
    if previous_ns is not None:
      if availability_ns <= previous_ns:
        self.can_error = True
        self.slot_to_ids = {}
        return ()
      fresh = [detection for detection in detections if detection.timestamp_ns > previous_ns]
      malformed |= len(fresh) != len(detections)
      detections = fresh
    oem = bucket.get(0x601)
    oem_word = int.from_bytes(oem.payload[4:8], 'little') if oem else None
    matches = [detection.slot for detection in detections if oem_word == detection.raw_word]
    oem_slot = matches[0] if matches else None
    objects = self.tracker.update(availability_ns, detections, yaw_rate=yaw_rate_left,
                                  v_ego=v_ego, oem_slot=oem_slot, vision=vision)
    self.last_scan_timestamp_ns = availability_ns
    self.can_error = not complete or malformed or unsupported
    self.wrong_config = unsupported
    self.slot_to_ids = {member.slot: (member.raw_track_id, obj.physical_track_id) for obj in objects for member in obj.members}
    self.debug_snapshot = {
      'bus': self.bus, 'phase_ns': phase_ns, 'scan_timestamp_ns': availability_ns,
      'closed_ns': self._last_now_ns, 'tick': tick, 'complete': complete,
      'can_error': self.can_error, 'wrong_config': self.wrong_config,
      'oem_word': oem_word, 'oem_selected_slot': oem_slot, 'oem_match_count': len(matches),
      'slot_to_ids': self.slot_to_ids,
      'objects': [{'physicalTrackId': obj.physical_track_id,
                   'rawTrackIds': [member.raw_track_id for member in obj.members],
                   'slots': list(obj.member_slots),
                   'measurement_ns': [member.timestamp_ns for member in obj.members],
                   'representative_rawTrackId': obj.representative_raw_track_id,
                   'representative_slot': next(member.slot for member in obj.members if member.raw_track_id == obj.representative_raw_track_id),
                   'oem_selected': obj.oem_selected} for obj in objects],
    }
    return objects

# End Bosch MRRevo14F passive radar


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    
    self.canfd = True if CP.flags & HyundaiFlags.CANFD else False
    self.radar_group1 = False
    self.radar_group3 = False
    self.radar_group4 = not self.canfd and bool(CP.extFlags & HyundaiExtFlags.RADAR_GROUP4.value)
    if self.canfd:
      if CP.extFlags & HyundaiExtFlags.RADAR_GROUP1.value:
        self.radar_start_addr = RADAR_START_ADDR_CANFD1
        self.radar_msg_count = RADAR_MSG_COUNT1
        self.radar_group1 = True
      elif CP.extFlags & HyundaiExtFlags.RADAR_GROUP3.value:
        self.radar_start_addr = RADAR_START_ADDR_CANFD3
        self.radar_msg_count = RADAR_MSG_COUNT3
        self.radar_group3 = True
      else:
        self.radar_start_addr = RADAR_START_ADDR_CANFD2
        self.radar_msg_count = RADAR_MSG_COUNT2
    else:
      self.radar_start_addr = RADAR_START_ADDR
      self.radar_msg_count = RADAR_MSG_COUNT4 if self.radar_group4 else RADAR_MSG_COUNT
    self.radar_required_msg_count = self.radar_msg_count
    if not self.canfd and not self.radar_group4:
      self.radar_required_msg_count = RADAR_REQUIRED_MSG_COUNT

    self.params = Params()
    self.radar_track_mode = self.params.get_int("EnableRadarTracks")
    self.radar_tracks = self.radar_track_mode >= 1
    self.bosch = None
    self._bosch_objects = ()
    self._bosch_now_ns = 0
    self._bosch_debug_ns = 0
    self._bosch_context = None
    if self.radar_tracks and CP.extFlags & HyundaiExtFlags.BOSCH_RADAR:
      CAN = CanBus(CP)
      bus = CAN.ACAN if CP.extFlags & HyundaiExtFlags.BOSCH_RADAR_BUS1 else CAN.CAM
      self.bosch = BoschRadarProvider(bus)
      self._bosch_make_points = bosch_make_points
    self.corner_object_tracks = bool(CP.extFlags & HyundaiExtFlags.CORNER_RADAR_OBJECTS_235.value) and self.params.get_int("EnableCornerRadar") > 0
    self.corner_object_180_tracks = bool(CP.extFlags & HyundaiExtFlags.CORNER_RADAR_OBJECTS_180.value) and self.params.get_int("EnableCornerRadar") > 0
    # The 0x430/0x440 DBC exposes unvalidated range-bin candidates rather than
    # confirmed objects. Promoting them can create false side tracks and unsafe
    # lead selection, so retain the decoder for offline analysis only.
    self.corner_object_430_tracks = False
    self.updated_tracks = set()
    self.updated_scc = set()
    self.updated_corner_objects = set()
    self.updated_corner_objects_180 = set()
    self.updated_corner_objects_430 = set()
    self.corner_object_missed_updates = 0
    self.corner_object_180_missed_updates = 0
    self.corner_object_430_missed_updates = 0
    self.corner_object_track_ids = CornerObjectTrackIdManager()
    self.rcp_tracks = get_radar_can_parser(
      CP, self.radar_tracks and self.bosch is None, self.radar_start_addr, self.radar_msg_count,
      self.radar_required_msg_count, self.radar_group4,
    )
    self.rcp_corner_objects = get_corner_object_can_parser(CP, self.corner_object_tracks)
    self.rcp_corner_objects_180 = get_corner_object_180_can_parser(CP, self.corner_object_180_tracks)
    self.rcp_corner_objects_430 = get_corner_object_430_can_parser(CP, self.corner_object_430_tracks)
    # Enabling raw radar tracks on legacy CAN disables the stock SCC11 stream on
    # some Hyundai/Kia platforms. Camera-SCC cars may still use SCC11.
    use_scc_parser = self.bosch is not None or not (self.radar_tracks and not self.canfd and not (CP.flags & HyundaiFlags.CAMERA_SCC))
    self.rcp_scc = get_radar_can_parser_scc(CP) if use_scc_parser else None
    self.trigger_msg_scc = 416 if self.canfd else 0x420

    self.trigger_msg_tracks = self.radar_start_addr + self.radar_required_msg_count - 1
    self.trigger_msg_corner_objects = CORNER_OBJECT_235_START_ADDR + CORNER_OBJECT_235_MSG_COUNT - 1
    self.trigger_msg_corner_objects_180 = CORNER_OBJECT_180_START_ADDR + CORNER_OBJECT_180_MSG_COUNT - 1
    self.trigger_msg_corner_objects_430 = CORNER_OBJECT_430_RIGHT_START_ADDR + CORNER_OBJECT_430_MSG_COUNT_PER_SIDE - 1
    self.track_id = 0

    self.corner_objects_available = self.rcp_corner_objects is not None or self.rcp_corner_objects_180 is not None or self.rcp_corner_objects_430 is not None
    self.radar_off_can = CP.radarUnavailable and not self.corner_objects_available and self.bosch is None
    print(
      "RadarInterface: "
      f"radarUnavailable={CP.radarUnavailable} radarTracks={self.radar_tracks} "
      f"group4={self.radar_group4} "
      f"corner235={self.rcp_corner_objects is not None} corner180={self.rcp_corner_objects_180 is not None} "
      f"corner430={self.rcp_corner_objects_430 is not None} "
      f"radarOffCan={self.radar_off_can}"
    )

    self.vRel_last = 0
    self.dRel_last = 0
    self.corner_object_430_prev_d_rel = {}
    self.corner_object_430_prev_v_rel = {}
    self.corner_object_430_prev_y_rel = {}
    self.corner_object_430_prev_yv_rel = {}
    self.corner_object_430_prev_code = {}
    self.corner_object_430_history = {}
    self.corner_object_430_noncenter_inward_frames = {}

    # Initialize pts
    if self.rcp_tracks is not None:
      total_tracks = self.radar_msg_count * (2 if self.radar_group1 else 1)
      for track_id in range(total_tracks):
        t_id = track_id + 32
        self.pts[t_id] = structs.RadarData.RadarPoint()
        self.pts[t_id].measured = False
        self.pts[t_id].trackId = t_id

    if self.rcp_scc is not None:
      self.pts[SCC_TID] = structs.RadarData.RadarPoint()
      self.pts[SCC_TID].trackId = SCC_TID
      self.pts[SCC_TID].radarSource = "scc"
    if self.rcp_corner_objects is not None:
      for slot in range(CORNER_OBJECT_235_MSG_COUNT):
        t_id = CORNER_OBJECT_235_TRACK_ID_OFFSET + slot
        self.pts[t_id] = structs.RadarData.RadarPoint()
        self.pts[t_id].measured = False
        self.pts[t_id].trackId = t_id
        self.pts[t_id].radarSource = "corner235"
    if self.rcp_corner_objects_180 is not None:
      for slot in range(CORNER_OBJECT_180_MSG_COUNT * CORNER_OBJECT_180_SLOTS_PER_MSG):
        t_id = CORNER_OBJECT_180_TRACK_ID_OFFSET + slot
        self.pts[t_id] = structs.RadarData.RadarPoint()
        self.pts[t_id].measured = False
        self.pts[t_id].trackId = t_id
        self.pts[t_id].radarSource = "corner180"
    if self.rcp_corner_objects_430 is not None:
      for slot in range(CORNER_OBJECT_430_MSG_COUNT_PER_SIDE * 2 * CORNER_OBJECT_430_SLOTS_PER_MSG):
        t_id = CORNER_OBJECT_430_TRACK_ID_OFFSET + slot
        self.pts[t_id] = structs.RadarData.RadarPoint()
        self.pts[t_id].measured = False
        self.pts[t_id].trackId = t_id
        self.pts[t_id].radarSource = "corner430"

    self.frame = 0

  def set_bosch_context(self, now_ns, pose=None, pose_ns=0, model=None, model_ns=0):
    """Optional receive context; Device yaw is right-positive, radar y is left."""
    if self.bosch is None:
      return
    yaw = None
    angular = getattr(pose, 'angularVelocityDevice', None)
    if (pose is not None and 0 <= now_ns - pose_ns <= 200_000_000 and
        pose.inputsOK and pose.sensorsOK and angular is not None and angular.valid and math.isfinite(angular.z)):
      yaw = -float(angular.z)
    cues = ()
    if model is not None and 0 <= now_ns - model_ns <= 200_000_000 and model.leadsV3:
      lead = model.leadsV3[0]
      if lead.x and lead.y:
        # Same model-to-radar coordinates as the existing primary matcher.
        cues = (BoschVisionCue(float(lead.x[0]) - 1.52, -float(lead.y[0]), float(lead.prob)),)
    self._bosch_context = (int(now_ns), yaw, cues)

  def update_carrot(self, v_ego, a_ego, rcv_time, can_packets):
    # Keep the legacy SCC/corner MyTrack processing intact. Bosch objects are
    # deliberately outside self.pts: MyTrack synthesizes lateral velocity and
    # acceleration, which have not been decoded for this sensor.
    ret = super().update_carrot(v_ego, a_ego, rcv_time, can_packets)
    if ret is not None and self.bosch is not None:
      scan_ns = self.bosch.last_scan_timestamp_ns
      points = []
      if scan_ns is not None and 0 <= self._bosch_now_ns - scan_ns <= BOSCH_SAMPLE_HOLD_NS:
        points = self._bosch_make_points(self._bosch_objects, self.v_ego)
        # liveTracks timestamps this publication. Align the latest 10 Hz
        # observation to the preserved 20 Hz SCC cadence, just as DPath aligns
        # a received scan to a model frame. Do not invent lateral dynamics.
        for point, obj in zip(points, self._bosch_objects):
          representative = next(member for member in obj.members if member.raw_track_id == obj.representative_raw_track_id)
          age_s = (self._bosch_now_ns - representative.timestamp_ns) * 1e-9
          point.dRel += point.vRel * age_s
      # Snapshot before replacing the Cap'n Proto list: its existing builders
      # otherwise alias the storage being replaced, corrupting SCC points.
      ret.points = [point.to_dict() for point in ret.points] + [point.to_dict() for point in points]
    return ret

  def update(self, can_strings):
    self.frame += 1
    if self.radar_off_can or (self.bosch is None and self.rcp_tracks is None and self.rcp_scc is None and self.rcp_corner_objects is None and self.rcp_corner_objects_180 is None and self.rcp_corner_objects_430 is None):
      return super().update(None)

    if self.rcp_scc is not None:
      vls_s = self.rcp_scc.update(can_strings)
      self.updated_scc.update(vls_s)

    track_ready = False
    if self.bosch is not None:
      now_ns, yaw, cues = self._bosch_context or (time.monotonic_ns(), None, ())
      self._bosch_now_ns = now_ns
      self._bosch_context = None
      objects = self.bosch.update(can_strings, now_ns=now_ns, v_ego=self.v_ego,
                                  yaw_rate_left=yaw, vision=cues)
      if objects is not None:
        self._bosch_objects = objects
        track_ready = True
        if now_ns - self._bosch_debug_ns >= 1_000_000_000:
          carlog.info("BoschRadar %s", self.bosch.debug_snapshot)
          self._bosch_debug_ns = now_ns
    if self.radar_tracks and self.rcp_tracks is not None:
      vls_t = self.rcp_tracks.update(can_strings)
      self.updated_tracks.update(vls_t)
      track_ready = self.trigger_msg_tracks in self.updated_tracks

    corner_ready = False
    if self.rcp_corner_objects is not None:
      vls_c = self.rcp_corner_objects.update(can_strings)
      self.updated_corner_objects.update(vls_c)
      corner_ready = self.trigger_msg_corner_objects in self.updated_corner_objects

    corner_180_ready = False
    if self.rcp_corner_objects_180 is not None:
      vls_180 = self.rcp_corner_objects_180.update(can_strings)
      self.updated_corner_objects_180.update(vls_180)
      corner_180_ready = self.trigger_msg_corner_objects_180 in self.updated_corner_objects_180

    corner_430_ready = False
    if self.rcp_corner_objects_430 is not None:
      vls_430 = self.rcp_corner_objects_430.update(can_strings)
      self.updated_corner_objects_430.update(vls_430)
      corner_430_ready = self.trigger_msg_corner_objects_430 in self.updated_corner_objects_430

    scc_ready = (not self.radar_tracks or self.bosch is not None) and self.frame % 5 == 0 and self.rcp_scc is not None

    if track_ready and self.rcp_tracks is not None:
      self._update(self.updated_tracks)
      self.updated_tracks.clear()

    if corner_ready:
      self._update_corner_objects(self.updated_corner_objects)
      self.corner_object_missed_updates = 0
      self.updated_corner_objects.clear()

    if corner_180_ready:
      self._update_corner_objects_180(self.updated_corner_objects_180)
      self.corner_object_180_missed_updates = 0
      self.updated_corner_objects_180.clear()

    if corner_430_ready:
      self._update_corner_objects_430(self.updated_corner_objects_430)
      self.corner_object_430_missed_updates = 0
      self.updated_corner_objects_430.clear()

    # Corner radar runs at its own cadence. Do not let corner-only frames publish
    # RadarData, since liveTracks uses a fixed radarTimeStep for aLead/jLead.
    bosch_front_only = self.bosch is not None and self.radar_track_mode == 1
    publish_ready = scc_ready if self.bosch is not None else track_ready or scc_ready
    if bosch_front_only:
      # Mode 1 consumes frontRadar; optional SCC reception cannot gate its clock.
      publish_ready = self.frame % 5 == 0
    if not publish_ready:
      return None

    if self.rcp_scc is not None:
      self._update_scc(self.updated_scc)
    if self.rcp_corner_objects is not None:
      if self.updated_corner_objects:
        self._update_corner_objects(self.updated_corner_objects)
        self.corner_object_missed_updates = 0
      else:
        self.corner_object_missed_updates += 1
        if self.corner_object_missed_updates > 10:
          self._clear_corner_objects()
    if self.rcp_corner_objects_180 is not None:
      if self.updated_corner_objects_180:
        self._update_corner_objects_180(self.updated_corner_objects_180)
        self.corner_object_180_missed_updates = 0
      else:
        self.corner_object_180_missed_updates += 1
        if self.corner_object_180_missed_updates > 10:
          self._clear_corner_objects_180()
    if self.rcp_corner_objects_430 is not None:
      if self.updated_corner_objects_430:
        self._update_corner_objects_430(self.updated_corner_objects_430)
        self.corner_object_430_missed_updates = 0
      else:
        self.corner_object_430_missed_updates += 1
        if self.corner_object_430_missed_updates > 10:
          self._clear_corner_objects_430()
    self.updated_scc.clear()
    self.updated_corner_objects.clear()
    self.updated_corner_objects_180.clear()
    self.updated_corner_objects_430.clear()

    ret = structs.RadarData()
    if self.bosch is not None:
      ret.errors.canError = self.bosch.can_error
      ret.errors.wrongConfig = self.bosch.wrong_config
    if ((self.rcp_tracks is not None and self.radar_tracks and not self.rcp_tracks.can_valid) or
        (self.rcp_scc is not None and not bosch_front_only and not self.corner_objects_available and not self.rcp_scc.can_valid) or
        (self.rcp_corner_objects is not None and not self.rcp_corner_objects.can_valid) or
        (self.rcp_corner_objects_180 is not None and not self.rcp_corner_objects_180.can_valid) or
        (self.rcp_corner_objects_430 is not None and not self.rcp_corner_objects_430.can_valid)):
      ret.errors.canError = True
    ret.points = [point for point in self.pts.values() if point.measured]
    return ret

  def _update(self, updated_messages):

    t_id = 32
    for addr in range(self.radar_start_addr, self.radar_start_addr + self.radar_msg_count):

      msg = self.rcp_tracks.vl[f"RADAR_TRACK_{addr:x}"]
      track_state = 0
      optional_track_stale = (addr >= self.radar_start_addr + self.radar_required_msg_count and
                              addr not in updated_messages)

      if self.radar_group1:
        valid = msg['VALID_CNT1'] > 10
      elif self.radar_group3:
        # Group 3 marks an empty object slot with LONG_DIST raw 0x7ff (204.7 m).
        valid = msg['LONG_DIST'] < 204.7
      elif self.canfd:
        valid, track_state = canfd_group2_track_status(msg)
      elif self.radar_group4:
        # EN: DNMWR006 exposes eight stable tracked-object slots at 0x500-0x507.
        #     Messages from 0x508 onward are distance-sorted raw detections without
        #     stable IDs, so they are excluded. OBJECT_STATE 3 is a confirmed track;
        #     empty slots use LONG_DIST raw 0xfff8 (409.55 m). Driving logs reached
        #     317.80 m, so 325 m preserves every observed confirmed track while
        #     retaining margin from the empty-slot sentinel. Keep the +/-6 m
        #     ego/adjacent-lane envelope to suppress farther roadside reflections.
        # KO: DNMWR006의 안정적인 추적 객체 슬롯은 0x500~0x507의 8개임.
        #     0x508 이후 메시지는 고정 ID가 없는 거리순 raw detection이므로 제외함.
        #     OBJECT_STATE 3은 확정 추적 객체이며, 빈 슬롯은 LONG_DIST raw
        #     0xfff8(409.55m)을 사용함. 주행 로그의 최대값은 317.80m였으므로
        #     325m 상한으로 관측된 확정 트랙을 모두 보존하면서 빈 슬롯 값과 충분한
        #     여유를 확보함. 원거리 도로변 반사를 줄이기 위해 좌우 6m 범위를 유지함.
        valid = (msg['OBJECT_STATE'] == 3 and 0.2 < msg['LONG_DIST'] < RADAR_GROUP4_MAX_LONG_DIST and
                 abs(msg['LAT_DIST']) <= RADAR_GROUP4_MAX_YREL)
      else:
        valid = msg['STATE'] in (3, 4)

      # Optional slots do not participate in CAN validity. Therefore explicitly
      # require a fresh frame in each radar cycle instead of retaining a valid
      # object from the last cycle if the upper bank stops transmitting.
      valid = valid and not optional_track_stale

      self.pts[t_id].measured = bool(valid)
      if not valid:
        self.pts[t_id].dRel = 0
        self.pts[t_id].yRel = 0
        self.pts[t_id].vRel = 0
        self.pts[t_id].vLead = self.pts[t_id].vRel + self.v_ego
        self.pts[t_id].aRel = float('nan')
        self.pts[t_id].yvRel = 0
      elif self.radar_group1:
        self.pts[t_id].dRel = msg['LONG_DIST1']
        self.pts[t_id].yRel = msg['LAT_DIST1']
        self.pts[t_id].vRel = msg['REL_SPEED1']
        self.pts[t_id].vLead = self.pts[t_id].vRel + self.v_ego
        self.pts[t_id].aRel = msg['REL_ACCEL1']
        self.pts[t_id].yvRel = msg['LAT_SPEED1']
      elif self.canfd:
        if self.radar_group3:
          # Group 3 reports the object's center. Convert it to the rear surface to match SCC/vision dRel.
          self.pts[t_id].dRel = max(0.0, msg['LONG_DIST'] - msg['OBJECT_LENGTH'] * 0.5 - 0.1)
        else:
          self.pts[t_id].dRel = msg['LONG_DIST']
        self.pts[t_id].yRel = msg['LAT_DIST']
        self.pts[t_id].vRel = msg['REL_SPEED']
        self.pts[t_id].vLead = self.pts[t_id].vRel + self.v_ego
        self.pts[t_id].aRel = float('nan') if self.radar_group3 else msg['REL_ACCEL']
        self.pts[t_id].yvRel = 0.0 if self.radar_group3 else msg['LAT_SPEED']
        self.pts[t_id].trackState = track_state
      elif self.radar_group4:
        self.pts[t_id].dRel = msg['LONG_DIST']
        self.pts[t_id].yRel = -msg['LAT_DIST']
        self.pts[t_id].vRel = msg['REL_SPEED']
        self.pts[t_id].vLead = self.pts[t_id].vRel + self.v_ego
        self.pts[t_id].aRel = float('nan')
        self.pts[t_id].yvRel = 0.0
      else:
        azimuth = math.radians(msg['AZIMUTH'])
        self.pts[t_id].dRel = math.cos(azimuth) * msg['LONG_DIST']
        self.pts[t_id].yRel = 0.5 * -math.sin(azimuth) * msg['LONG_DIST']
        self.pts[t_id].vRel = msg['REL_SPEED']
        self.pts[t_id].vLead = self.pts[t_id].vRel + self.v_ego
        self.pts[t_id].aRel = msg['REL_ACCEL']
        self.pts[t_id].yvRel = 0.0

      t_id += 1
    # Radar group 1 carries two messages per object.
    if self.radar_group1:
      for addr in range(self.radar_start_addr, self.radar_start_addr + self.radar_msg_count):
        msg = self.rcp_tracks.vl[f"RADAR_TRACK_{addr:x}"]

        optional_track_stale = (addr >= self.radar_start_addr + self.radar_required_msg_count and
                                addr not in updated_messages)
        valid = msg['VALID_CNT2'] > 10 and not optional_track_stale
        self.pts[t_id].measured = bool(valid)
        if not valid:
          self.pts[t_id].dRel = 0
          self.pts[t_id].yRel = 0
          self.pts[t_id].vRel = 0
          self.pts[t_id].vLead = self.pts[t_id].vRel + self.v_ego
          self.pts[t_id].aRel = float('nan')
          self.pts[t_id].yvRel = 0
        else:
          self.pts[t_id].dRel = msg['LONG_DIST2']
          self.pts[t_id].yRel = msg['LAT_DIST2']
          self.pts[t_id].vRel = msg['REL_SPEED2']
          self.pts[t_id].vLead = self.pts[t_id].vRel + self.v_ego
          self.pts[t_id].aRel = msg['REL_ACCEL2']
          self.pts[t_id].yvRel = msg['LAT_SPEED2']

        t_id += 1

  def _update_corner_objects(self, updated_messages):
    if self.rcp_corner_objects is None:
      return

    if not updated_messages:
      self._clear_corner_objects()
      return

    candidates = []
    for slot, addr in enumerate(range(CORNER_OBJECT_235_START_ADDR, CORNER_OBJECT_235_START_ADDR + CORNER_OBJECT_235_MSG_COUNT)):
      t_id = CORNER_OBJECT_235_TRACK_ID_OFFSET + slot
      msg = self.rcp_corner_objects.vl[f"CORNER_RADAR_235_OBJECTS_{addr:x}"]

      d_rel = msg["OBJ_REL_POS_X"]
      y_rel = msg["OBJ_REL_POS_Y"]
      v_rel = msg["OBJ_REL_VEL_X"]
      yv_rel = msg["OBJ_REL_VEL_Y"]
      a_rel = msg["OBJ_REL_ACCEL_X"]
      # Side objects are clipped to x=0 by the corner radar. Quality, identity,
      # and lateral motion still describe a real object, so keep them for
      # corner-confirmed front-radar association in radard.
      valid = msg["OBJ_QUAL_LEVEL"] > 0 and corner_object_position_valid(d_rel, y_rel) and v_rel > -99.0

      if not valid:
        continue
      candidates.append((t_id, int(msg["OBJ_OBJECT_ID"]), int(msg["OBJ_AGE"]), int(msg["OBJ_QUAL_LEVEL"]),
                         d_rel, y_rel, v_rel, yv_rel, a_rel))

    self._apply_corner_objects("corner235", candidates,
                               range(CORNER_OBJECT_235_TRACK_ID_OFFSET,
                                     CORNER_OBJECT_235_TRACK_ID_OFFSET + CORNER_OBJECT_235_MSG_COUNT))

  def _update_corner_objects_180(self, updated_messages):
    if self.rcp_corner_objects_180 is None:
      return

    if not updated_messages:
      self._clear_corner_objects_180()
      return

    candidates = []
    for msg_index, addr in enumerate(range(CORNER_OBJECT_180_START_ADDR, CORNER_OBJECT_180_START_ADDR + CORNER_OBJECT_180_MSG_COUNT)):
      msg = self.rcp_corner_objects_180.vl[f"CORNER_RADAR_180_OBJECTS_{addr:x}"]
      for slot_index in range(CORNER_OBJECT_180_SLOTS_PER_MSG):
        t_id = CORNER_OBJECT_180_TRACK_ID_OFFSET + msg_index * CORNER_OBJECT_180_SLOTS_PER_MSG + slot_index
        prefix = f"SLOT{slot_index + 1}_"
        d_rel = msg[f"{prefix}REL_POS_X"]
        y_rel = msg[f"{prefix}REL_POS_Y"]
        v_rel = msg[f"{prefix}REL_VEL_X"]
        yv_rel = msg[f"{prefix}REL_VEL_Y"]
        a_rel = msg[f"{prefix}REL_ACCEL_X"]
        valid = msg[f"{prefix}QUAL_LEVEL"] > 0 and corner_object_position_valid(d_rel, y_rel) and v_rel > -99.0

        if not valid:
          continue
        candidates.append((t_id, int(msg[f"{prefix}OBJECT_ID"]), int(msg[f"{prefix}AGE"]), int(msg[f"{prefix}QUAL_LEVEL"]),
                           d_rel, y_rel, v_rel, yv_rel, a_rel))

    self._apply_corner_objects("corner180", candidates,
                               range(CORNER_OBJECT_180_TRACK_ID_OFFSET,
                                     CORNER_OBJECT_180_TRACK_ID_OFFSET + CORNER_OBJECT_180_MSG_COUNT * CORNER_OBJECT_180_SLOTS_PER_MSG))

  def _apply_corner_objects(self, source, candidates, slot_ids):
    for t_id in slot_ids:
      self._clear_point(t_id)

    # The same object can occupy two CAN slots for one cycle during a slot handoff.
    # Only merge physically close copies. The radar can assign one object ID to
    # two distant objects concurrently, so object ID alone is not an identity.
    objects = deduplicate_corner_candidates(candidates)
    track_ids = self.corner_object_track_ids.get_track_ids(source, objects)

    for t_id, _, _, _, d_rel, y_rel, v_rel, yv_rel, a_rel in objects:
      point = self.pts[t_id]
      point.measured = True
      point.trackId = track_ids[t_id]
      point.radarSource = source
      point.dRel = d_rel
      point.yRel = y_rel
      point.vRel = v_rel
      point.vLead = v_rel + self.v_ego
      point.aRel = a_rel
      point.yvRel = yv_rel

  def _update_corner_objects_430(self, updated_messages):
    if self.rcp_corner_objects_430 is None:
      return

    if not updated_messages:
      self._clear_corner_objects_430()
      return

    bank_defs = (
      (CORNER_OBJECT_430_LEFT_START_ADDR, 1.0, 0),
      (CORNER_OBJECT_430_RIGHT_START_ADDR, -1.0, CORNER_OBJECT_430_MSG_COUNT_PER_SIDE * CORNER_OBJECT_430_SLOTS_PER_MSG),
    )
    for start_addr, side_sign, track_base in bank_defs:
      bins = []
      for msg_index, addr in enumerate(range(start_addr, start_addr + CORNER_OBJECT_430_MSG_COUNT_PER_SIDE)):
        msg = self.rcp_corner_objects_430.vl[f"CORNER_RADAR_430_OBJECTS_{addr:x}"]
        for slot_index in range(CORNER_OBJECT_430_SLOTS_PER_MSG):
          prefix = f"SLOT{slot_index + 1}_"
          distance_raw = int(msg[f"{prefix}DISTANCE_RAW"])
          raw = (
            distance_raw |
            (int(msg[f"{prefix}META_13_15"]) << 13) |
            (int(msg[f"{prefix}META_BYTE_2"]) << 16) |
            (int(msg[f"{prefix}META_BYTE_3"]) << 24)
          )
          code = (
            int(msg[f"{prefix}META_13_15"]),
            int(msg[f"{prefix}META_BYTE_2"]),
            int(msg[f"{prefix}META_BYTE_3"]),
          )
          d_rel = distance_raw * 0.05
          default_distance = CORNER_OBJECT_430_DEFAULT_DISTANCE_RAW_MIN <= distance_raw <= CORNER_OBJECT_430_DEFAULT_DISTANCE_RAW_MAX
          base_valid = (
            raw not in CORNER_OBJECT_430_EMPTY_RAW_VALUES and
            distance_raw not in (0, 8000, 8191) and
            not default_distance and
            0.2 < d_rel < CORNER_OBJECT_430_MAX_DREL
          )
          candidate_valid = (
            base_valid and
            slot_index + 1 not in CORNER_OBJECT_430_CANDIDATE_EXCLUDED_SLOTS and
            code[2] in CORNER_OBJECT_430_CANDIDATE_META_BYTE_3 and
            code[1] in CORNER_OBJECT_430_STRONG_META_BYTE_2 + CORNER_OBJECT_430_WEAK_META_BYTE_2
          )
          bins.append({
            "msg_index": msg_index,
            "slot_index": slot_index,
            "distance_raw": distance_raw,
            "d_rel": d_rel,
            "code": code,
            "candidate_valid": candidate_valid,
          })

      supported_bins = []
      candidates = [b for b in bins if b["candidate_valid"]]
      for b in candidates:
        support = 1
        for other in candidates:
          if other is b:
            continue
          if abs(other["msg_index"] - b["msg_index"]) > 1:
            continue
          if abs(other["slot_index"] - b["slot_index"]) > 2:
            continue
          if abs(other["distance_raw"] - b["distance_raw"]) > CORNER_OBJECT_430_CANDIDATE_RAW_DELTA:
            continue
          support += 1
        min_support = (CORNER_OBJECT_430_STRONG_MIN_SUPPORT if b["code"][1] in CORNER_OBJECT_430_STRONG_META_BYTE_2
                       else CORNER_OBJECT_430_WEAK_MIN_SUPPORT)
        if support >= min_support:
          supported_bins.append({**b, "support": support})

      clusters = []
      for b in sorted(supported_bins, key=lambda item: item["distance_raw"]):
        if not clusters or b["distance_raw"] - clusters[-1][-1]["distance_raw"] > CORNER_OBJECT_430_CLUSTER_RAW_GAP:
          clusters.append([b])
        else:
          clusters[-1].append(b)
      clusters = sorted(clusters, key=lambda cluster: sum(b["distance_raw"] for b in cluster) / len(cluster))[:CORNER_OBJECT_430_MAX_TRACKS_PER_SIDE]

      cluster_objects = []
      for cluster in clusters:
        msg_index = sum(b["msg_index"] for b in cluster) / len(cluster)
        slot = sum(b["slot_index"] + 1 for b in cluster) / len(cluster)
        lateral_cell = (CORNER_OBJECT_430_LATERAL_CELL_MSG_WEIGHT * msg_index +
                        CORNER_OBJECT_430_LATERAL_CELL_SLOT_WEIGHT * slot)
        mapped_cell = lateral_cell if side_sign > 0.0 else CORNER_OBJECT_430_RIGHT_CELL_MIRROR - lateral_cell
        y_abs = max(CORNER_OBJECT_430_MIN_ABS_YREL,
                    min(CORNER_OBJECT_430_MAX_ABS_YREL,
                        CORNER_OBJECT_430_YREL_OFFSET - CORNER_OBJECT_430_YREL_SCALE * mapped_cell))
        cluster_objects.append({
          "d_rel": sum(b["d_rel"] for b in cluster) / len(cluster),
          "y_rel": side_sign * y_abs,
          "code": max((b["code"] for b in cluster), key=lambda code: sum(1 for item in cluster if item["code"] == code)),
        })

      active_t_ids = set()
      side_track_ids = [
        CORNER_OBJECT_430_TRACK_ID_OFFSET + track_base + slot
        for slot in range(CORNER_OBJECT_430_MAX_TRACKS_PER_SIDE)
      ]
      unmatched_track_ids = {t_id for t_id in side_track_ids if t_id in self.corner_object_430_prev_d_rel}
      unused_track_ids = [t_id for t_id in side_track_ids if t_id not in unmatched_track_ids]

      for cluster in cluster_objects:
        d_rel = cluster["d_rel"]
        code = cluster["code"]
        matched_t_id = None
        if unmatched_track_ids:
          nearest_t_id = min(unmatched_track_ids, key=lambda t_id: abs(d_rel - self.corner_object_430_prev_d_rel[t_id]))
          if abs(d_rel - self.corner_object_430_prev_d_rel[nearest_t_id]) <= CORNER_OBJECT_430_TRACK_MATCH_MAX_DREL_DELTA:
            matched_t_id = nearest_t_id
            unmatched_track_ids.remove(matched_t_id)
        if matched_t_id is None and unused_track_ids:
          matched_t_id = unused_track_ids.pop(0)
        if matched_t_id is None:
          continue

        t_id = matched_t_id
        active_t_ids.add(t_id)
        prev_d_rel = self.corner_object_430_prev_d_rel.get(t_id)
        prev_code = self.corner_object_430_prev_code.get(t_id)
        self.corner_object_430_prev_d_rel[t_id] = d_rel
        self.corner_object_430_prev_y_rel[t_id] = cluster["y_rel"]
        self.corner_object_430_prev_code[t_id] = code
        reset_track = prev_d_rel is None or code != prev_code or abs(d_rel - prev_d_rel) > CORNER_OBJECT_430_MAX_DREL_DELTA
        if reset_track:
          self.corner_object_430_prev_v_rel.pop(t_id, None)
          self.corner_object_430_prev_yv_rel.pop(t_id, None)
          self.corner_object_430_history.pop(t_id, None)
          self.corner_object_430_noncenter_inward_frames.pop(t_id, None)

        history = self.corner_object_430_history.setdefault(t_id, deque(maxlen=CORNER_OBJECT_430_HISTORY_SIZE))
        history.append((d_rel, cluster["y_rel"]))
        if len(history) < CORNER_OBJECT_430_MIN_HISTORY:
          self._clear_point(t_id)
          continue

        window_dt = CORNER_OBJECT_430_DT * (len(history) - 1)
        first_d_rel, first_y_rel = history[0]
        hist_v_rel = (d_rel - first_d_rel) / window_dt
        if abs(hist_v_rel) > CORNER_OBJECT_430_MAX_ABS_VREL:
          self.corner_object_430_prev_v_rel.pop(t_id, None)
          self.corner_object_430_prev_yv_rel.pop(t_id, None)
          self.corner_object_430_history.pop(t_id, None)
          self.corner_object_430_noncenter_inward_frames.pop(t_id, None)
          self._clear_point(t_id)
          continue
        prev_v_rel = self.corner_object_430_prev_v_rel.get(t_id, hist_v_rel)
        v_rel = (1.0 - CORNER_OBJECT_430_VREL_ALPHA) * prev_v_rel + CORNER_OBJECT_430_VREL_ALPHA * hist_v_rel
        self.corner_object_430_prev_v_rel[t_id] = v_rel

        inward_steps = 0
        usable_steps = 0
        prev_abs_y = abs(history[0][1])
        for _, y_rel in list(history)[1:]:
          abs_y = abs(y_rel)
          delta = prev_abs_y - abs_y
          if abs(delta) > 1e-3:
            usable_steps += 1
            if delta > 0.0:
              inward_steps += 1
          prev_abs_y = abs_y
        net_inward_y = abs(first_y_rel) - abs(cluster["y_rel"])
        inward_ratio = inward_steps / usable_steps if usable_steps > 0 else 0.0
        hist_yv_rel = (cluster["y_rel"] - first_y_rel) / window_dt
        recent_inward_y = abs(history[-3][1]) - abs(cluster["y_rel"]) if len(history) >= 3 else net_inward_y
        if (net_inward_y < CORNER_OBJECT_430_MIN_INWARD_YREL_DELTA or
            recent_inward_y < CORNER_OBJECT_430_MIN_RECENT_INWARD_YREL_DELTA or
            inward_ratio < CORNER_OBJECT_430_MIN_INWARD_RATIO or
            abs(hist_yv_rel) > CORNER_OBJECT_430_MAX_ABS_YVREL):
          hist_yv_rel = 0.0
        inward_motion_candidate = hist_yv_rel != 0.0 and abs(cluster["y_rel"]) <= CORNER_OBJECT_430_INWARD_KEEP_YVREL_ABS_YREL
        inward_center_candidate = inward_motion_candidate and abs(cluster["y_rel"]) <= CORNER_OBJECT_430_INWARD_CENTER_ABS_YREL
        y_rel = cluster["y_rel"]
        if inward_motion_candidate:
          if inward_center_candidate:
            self.corner_object_430_noncenter_inward_frames[t_id] = 0
            prev_yv_rel = self.corner_object_430_prev_yv_rel.get(t_id, hist_yv_rel)
            yv_rel = (1.0 - CORNER_OBJECT_430_YVREL_ALPHA) * prev_yv_rel + CORNER_OBJECT_430_YVREL_ALPHA * hist_yv_rel
          else:
            noncenter_frames = self.corner_object_430_noncenter_inward_frames.get(t_id, 0) + 1
            self.corner_object_430_noncenter_inward_frames[t_id] = noncenter_frames
            if noncenter_frames <= CORNER_OBJECT_430_EARLY_INWARD_NONCENTER_FRAMES:
              prev_yv_rel = self.corner_object_430_prev_yv_rel.get(t_id, hist_yv_rel)
              yv_rel = (1.0 - CORNER_OBJECT_430_YVREL_ALPHA) * prev_yv_rel + CORNER_OBJECT_430_YVREL_ALPHA * hist_yv_rel
            else:
              yv_rel = 0.0
          if not inward_center_candidate and abs(y_rel) < CORNER_OBJECT_430_SIDE_KEEP_ABS_YREL:
            y_rel = math.copysign(CORNER_OBJECT_430_SIDE_KEEP_ABS_YREL, y_rel)
        else:
          hist_yv_rel = 0.0
          yv_rel = 0.0
          self.corner_object_430_noncenter_inward_frames[t_id] = 0
          if abs(y_rel) < CORNER_OBJECT_430_SIDE_KEEP_ABS_YREL:
            y_rel = math.copysign(CORNER_OBJECT_430_SIDE_KEEP_ABS_YREL, y_rel)
        self.corner_object_430_prev_yv_rel[t_id] = yv_rel

        self.pts[t_id].measured = True
        self.pts[t_id].trackId = t_id
        self.pts[t_id].dRel = d_rel
        self.pts[t_id].yRel = y_rel
        self.pts[t_id].vRel = v_rel
        self.pts[t_id].vLead = v_rel + self.v_ego
        self.pts[t_id].aRel = float('nan')
        self.pts[t_id].yvRel = yv_rel

      side_track_count = CORNER_OBJECT_430_MSG_COUNT_PER_SIDE * CORNER_OBJECT_430_SLOTS_PER_MSG
      for slot in range(side_track_count):
        t_id = CORNER_OBJECT_430_TRACK_ID_OFFSET + track_base + slot
        if t_id in active_t_ids:
          continue
        self.corner_object_430_prev_d_rel.pop(t_id, None)
        self.corner_object_430_prev_v_rel.pop(t_id, None)
        self.corner_object_430_prev_y_rel.pop(t_id, None)
        self.corner_object_430_prev_yv_rel.pop(t_id, None)
        self.corner_object_430_prev_code.pop(t_id, None)
        self.corner_object_430_history.pop(t_id, None)
        self.corner_object_430_noncenter_inward_frames.pop(t_id, None)
        self._clear_point(t_id)


  def _clear_point(self, t_id):
    self.pts[t_id].measured = False
    self.pts[t_id].dRel = 0
    self.pts[t_id].yRel = 0
    self.pts[t_id].vRel = 0
    self.pts[t_id].vLead = self.v_ego
    self.pts[t_id].aRel = float('nan')
    self.pts[t_id].yvRel = 0

  def _clear_corner_objects(self):
    for slot in range(CORNER_OBJECT_235_MSG_COUNT):
      self._clear_point(CORNER_OBJECT_235_TRACK_ID_OFFSET + slot)
    self.corner_object_track_ids.clear_source("corner235")

  def _clear_corner_objects_180(self):
    for slot in range(CORNER_OBJECT_180_MSG_COUNT * CORNER_OBJECT_180_SLOTS_PER_MSG):
      self._clear_point(CORNER_OBJECT_180_TRACK_ID_OFFSET + slot)
    self.corner_object_track_ids.clear_source("corner180")

  def _clear_corner_objects_430(self):
    self.corner_object_430_prev_d_rel.clear()
    self.corner_object_430_prev_v_rel.clear()
    self.corner_object_430_prev_y_rel.clear()
    self.corner_object_430_prev_yv_rel.clear()
    self.corner_object_430_prev_code.clear()
    self.corner_object_430_history.clear()
    self.corner_object_430_noncenter_inward_frames.clear()
    for slot in range(CORNER_OBJECT_430_MSG_COUNT_PER_SIDE * 2 * CORNER_OBJECT_430_SLOTS_PER_MSG):
      self._clear_point(CORNER_OBJECT_430_TRACK_ID_OFFSET + slot)

  def _update_scc(self, updated_messages):
    cpt = self.rcp_scc.vl
    t_id = SCC_TID
    if self.canfd:
      dRel = cpt["SCC_CONTROL"]['ACC_ObjDist']
      vRel = cpt["SCC_CONTROL"]['ACC_ObjRelSpd']
      new_pts = abs(dRel - self.dRel_last) > 3 or abs(vRel - self.vRel_last) > 1
      vLead = vRel + self.v_ego
      valid = 0 < dRel < 150 and not new_pts #cpt["SCC_CONTROL"]['OBJ_STATUS'] and dRel < 150
      self.pts[t_id].measured = bool(valid)
      if not valid:
        self.pts[t_id].dRel = 0
        self.pts[t_id].yRel = 0
        self.pts[t_id].vRel = 0
        self.pts[t_id].vLead = self.pts[t_id].vRel + self.v_ego
        self.pts[t_id].aRel = float('nan')
        self.pts[t_id].yvRel = 0
      else:
        self.pts[t_id].dRel = dRel
        self.pts[t_id].yRel = 0
        self.pts[t_id].vRel = vRel
        self.pts[t_id].vLead = vLead
        self.pts[t_id].aRel = float('nan')
        self.pts[t_id].yvRel = 0 #float('nan')
    else:
      dRel = cpt["SCC11"]['ACC_ObjDist']
      vRel = cpt["SCC11"]['ACC_ObjRelSpd']
      new_pts = abs(dRel - self.dRel_last) > 3 or abs(vRel - self.vRel_last) > 1
      vLead = vRel + self.v_ego
      valid = cpt["SCC11"]['ACC_ObjStatus'] and dRel < 150 and not new_pts
      self.pts[t_id].measured = bool(valid)
      if not valid:
        self.pts[t_id].dRel = 0
        self.pts[t_id].yRel = 0
        self.pts[t_id].vRel = 0
        self.pts[t_id].vLead = self.pts[t_id].vRel + self.v_ego
        self.pts[t_id].aRel = float('nan')
        self.pts[t_id].yvRel = 0
      else:
        self.pts[t_id].dRel = dRel
        self.pts[t_id].yRel = -cpt["SCC11"]['ACC_ObjLatPos']  # in car frame's y axis, left is negative
        self.pts[t_id].vRel = vRel
        self.pts[t_id].vLead = vLead
        self.pts[t_id].aRel = float('nan')
        self.pts[t_id].yvRel = 0 #float('nan')

    self.dRel_last = dRel
    self.vRel_last = vRel
