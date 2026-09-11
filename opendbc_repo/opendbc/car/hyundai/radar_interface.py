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
# A model path is only a corridor where it still reaches well past the object.
# modelV2.position is 33 samples on the fixed grid T_IDXS[i] = 10*(i/32)**2
# seconds, so its reach in metres collapses with speed: about 10*vEgo. Near the
# end of that reach the samples are a forecast rather than observed road, and
# they stop being usable to drop a return. Measured over 306,896 consecutive
# static observations on five routes, the scan-to-scan disagreement of the
# corridor residual is 0.72 m at p95 with no rule and 0.40 m once this much
# path is required beyond the query point.
BOSCH_PATH_TRUST_MARGIN_M = 40.0
BOSCH_STALE_NS = 300_000_000
BOSCH_OUTPUT_INTERVAL_NS = 100_000_000
BOSCH_SAMPLE_HOLD_NS = 150_000_000  # one 10 Hz observation period plus one SCC publication period

# Candidate P91 changes only the final Bosch publication view; raw detections,
# grouping, physical IDs, qualification and alias bindings remain. SHADOW
# diagnostics stay available for replay and on-device comparison.
BOSCH_P91_OFF = 0
BOSCH_P91_SHADOW = 1
BOSCH_P91_ACTIVE = 2
BOSCH_P91_ACTIVE_TEST = BOSCH_P91_ACTIVE  # compatibility name for existing synthetic tests
BOSCH_P91_MODE = BOSCH_P91_ACTIVE
BOSCH_P91_SUPPORT_HOLD_NS = 500_000_000

# Behavioural naming for the two 0x601 records. No proprietary signal name is
# claimed. word1 (bytes 4..7) is bit-identical to exactly one raw record in
# 63,819 of 63,822 active scans across 146 segments, and its activity equals
# SCC11.ObjValid in 99 % of frames; word0 (bytes 0..3) matches no raw record in
# 96 % of active scans and is gated by lateral offset and by range near 100 m.
# The pair is therefore read as a state, not as one boolean support flag.
BOSCH_OEM_STATE_NONE = 0
BOSCH_OEM_STATE_TENTATIVE = 1   # word0 only: in-path candidate, no selected member
BOSCH_OEM_STATE_SELECTED = 2    # word1 only: selected member outside the word0 gate
BOSCH_OEM_STATE_VALIDATED = 3   # word0 and word1

BOSCH_OEM_GATE_OFF = 0
BOSCH_OEM_GATE_SHADOW = 1
BOSCH_OEM_GATE_ACTIVE = 2
BOSCH_OEM_GATE_MODE = BOSCH_OEM_GATE_ACTIVE
# A validated target that drops word1 for one or two scans must not become
# gate-eligible. Measured bracketed word1 dropouts on 146 segments are 58 in
# total and only 12 % are shorter than five scans, so the hold is cheap.
BOSCH_OEM_EVIDENCE_HOLD_NS = 500_000_000
# SCC12 keeps computing a longitudinal request while openpilot drives. It is an
# independent OEM intent observation, never an actuator command: it only helps
# decide whether an unvalidated in-path candidate may reach the published view.
# Raw field 0 decodes to the -10.23 floor and means "not populated" here, so a
# sane band is required before the sample counts at all.
BOSCH_OEM_INTENT_MIN_MPS2 = -9.0
BOSCH_OEM_INTENT_MAX_MPS2 = 3.0
BOSCH_OEM_INTENT_FRESH_NS = 150_000_000
BOSCH_OEM_INTENT_WINDOW_NS = 300_000_000
BOSCH_OEM_INTENT_MIN_SAMPLES = 3
# Gate thresholds. Every value is set from the route273/26d/274 sweep in
# analysis/bosch_oem_validation; none is a guessed round number.
BOSCH_OEM_GATE_NEUTRAL_MPS2 = -0.25
BOSCH_OEM_GATE_PERSIST_SCANS = 3
BOSCH_OEM_GATE_MIN_SPEED_MPS = 8.0
BOSCH_OEM_GATE_MIN_RANGE_M = 30.0
BOSCH_OEM_GATE_MIN_TTC_S = 4.0
BOSCH_OEM_GATE_CUTIN_MPS = 0.90
BOSCH_OEM_GATE_CUTIN_SCANS = 3
BOSCH_OEM_GATE_LATERAL_WINDOW_NS = 600_000_000
# Only a lead candidate can be withheld, and only while it is new to the path.
# Widening the corridor past 1.0 m or letting a settled object stay eligible
# both reintroduced real-lead losses in the route273/26d/274 sweep.
BOSCH_OEM_GATE_IN_PATH_M = 1.0
BOSCH_OEM_GATE_SETTLED_SCANS = 10
# A large vehicle answers as several returns. When one of them is the OEM's
# validated target the others are companions of a confirmed object, not
# unvalidated candidates; without this the second return of a followed box
# truck was withheld for 0.6 s on route26d segment 87.
BOSCH_OEM_GATE_SIBLING_D_M = 12.0
BOSCH_OEM_GATE_SIBLING_Y_M = 3.0
BOSCH_OEM_GATE_SIBLING_V_MPS = 1.5
# Stock SCC observation, read on the Bosch provider's own CAN stream so the
# samples carry receive timestamps and no shared CANParser changes. Offsets are
# the hyundai_kia_generic SCC11/SCC12 little-endian start bits.
BOSCH_SCC_BUS = 2
BOSCH_SCC11_ADDR = 0x420
BOSCH_SCC12_ADDR = 0x421
BOSCH_SCC11_OBJ_VALID_BIT = 16      # SCC11.ObjValid
BOSCH_SCC12_AREQ_RAW_BIT = 24       # SCC12.aReqRaw, 11 bits, (0.01, -10.23)
BOSCH_SCC_ADDRESSES = frozenset((BOSCH_SCC11_ADDR, BOSCH_SCC12_ADDR))
BOSCH_SCC_STALE_NS = 300_000_000

# Frozen Bosch <-> OEM-camera extended grouping candidate.  Keep production
# disabled until the on-device SHADOW timing pass is complete; tests and replay
# may opt into SHADOW/ACTIVE explicitly when constructing BoschRadarProvider.
BOSCH_CAMERA_EXTENDED_OFF = 0
BOSCH_CAMERA_EXTENDED_SHADOW = 1
BOSCH_CAMERA_EXTENDED_ACTIVE = 2
BOSCH_CAMERA_EXTENDED_ACTIVE_TEST = 3
# EXPERIMENTAL / TEST 전용: 실제 RadarData를 변경한다. UI 전용 overlay가 아니다.
# 감독하 테스트카 로그 수집에만 사용하며 longitudinal control engagement는 승인되지 않았다.
# 원복은 아래 MODE 한 줄을 OFF 또는 SHADOW로 변경하고 card를 재시작한다.
BOSCH_CAMERA_EXTENDED_TEST_INTERVALS = 2
# A scan that keeps an extended group alive through the E2 coast but has no
# strict camera edge used to throw the accumulated maturity away, so a single
# missing camera confirmation restarted the two-interval warm-up. Hold it while
# the member motion still tracks the prediction, bounded by the last strict
# confirmation. A coasting group still may not collapse publication: the camera
# has not re-confirmed the pair this scan, so both contacts stay published.
BOSCH_CAMERA_MATURITY_HOLD_NS = 200_000_000
# The OEM selected the same member of this exact set on the previous scan too,
# which is independent evidence that the set is one vehicle. Shorten the
# interval requirement for that case only; the geometry checks are unchanged.
BOSCH_CAMERA_MATURITY_WORD1_INTERVALS = 1
BOSCH_CAMERA_EXTENDED_MODE = BOSCH_CAMERA_EXTENDED_ACTIVE_TEST
BOSCH_CAMERA_HEADER = 0x738
BOSCH_CAMERA_FIRST_OBJECT = 0x739
BOSCH_CAMERA_LAST_OBJECT = 0x756
BOSCH_CAMERA_LAST_FAMILY = 0x760
BOSCH_CAMERA_SLOTS = 10
BOSCH_CAMERA_PERIOD_NS = 40_000_000
BOSCH_CAMERA_FRAME_TOLERANCE_NS = 25_000_000
BOSCH_CAMERA_OBSERVATION_GAP_NS = 160_000_000
BOSCH_CAMERA_E2_HOLD_NS = 250_000_000
BOSCH_CAMERA_SNAPSHOT_COUNT = 4
BOSCH_CAMERA_WIDTH_REF_M = 1.70
BOSCH_CAMERA_ANGLE_LSB = 1.0 / 4496.3
BOSCH_CAMERA_ASSOC_UNRESOLVED = 0
BOSCH_CAMERA_ASSOC_ASSIGNED = 1
BOSCH_CAMERA_ASSOC_AMBIGUOUS = 2
# camera class 6은 경험적으로 car-family 상태이지 truck enum이 아니다.
# 아래의 독립적인 폭/rigid-pair 증거를 모두 만족할 때만 대형차 P2를 보조하며,
# 기존 class-1 경로는 별도 분기로 그대로 유지한다.
BOSCH_TRUCK_P2_CLASS = 6
BOSCH_TRUCK_P2_CONFIRMATIONS = 5
BOSCH_TRUCK_P2_STATE_MAX = 16
BOSCH_TRUCK_P2_WIDTH_MIN_M = 2.40
BOSCH_TRUCK_P2_DD_MIN_M = 5.50
BOSCH_TRUCK_P2_DD_MAX_M = 9.00
BOSCH_TRUCK_P2_DY_MAX_M = 0.875
BOSCH_TRUCK_P2_DV_MAX_MPS = 0.50
BOSCH_TRUCK_A0_RECOVERY_HOLD_SCANS = 2
BOSCH_TRUCK_A0_RECOVERY_BEARING_EXCESS_RAD = 0.010
BOSCH_TRUCK_A0_RECOVERY_COST_MARGIN = 0.15


def _bosch_camera_signed(value, bits):
  sign = 1 << (bits - 1)
  return value - (1 << bits) if value & sign else value


@dataclass(slots=True)
class BoschCameraObject:
  obj_id: int = 0
  episode: int = 0
  long_m: float = 0.0
  lat_m: float = 0.0
  vrel_mps: float = 0.0
  width_m: float = 0.0
  class_code: int = -1
  angle_left: float = 0.0
  angle_right: float = 0.0


class BoschCameraCycleCache:
  """Bounded, allocation-free-per-frame cache for the 25 Hz camera family.

  A/B/C are accepted only with the current header counter.  A tiny fixed
  pre-header bank handles arbitrary ordering inside one CAN receive batch.
  Four preallocated snapshots retain the latest causal cycle when a newer
  camera cycle is already later than the Bosch scan being closed.
  """
  def __init__(self):
    self._counter = -1
    self._cycle_index = -1
    self._header_ns = -1
    self._n_objects = 0
    self._a = [0] * BOSCH_CAMERA_SLOTS
    self._b = [0] * BOSCH_CAMERA_SLOTS
    self._c = [0] * BOSCH_CAMERA_SLOTS
    self._a_ns = [-1] * BOSCH_CAMERA_SLOTS
    self._b_ns = [-1] * BOSCH_CAMERA_SLOTS
    self._c_ns = [-1] * BOSCH_CAMERA_SLOTS
    self._masks = [0, 0, 0]
    pending_size = 16 * BOSCH_CAMERA_SLOTS
    self._pending = [[0] * pending_size for _ in range(3)]
    self._pending_ns = [[-1] * pending_size for _ in range(3)]
    self._buffers = [[BoschCameraObject() for _ in range(BOSCH_CAMERA_SLOTS)]
                     for _ in range(BOSCH_CAMERA_SNAPSHOT_COUNT)]
    self._snapshot_cycle = [-1] * BOSCH_CAMERA_SNAPSHOT_COUNT
    self._snapshot_header_ns = [-1] * BOSCH_CAMERA_SNAPSHOT_COUNT
    self._snapshot_complete_ns = [-1] * BOSCH_CAMERA_SNAPSHOT_COUNT
    self._snapshot_count = [0] * BOSCH_CAMERA_SNAPSHOT_COUNT
    self._snapshot_next = 0
    self._last_seen_cycle = [-1000] * 256
    self._episode = [0] * 256
    self._next_episode = 0
    self.fault_ns = -1
    self.completed_cycles = 0
    self.rejected_cycles = 0

  @staticmethod
  def _counter_of(word):
    return (word >> 52) & 0xf

  def _unwrap(self, counter, timestamp_ns):
    if self._counter < 0:
      return 0
    elapsed = max(0, round((timestamp_ns - self._header_ns) / BOSCH_CAMERA_PERIOD_NS))
    wanted = (counter - self._counter) & 0xf
    step = elapsed + ((wanted - elapsed) & 0xf)
    if step - elapsed > 8:
      step -= 16
    return self._cycle_index + max(step, wanted)

  def _store(self, kind, slot, word, timestamp_ns):
    arrays = (self._a, self._b, self._c)
    times = (self._a_ns, self._b_ns, self._c_ns)
    arrays[kind][slot] = word
    times[kind][slot] = timestamp_ns
    self._masks[kind] |= 1 << slot

  def _pending_store(self, counter, kind, slot, word, timestamp_ns):
    index = counter * BOSCH_CAMERA_SLOTS + slot
    if timestamp_ns >= self._pending_ns[kind][index]:
      self._pending[kind][index] = word
      self._pending_ns[kind][index] = timestamp_ns

  def _start_cycle(self, counter, n_objects, timestamp_ns):
    if self._header_ns >= 0 and timestamp_ns <= self._header_ns:
      return False
    if self._counter >= 0 and self._n_objects and not self._is_complete():
      self.fault_ns = timestamp_ns
      self.rejected_cycles += 1
    cycle_index = self._unwrap(counter, timestamp_ns)
    self._counter = counter
    self._cycle_index = cycle_index
    self._header_ns = timestamp_ns
    self._n_objects = n_objects
    self._masks[:] = (0, 0, 0)
    self._a_ns[:] = [-1] * BOSCH_CAMERA_SLOTS
    self._b_ns[:] = [-1] * BOSCH_CAMERA_SLOTS
    self._c_ns[:] = [-1] * BOSCH_CAMERA_SLOTS
    for kind in range(3):
      for slot in range(BOSCH_CAMERA_SLOTS):
        index = counter * BOSCH_CAMERA_SLOTS + slot
        pending_ns = self._pending_ns[kind][index]
        if pending_ns >= 0 and abs(pending_ns - timestamp_ns) <= BOSCH_CAMERA_FRAME_TOLERANCE_NS:
          self._store(kind, slot, self._pending[kind][index], pending_ns)
        self._pending_ns[kind][index] = -1
    self._try_complete()
    return True

  def _is_complete(self):
    required = (1 << self._n_objects) - 1
    return all(mask & required == required for mask in self._masks)

  def _try_complete(self):
    if self._counter < 0 or not self._is_complete():
      return
    if any(cycle == self._cycle_index for cycle in self._snapshot_cycle):
      return
    seen_ids = 0
    for slot in range(self._n_objects):
      a = self._a[slot]
      if not (a & ((1 << 48) - 1)):
        self.fault_ns = max(self._header_ns, self._a_ns[slot])
        self.rejected_cycles += 1
        return
      obj_id = a & 0xff
      bit = 1 << obj_id
      if seen_ids & bit:
        self.fault_ns = max(self._header_ns, self._a_ns[slot])
        self.rejected_cycles += 1
        return
      seen_ids |= bit
    target = self._snapshot_next
    objects = self._buffers[target]
    complete_ns = self._header_ns
    for slot in range(self._n_objects):
      a, b, c = self._a[slot], self._b[slot], self._c[slot]
      complete_ns = max(complete_ns, self._a_ns[slot], self._b_ns[slot], self._c_ns[slot])
      obj_id = a & 0xff
      if self._cycle_index - self._last_seen_cycle[obj_id] > 2:
        self._next_episode += 1
        self._episode[obj_id] = self._next_episode
      self._last_seen_cycle[obj_id] = self._cycle_index
      obj = objects[slot]
      obj.obj_id = obj_id
      obj.episode = self._episode[obj_id]
      obj.long_m = ((a >> 8) & 0xfff) * 0.0625
      obj.lat_m = _bosch_camera_signed((a >> 20) & 0xfff, 12) * 0.0625
      obj.vrel_mps = _bosch_camera_signed((a >> 40) & 0xfff, 12) * 0.0625
      obj.width_m = (b & 0x3f) * 0.05
      obj.class_code = int((b >> 48) & 0x3) + 4 * int((b >> 50) & 0x1)
      obj.angle_right = _bosch_camera_signed((c >> 18) & 0x1fff, 13) * BOSCH_CAMERA_ANGLE_LSB
      obj.angle_left = _bosch_camera_signed((c >> 31) & 0x1fff, 13) * BOSCH_CAMERA_ANGLE_LSB
    self._snapshot_cycle[target] = self._cycle_index
    self._snapshot_header_ns[target] = self._header_ns
    self._snapshot_complete_ns[target] = complete_ns
    self._snapshot_count[target] = self._n_objects
    self._snapshot_next = (target + 1) % BOSCH_CAMERA_SNAPSHOT_COUNT
    self.completed_cycles += 1

  def ingest(self, timestamp_ns, address, payload):
    if len(payload) != 8:
      self.fault_ns = timestamp_ns
      return False
    word = int.from_bytes(payload, 'little')
    if address == BOSCH_CAMERA_HEADER:
      n_objects = word & 0xf
      if n_objects > BOSCH_CAMERA_SLOTS:
        self.fault_ns = timestamp_ns
        self.rejected_cycles += 1
        return False
      return self._start_cycle(self._counter_of(word), n_objects, timestamp_ns)
    if not BOSCH_CAMERA_FIRST_OBJECT <= address <= BOSCH_CAMERA_LAST_OBJECT:
      return False
    offset = address - BOSCH_CAMERA_FIRST_OBJECT
    slot, kind = divmod(offset, 3)
    counter = self._counter_of(word)
    if (counter == self._counter and self._header_ns >= 0 and
        abs(timestamp_ns - self._header_ns) <= BOSCH_CAMERA_FRAME_TOLERANCE_NS):
      self._store(kind, slot, word, timestamp_ns)
      self._try_complete()
    else:
      self._pending_store(counter, kind, slot, word, timestamp_ns)
    return True

  def snapshot(self, timestamp_ns):
    best = -1
    best_cycle = -1
    for index in range(BOSCH_CAMERA_SNAPSHOT_COUNT):
      complete_ns = self._snapshot_complete_ns[index]
      if (complete_ns <= timestamp_ns and complete_ns > self.fault_ns and
          timestamp_ns - complete_ns <= BOSCH_CAMERA_OBSERVATION_GAP_NS and
          self._snapshot_cycle[index] > best_cycle):
        best, best_cycle = index, self._snapshot_cycle[index]
    if best < 0:
      return None
    return self._buffers[best], self._snapshot_count[best], self._snapshot_cycle[best], self._snapshot_complete_ns[best]


@dataclass(slots=True)
class _BoschExtendedHistory:
  members: tuple[int, ...]
  cam_key: int
  class_code: int
  last_confirm_ns: int
  # 현재 exact tuple의 직전 관측만 보관한다(최대 8 member). 과거 이력 원장이 아니다.
  stable_intervals: int = 0
  sample_ns: int = 0
  observations: tuple[tuple[float, float, float], ...] = ()
  # physical ID the OEM word1 selected inside this member set, or -1.
  oem_anchor: int = -1


@dataclass(slots=True)
class _BoschTruckPairHistory:
  members: tuple[int, int]
  camera_id: int
  episode: int
  last_ns: int
  confirmations: int
  dd: float
  dy: float
  dv: float
  camera_d: float
  camera_y: float
  camera_v: float
  camera_width: float
  recovery_age: int = 0


@dataclass(slots=True)
class _BoschExtendedRepresentative:
  members: frozenset[int]
  representative_pid: int
  timestamp_ns: int
  d_rel: float
  y_rel: float
  v_rel: float
  age: int


class BoschCameraExtendedGrouping:
  """A0/P2/G0/E2 그룹과 테스트 전용 M2 eligibility를 계산한다.

  OFF/SHADOW/안전용 ACTIVE는 baseline 관측을 보존한다. ACTIVE_TEST만
  allocator 이후 실제 publication을 축소한다. 별도 PID의 소실된 운동 이력을
  대표점이 이전하지 못하므로 longitudinal control 사용은 NO-GO다.
  """
  def __init__(self, mode=BOSCH_CAMERA_EXTENDED_MODE):
    if mode not in (BOSCH_CAMERA_EXTENDED_OFF, BOSCH_CAMERA_EXTENDED_SHADOW,
                    BOSCH_CAMERA_EXTENDED_ACTIVE, BOSCH_CAMERA_EXTENDED_ACTIVE_TEST):
      raise ValueError('invalid Bosch camera extended-grouping mode')
    self.mode = mode
    self.camera = BoschCameraCycleCache() if mode != BOSCH_CAMERA_EXTENDED_OFF else None
    self.histories: dict[tuple[int, ...], _BoschExtendedHistory] = {}
    self.truck_pair_histories: dict[tuple[int, int], _BoschTruckPairHistory] = {}
    self.representatives: list[_BoschExtendedRepresentative] = []
    self.last_ns = None
    self.last_groups: tuple[tuple[int, ...], ...] = ()
    self.last_association_count = 0
    self.last_associations = {}
    self.last_candidate_count = 0
    self.last_coast_count = 0
    self.max_state_count = 0
    self.representative_switches = 0
    self.max_representative_jump = [0.0, 0.0, 0.0]
    self.perf_sum = Counter()
    self.perf_max = Counter()
    self.perf_scans = 0
    self.mature_groups = ()
    self.maturity_resets = 0
    self.last_maturity_resets = 0
    self.test_scans = self.test_group_scans = self.test_mature_scans = 0
    self.test_maturity_reached = 0
    self.test_full_groups = self.test_coast_groups = 0
    self.last_camera_ns = None
    self.last_truck_edge_count = 0
    self.max_truck_pair_state = 0
    self.last_truck_recovery_count = 0

  @staticmethod
  def _geometry(a, b, v_ego, yaw_rate):
    dd = abs(a.d_rel - b.d_rel)
    dy = abs(a.y_rel - b.y_rel)
    dv = abs(a.v_rel - b.v_rel)
    conflict = False
    if math.isfinite(v_ego):
      yaw = yaw_rate if yaw_rate is not None and math.isfinite(yaw_rate) else 0.0
      wa = abs(a.v_rel + v_ego - yaw * a.y_rel)
      wb = abs(b.v_rel + v_ego - yaw * b.y_rel)
      conflict = min(wa, wb) <= 0.6 and max(wa, wb) >= 1.4
    return dd, dy, dv, 3.0 < dd <= 12.0 and dy <= 1.5 and dv <= 1.5 and not conflict

  @staticmethod
  def _associate(obj, camera_objects, count):
    bearing = math.atan2(-obj.y_rel, max(obj.d_rel, 0.5))
    best = second = math.inf
    best_obj = None
    passed = 0
    for index in range(count):
      cam = camera_objects[index]
      extra = max(cam.width_m - BOSCH_CAMERA_WIDTH_REF_M, 0.0)
      l_pos = max(5.0 + 4.0 * extra, 0.5)
      l_neg = max(8.0 * extra, 0.0)
      d_long = cam.long_m - obj.d_rel
      d_lat = cam.lat_m + obj.y_rel
      lo, hi = min(cam.angle_left, cam.angle_right), max(cam.angle_left, cam.angle_right)
      bear_out = max(lo - bearing, bearing - hi)
      if d_long > l_pos or d_long < -l_neg or bear_out > 0.020 or abs(d_lat) > 2.5:
        continue
      passed += 1
      half = (hi - lo) * 0.5
      bear_n = (half + bear_out) / max(half + 0.020, 1e-6)
      cost = (4.0 * bear_n + 2.0 * abs(cam.vrel_mps - obj.v_rel) / 3.0 +
              abs(d_lat) / 2.5 + 0.5 * abs(d_long) / max(l_pos + l_neg, 1e-6))
      if cost < best:
        second, best, best_obj = best, cost, cam
      elif cost < second:
        second = cost
    if not passed:
      return BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1
    if passed >= 2 and second - best < 0.15:
      return BOSCH_CAMERA_ASSOC_AMBIGUOUS, -1, -1
    return BOSCH_CAMERA_ASSOC_ASSIGNED, best_obj.episode, best_obj.class_code

  @staticmethod
  def _complete_link(objects, edges, previous):
    ids = sorted(obj.physical_track_id for obj in objects)
    active = {pid for pair in edges for pid in pair}
    groups = [{pid} for pid in ids if pid in active]
    groups += [{pid} for pid in ids if pid not in active]
    previous = [set(group) for group in previous]
    while True:
      choices = []
      for i, a in enumerate(groups):
        for j in range(i + 1, len(groups)):
          b = groups[j]
          if len(a) + len(b) > 8:
            continue
          cross = [tuple(sorted((x, y))) for x in a for y in b]
          if all(pair in edges for pair in cross):
            overlap = any(old & a and old & b for old in previous)
            choices.append((not overlap, max(edges[pair] for pair in cross), min(a), min(b), i, j))
      if not choices:
        break
      *_, i, j = min(choices)
      groups[i] |= groups.pop(j)
    return groups

  def _choose_representatives(self, groups, by_pid, timestamp_ns, yaw_rate):
    multi = [set(group) for group in groups if len(group) > 1]
    matches = {}
    claims = []
    for i, group in enumerate(multi):
      for j, prior in enumerate(self.representatives):
        overlap = len(group & prior.members)
        if overlap:
          score = 10 * overlap + 3 * (prior.representative_pid in group) + min(prior.age, 1000) * 1e-5
          claims.append((-score, min(group), prior.representative_pid, i, j))
    used_current, used_prior = set(), set()
    for _, _, _, i, j in sorted(claims):
      if i not in used_current and j not in used_prior:
        matches[i] = self.representatives[j]
        used_current.add(i)
        used_prior.add(j)
    new_states = []
    representatives = {}
    for i, group in enumerate(multi):
      members = [by_pid[pid] for pid in group]
      prior = matches.get(i)
      if prior is not None:
        dt = (timestamp_ns - prior.timestamp_ns) / 1e9
        angle = -(yaw_rate if yaw_rate is not None and math.isfinite(yaw_rate) else 0.0) * dt
        dx = prior.d_rel + prior.v_rel * dt
        px = dx * math.cos(angle) - prior.y_rel * math.sin(angle)
        py = dx * math.sin(angle) + prior.y_rel * math.cos(angle)
        def cost(obj):
          continuity = abs(obj.d_rel - px) + .5 * abs(obj.y_rel - py) + .5 * abs(obj.v_rel - prior.v_rel)
          return (continuity, obj.physical_track_id != prior.representative_pid,
                  not obj.vision_supported, not obj.oem_selected, -obj.age_scans, obj.physical_track_id)
      else:
        ordered = sorted(obj.d_rel for obj in members)
        middle = len(ordered) // 2
        median = ordered[middle] if len(ordered) & 1 else (ordered[middle - 1] + ordered[middle]) * .5
        def cost(obj):
          return (abs(obj.d_rel - median), False, not obj.vision_supported,
                  not obj.oem_selected, -obj.age_scans, obj.physical_track_id)
      # publication_view hides every member of a mature group except this one,
      # so this choice decides which surface of one vehicle downstream keeps.
      # Every pair inside a group passed 3.0 < dd <= 12.0, so range order is
      # unambiguous by at least 3 m and cannot chatter on measurement noise.
      # Prefer the member the OEM selected; otherwise the nearest, because
      # hiding the nearest surface of a vehicle overstates the lead range.
      rep = min(members, key=lambda obj: (not obj.oem_selected, obj.d_rel, cost(obj)))
      if prior is not None and rep.physical_track_id != prior.representative_pid:
        self.representative_switches += 1
        self.max_representative_jump[0] = max(self.max_representative_jump[0], abs(rep.d_rel - px))
        self.max_representative_jump[1] = max(self.max_representative_jump[1], abs(rep.y_rel - py))
        self.max_representative_jump[2] = max(self.max_representative_jump[2], abs(rep.v_rel - prior.v_rel))
      representatives[tuple(sorted(group))] = rep
      new_states.append(_BoschExtendedRepresentative(
        frozenset(group), rep.physical_track_id, timestamp_ns, rep.d_rel, rep.y_rel, rep.v_rel,
        prior.age + 1 if prior is not None else 1))
    self.representatives = new_states
    return representatives

  def _maturity(self, members, episode, class_code, timestamp_ns, by_pid, yaw_rate,
                *, confirmed=True, confirm_ns=None):
    # 연구 M2 정의 그대로: 첫 confirmed scan은 0, 두 이전 안정 간격 후 2.
    # camera ID/episode만으로 연속성을 인정하지 않고 모든 member의 d/y/v를 확인한다.
    prior = self.histories.get(members)
    age = 0
    if prior is not None and prior.cam_key == episode and prior.observations:
      dt = (timestamp_ns - prior.sample_ns) * 1e-9
      if 0 < dt <= .160:
        angle = -(yaw_rate if yaw_rate is not None and math.isfinite(yaw_rate) else 0.) * dt
        co, si = math.cos(angle), math.sin(angle)
        for pid, (d, y, v) in zip(members, prior.observations):
          obj = by_pid[pid]
          x = d + v * dt
          if not (abs(obj.d_rel - (x * co - y * si)) <= .5 + 2.5 * dt * dt and
                  abs(obj.y_rel - (x * si + y * co)) <= .0625 + 2 * dt and
                  abs(obj.v_rel - v) <= .25 + 5 * dt):
            break
        else:
          # confirmed=False is a coast: this scan has no strict camera edge. The
          # same motion-continuity test above is still required; only the
          # accumulated interval survives instead of being discarded.
          age = (min(prior.stable_intervals + 1, BOSCH_CAMERA_EXTENDED_TEST_INTERVALS)
                 if confirmed else prior.stable_intervals)
    anchor = -1
    for pid in members:
      if by_pid[pid].oem_selected:
        anchor = pid
        break
    return _BoschExtendedHistory(members, episode, class_code,
                                 timestamp_ns if confirm_ns is None else confirm_ns,
                                 age, timestamp_ns,
                                 tuple((by_pid[p].d_rel, by_pid[p].y_rel, by_pid[p].v_rel) for p in members),
                                 anchor)

  @staticmethod
  def _truck_pair_continuous(prior, timestamp_ns, dd, dy, dv, camera):
    dt = (timestamp_ns - prior.last_ns) * 1e-9
    return (
      0 < dt <= BOSCH_CAMERA_OBSERVATION_GAP_NS * 1e-9 and
      prior.episode == camera.episode and prior.camera_id == camera.obj_id and
      abs(dd - prior.dd) <= .50 and abs(dy - prior.dy) <= .375 and abs(dv - prior.dv) <= .25 and
      abs(camera.long_m - (prior.camera_d + prior.camera_v * dt)) <= .75 and
      abs(camera.lat_m - prior.camera_y) <= .125 and
      abs(camera.vrel_mps - prior.camera_v) <= .50 and
      abs(camera.width_m - prior.camera_width) <= .10)

  @staticmethod
  def _truck_a0_bearing_near_miss(obj, anchor, camera_objects, count):
    """Frozen A0 바로 바깥의 명확한 bearing miss인지 확인한다.

    A0 verdict 자체는 바꾸지 않는다. 이미 seed된 대형차 pair의 빠진 한 node가
    같은 camera object에 대해 다른 frozen gate를 모두 통과하고, raw cost도
    경쟁 object보다 명확히 작을 때만 truck P2용 보강 evidence로 사용한다.
    """
    bearing = math.atan2(-obj.y_rel, max(obj.d_rel, .5))
    anchor_cost = math.inf
    other_cost = math.inf
    anchor_gate = False
    for index in range(count):
      camera = camera_objects[index]
      extra = max(camera.width_m - BOSCH_CAMERA_WIDTH_REF_M, 0.)
      l_pos = max(5.0 + 4.0 * extra, .5)
      l_neg = max(8.0 * extra, 0.)
      d_long = camera.long_m - obj.d_rel
      d_lat = camera.lat_m + obj.y_rel
      lo, hi = min(camera.angle_left, camera.angle_right), max(camera.angle_left, camera.angle_right)
      bear_out = max(lo - bearing, bearing - hi)
      half = (hi - lo) * .5
      bear_n = (half + bear_out) / max(half + .020, 1e-6)
      cost = (4.0 * bear_n + 2.0 * abs(camera.vrel_mps - obj.v_rel) / 3.0 +
              abs(d_lat) / 2.5 + .5 * abs(d_long) / max(l_pos + l_neg, 1e-6))
      if camera.obj_id == anchor.obj_id and camera.episode == anchor.episode:
        anchor_cost = cost
        anchor_gate = (-l_neg <= d_long <= l_pos and abs(d_lat) <= 2.5 and
                       .020 < bear_out <= .020 + BOSCH_TRUCK_A0_RECOVERY_BEARING_EXCESS_RAD)
      else:
        other_cost = min(other_cost, cost)
    return anchor_gate and anchor_cost + BOSCH_TRUCK_A0_RECOVERY_COST_MARGIN <= other_cost

  def _truck_strict(self, timestamp_ns, geometry, associations, by_pid, camera_by_episode,
                    camera_objects=(), camera_count=0):
    candidates = []
    for key, (dd, dy, dv) in geometry.items():
      aa = associations.get(key[0], (BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1))
      ab = associations.get(key[1], (BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1))
      if not (aa[0] == BOSCH_CAMERA_ASSOC_ASSIGNED and ab[0] == BOSCH_CAMERA_ASSOC_ASSIGNED and
              aa[1] == ab[1] and aa[1] >= 0 and aa[2] == ab[2] == BOSCH_TRUCK_P2_CLASS):
        continue
      camera = camera_by_episode.get(aa[1])
      fresh = (camera is not None and self.last_camera_ns is not None and
               0 <= timestamp_ns - self.last_camera_ns <= BOSCH_CAMERA_OBSERVATION_GAP_NS and
               by_pid[key[0]].timestamp_ns == by_pid[key[1]].timestamp_ns == timestamp_ns)
      if not (fresh and camera.width_m >= BOSCH_TRUCK_P2_WIDTH_MIN_M and
              BOSCH_TRUCK_P2_DD_MIN_M <= dd <= BOSCH_TRUCK_P2_DD_MAX_M and
              dy <= BOSCH_TRUCK_P2_DY_MAX_M and dv <= BOSCH_TRUCK_P2_DV_MAX_MPS):
        continue
      score = dd / 12.0 + dy / 1.5 + dv / 1.5
      candidates.append((score, key, dd, dy, dv, camera))

    # 비정상적으로 조밀한 return에서도 fail-open으로 state 상한을 지킨다.
    candidates.sort(key=lambda row: (row[0], row[1]))
    next_history = {}
    strict = {}
    for score, key, dd, dy, dv, camera in candidates[:BOSCH_TRUCK_P2_STATE_MAX]:
      prior = self.truck_pair_histories.get(key)
      confirmations = prior.confirmations + 1 if prior is not None and self._truck_pair_continuous(
        prior, timestamp_ns, dd, dy, dv, camera) else 1
      state = _BoschTruckPairHistory(key, camera.obj_id, camera.episode, timestamp_ns, confirmations,
                                     dd, dy, dv, camera.long_m, camera.lat_m, camera.vrel_mps, camera.width_m)
      next_history[key] = state
      if confirmations >= BOSCH_TRUCK_P2_CONFIRMATIONS:
        strict[key] = score

    recovered = 0
    recovery_candidates = []
    if len(next_history) < BOSCH_TRUCK_P2_STATE_MAX:
      for key, prior in self.truck_pair_histories.items():
        if key in next_history or key not in geometry or prior.confirmations < BOSCH_TRUCK_P2_CONFIRMATIONS:
          continue
        prior_rep = next((rep for rep in self.representatives
                          if len(rep.members) == len(key) and all(pid in rep.members for pid in key)), None)
        if prior_rep is None or prior_rep.representative_pid not in by_pid:
          continue
        aa = associations.get(key[0], (BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1))
        ab = associations.get(key[1], (BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1))
        verdicts = (aa, ab)
        assigned = [index for index, value in enumerate(verdicts) if value[0] == BOSCH_CAMERA_ASSOC_ASSIGNED]
        if len(assigned) != 1:
          continue
        assigned_index = assigned[0]
        missing_index = 1 - assigned_index
        anchor_verdict, missing_verdict = verdicts[assigned_index], verdicts[missing_index]
        if (missing_verdict[0] != BOSCH_CAMERA_ASSOC_UNRESOLVED or
            anchor_verdict[1] != prior.episode or anchor_verdict[2] != BOSCH_TRUCK_P2_CLASS):
          continue
        camera = camera_by_episode.get(anchor_verdict[1])
        dd, dy, dv = geometry[key]
        fresh = (camera is not None and camera.obj_id == prior.camera_id and self.last_camera_ns is not None and
                 0 <= timestamp_ns - self.last_camera_ns <= BOSCH_CAMERA_OBSERVATION_GAP_NS and
                 by_pid[key[0]].timestamp_ns == by_pid[key[1]].timestamp_ns == timestamp_ns)
        absolute = (fresh and camera.width_m >= BOSCH_TRUCK_P2_WIDTH_MIN_M and
                    BOSCH_TRUCK_P2_DD_MIN_M <= dd <= BOSCH_TRUCK_P2_DD_MAX_M and
                    dy <= BOSCH_TRUCK_P2_DY_MAX_M and dv <= BOSCH_TRUCK_P2_DV_MAX_MPS)
        recovery_age = prior.recovery_age + 1
        if (not absolute or recovery_age > BOSCH_TRUCK_A0_RECOVERY_HOLD_SCANS or
            not self._truck_pair_continuous(prior, timestamp_ns, dd, dy, dv, camera) or
            not self._truck_a0_bearing_near_miss(by_pid[key[missing_index]], camera, camera_objects, camera_count)):
          continue
        score = dd / 12.0 + dy / 1.5 + dv / 1.5
        recovery_candidates.append((score, key, dd, dy, dv, camera, prior.confirmations, recovery_age))

    recovery_candidates.sort(key=lambda row: (row[0], row[1]))
    capacity = BOSCH_TRUCK_P2_STATE_MAX - len(next_history)
    for score, key, dd, dy, dv, camera, confirmations, recovery_age in recovery_candidates[:capacity]:
      next_history[key] = _BoschTruckPairHistory(
        key, camera.obj_id, camera.episode, timestamp_ns, confirmations, dd, dy, dv,
        camera.long_m, camera.lat_m, camera.vrel_mps, camera.width_m, recovery_age)
      strict[key] = score
      recovered += 1
    self.truck_pair_histories = next_history
    self.max_truck_pair_state = max(self.max_truck_pair_state, len(next_history))
    self.last_truck_edge_count = len(strict)
    self.last_truck_recovery_count = recovered
    return strict

  def _record_perf(self, name, elapsed):
    self.perf_sum[name] += elapsed
    self.perf_max[name] = max(self.perf_max[name], elapsed)

  def update(self, timestamp_ns, objects, v_ego, yaw_rate=None):
    if self.mode == BOSCH_CAMERA_EXTENDED_OFF:
      return objects
    start = time.perf_counter_ns()
    prior_maturity = self.histories
    self.mature_groups = ()
    if self.last_ns is not None and timestamp_ns - self.last_ns > BOSCH_CAMERA_OBSERVATION_GAP_NS:
      self.histories = {}
      self.truck_pair_histories = {}
      self.representatives.clear()
    self.last_ns = timestamp_ns
    by_pid = {obj.physical_track_id: obj for obj in objects}
    ordered = sorted(objects, key=lambda obj: (obj.d_rel, obj.physical_track_id))
    geometry = {}
    candidate_nodes = set()
    for i, a in enumerate(ordered):
      for b in ordered[i + 1:]:
        dd = b.d_rel - a.d_rel
        if dd > 12.0:
          break
        if dd <= 3.0:
          continue
        dd, dy, dv, okay = self._geometry(a, b, v_ego, yaw_rate)
        if not okay:
          continue
        key = tuple(sorted((a.physical_track_id, b.physical_track_id)))
        geometry[key] = (dd, dy, dv)
        candidate_nodes.update(key)
    self.last_candidate_count = len(geometry)
    now = time.perf_counter_ns()
    self._record_perf('candidate', now - start)

    snapshot = self.camera.snapshot(timestamp_ns)
    self.last_camera_ns = snapshot[3] if snapshot is not None else None
    associations = {}
    camera_by_episode = {}
    camera_objects = ()
    camera_count = 0
    assoc_start = now
    if snapshot is not None and candidate_nodes:
      camera_objects, camera_count, _, _ = snapshot
      camera_by_episode = {camera_objects[index].episode: camera_objects[index] for index in range(camera_count)}
      for pid in candidate_nodes:
        associations[pid] = self._associate(by_pid[pid], camera_objects, camera_count)
    self.last_association_count = len(associations)
    self.last_associations = associations
    now = time.perf_counter_ns()
    self._record_perf('association', now - assoc_start)

    strict = {}
    strict_classes = {}
    strict_episodes = {}
    for key, (dd, dy, dv) in geometry.items():
      aa = associations.get(key[0], (BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1))
      ab = associations.get(key[1], (BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1))
      if (aa[0] == BOSCH_CAMERA_ASSOC_ASSIGNED and ab[0] == BOSCH_CAMERA_ASSOC_ASSIGNED and
          aa[1] == ab[1] and aa[1] >= 0 and aa[2] == 1 and ab[2] == 1):
        strict[key] = dd / 12.0 + dy / 1.5 + dv / 1.5
        strict_classes[key] = 1
        strict_episodes[key] = aa[1]
    truck_strict = self._truck_strict(timestamp_ns, geometry, associations, by_pid, camera_by_episode,
                                      camera_objects, camera_count)
    strict.update(truck_strict)
    strict_classes.update((key, BOSCH_TRUCK_P2_CLASS) for key in truck_strict)
    strict_episodes.update((key, self.truck_pair_histories[key].episode) for key in truck_strict)

    e2_start = time.perf_counter_ns()
    edges = dict(strict)
    next_history = {}
    coast_groups = []
    for members, history in list(self.histories.items()):
      pairs = [tuple(sorted((members[i], members[j]))) for i in range(len(members)) for j in range(i + 1, len(members))]
      verdicts = [associations.get(pid, (BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1)) for pid in members]
      assigned = [value for value in verdicts if value[0] == BOSCH_CAMERA_ASSOC_ASSIGNED]
      okay = (all(pid in by_pid for pid in members) and all(pair in geometry for pair in pairs) and
              timestamp_ns - history.last_confirm_ns <= BOSCH_CAMERA_E2_HOLD_NS and
              not any(value[0] == BOSCH_CAMERA_ASSOC_AMBIGUOUS for value in verdicts) and
              not any(value[1] != history.cam_key or value[2] != history.class_code for value in assigned) and
              any(value[1] == history.cam_key for value in assigned))
      if not okay:
        continue
      if not all(pair in strict for pair in pairs):
        member_set = set(members)
        # A coast may preserve exactly this confirmed set, never recruit a new
        # node through a currently-confirmed cross edge.
        for pair in [pair for pair in edges if bool(member_set & set(pair)) and not set(pair) <= member_set]:
          del edges[pair]
        for pair in pairs:
          dd, dy, dv = geometry[pair]
          edges[pair] = dd / 12.0 + dy / 1.5 + dv / 1.5
        coast_groups.append(members)
        next_history[members] = history
    self.last_coast_count = len(coast_groups)
    now = time.perf_counter_ns()
    self._record_perf('e2', now - e2_start)

    group_start = now
    previous = [state.members for state in self.representatives]
    groups = self._complete_link(objects, edges, previous)
    now = time.perf_counter_ns()
    self._record_perf('group', now - group_start)

    output_start = now
    self._choose_representatives(groups, by_pid, timestamp_ns, yaw_rate)
    mature = []
    for group in groups:
      if len(group) < 2:
        continue
      members = tuple(sorted(group))
      pairs = [tuple(sorted((members[i], members[j]))) for i in range(len(members)) for j in range(i + 1, len(members))]
      if all(pair in strict for pair in pairs):
        episode = strict_episodes[pairs[0]]
        class_code = strict_classes[pairs[0]]
        fresh = (self.last_camera_ns is not None and
                 0 <= timestamp_ns - self.last_camera_ns <= BOSCH_CAMERA_OBSERVATION_GAP_NS and
                 all(by_pid[p].timestamp_ns == timestamp_ns for p in members))
        if self.mode == BOSCH_CAMERA_EXTENDED_ACTIVE_TEST and fresh:
          state = self._maturity(members, episode, class_code, timestamp_ns, by_pid, yaw_rate)
          next_history[members] = state
          prior = self.histories.get(members)
          required = BOSCH_CAMERA_EXTENDED_TEST_INTERVALS
          if prior is not None and state.oem_anchor >= 0 and prior.oem_anchor == state.oem_anchor:
            required = BOSCH_CAMERA_MATURITY_WORD1_INTERVALS
          if state.stable_intervals >= required:
            mature.append(members)
            if prior is None or prior.stable_intervals < required:
              self.test_maturity_reached += 1
        else:
          next_history[members] = _BoschExtendedHistory(members, episode, class_code, timestamp_ns)
      elif members not in next_history:
        raise AssertionError('E2 coast created an unconfirmed extended member set')
      else:
        # E2 association은 유지되지만 이 scan에 strict edge가 없는 coast다. 마지막
        # strict 확인 이후 bounded window 안이고 모든 member의 운동 연속성이 계속
        # 성립하면 누적 interval을 보존한다. coast 중에는 mature에 넣지 않으므로
        # publication은 축소되지 않는다(camera 재확인 전에는 두 contact 유지).
        state = next_history[members]
        if (self.mode == BOSCH_CAMERA_EXTENDED_ACTIVE_TEST and state.observations and
            state.stable_intervals > 0 and
            timestamp_ns - state.last_confirm_ns <= BOSCH_CAMERA_MATURITY_HOLD_NS):
          next_history[members] = self._maturity(
            members, state.cam_key, state.class_code, timestamp_ns, by_pid, yaw_rate,
            confirmed=False, confirm_ns=state.last_confirm_ns)
        else:
          next_history[members] = _BoschExtendedHistory(members, state.cam_key, state.class_code,
                                                        state.last_confirm_ns)
    self.last_maturity_resets = sum(bool(old.observations) and
      (key not in next_history or not next_history[key].observations or next_history[key].stable_intervals == 0)
      for key, old in prior_maturity.items())
    self.maturity_resets += self.last_maturity_resets
    self.mature_groups = tuple(mature)
    self.histories = next_history
    self.max_state_count = max(self.max_state_count, len(self.histories))
    self.last_groups = tuple(sorted(tuple(sorted(group)) for group in groups if len(group) > 1))
    if self.mode == BOSCH_CAMERA_EXTENDED_ACTIVE_TEST:
      self.test_scans += 1
      self.test_group_scans += len(self.last_groups)
      self.test_mature_scans += len(self.mature_groups)
      self.test_full_groups += len(self.last_groups) - self.last_coast_count
      self.test_coast_groups += self.last_coast_count
    # qualifier와 allocator 입력은 모든 모드에서 baseline 그대로 보존한다.
    result = objects
    done = time.perf_counter_ns()
    self._record_perf('output', done - output_start)
    self._record_perf('total', done - start)
    self.perf_scans += 1
    return result

  def perf_fields(self):
    count = max(self.perf_scans, 1)
    fields = ' '.join(
      f'camera_ext_{name}_ms_avg={self.perf_sum[name] * 1e-6 / count:.3f} '
      f'camera_ext_{name}_ms_max={self.perf_max[name] * 1e-6:.3f}'
      for name in ('candidate', 'association', 'group', 'e2', 'output', 'total'))
    fields += (f' camera_ext_candidates={self.last_candidate_count} camera_ext_nodes={self.last_association_count}'
               f' camera_ext_coasts={self.last_coast_count} camera_ext_state_peak={self.max_state_count}'
               f' camera_ext_truck_edges={self.last_truck_edge_count}'
               f' camera_ext_truck_a0_recoveries={self.last_truck_recovery_count}'
               f' camera_ext_truck_state_peak={self.max_truck_pair_state}'
               f' camera_ext_rep_switches={self.representative_switches}'
               f' camera_ext_test_scans={self.test_scans}'
               f' camera_ext_group_scans={self.test_group_scans} camera_ext_mature_group_scans={self.test_mature_scans}'
               f' camera_ext_maturity_reached={self.test_maturity_reached}'
               f' camera_ext_full_group_scans={self.test_full_groups} camera_ext_coast_group_scans={self.test_coast_groups}'
               f' camera_ext_groups={len(self.last_groups)} camera_ext_mature_groups={len(self.mature_groups)}'
               f' camera_ext_maturity_resets={self.maturity_resets}'
               f' camera_ext_camera_age_ms={(self.last_ns - self.last_camera_ns) * 1e-6 if self.last_camera_ns is not None else -1:.3f}'
               f' camera_ext_camera_reject={self.camera.rejected_cycles}')
    self.perf_sum.clear()
    self.perf_max.clear()
    self.perf_scans = 0
    return fields


def bosch_numpy_linear_sum_assignment(cost_matrix, *, potentials=False):
  """Jonker-Volgenant assignment. potentials also returns the dual row/column
  potentials, which satisfy cost - u - v >= 0 with equality on every match."""
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
    empty = np.empty(0, dtype=int)
    return (empty, empty, np.empty(0), np.empty(0)) if potentials else (empty, empty)
  u, v = np.zeros(n + 1), np.zeros(m + 1)
  p, way = np.zeros(m + 1, dtype=int), np.zeros(m + 1, dtype=int)
  # Scratch reused across rows. The search only ever touches columns 1..m, so
  # every vector stays full width and is masked in place: the arithmetic per
  # column is the same as the gather/scatter form, without its temporaries.
  reduced = np.empty(m)
  improve = np.empty(m, dtype=bool)
  minimum = np.empty(m + 1)
  used = np.empty(m + 1, dtype=bool)
  unused = np.empty(m + 1, dtype=bool)
  path_rows = np.empty(m + 1, dtype=int)
  v_tail, minimum_tail, way_tail = v[1:], minimum[1:], way[1:]
  for row in range(1, n + 1):
    p[0] = row
    minimum.fill(np.inf)
    used.fill(False)
    unused.fill(True)
    column = 0
    depth = 0
    while True:
      used[column] = True
      unused[column] = False
      # A used column is never revisited, so parking it at +inf lets the plain
      # argmin below select exactly the column the masked argmin selected.
      minimum[column] = np.inf
      current_row = p[column]
      path_rows[depth] = current_row
      depth += 1
      np.subtract(cost[current_row - 1], u[current_row], out=reduced)
      np.subtract(reduced, v_tail, out=reduced)
      np.less(reduced, minimum_tail, out=improve)
      np.logical_and(improve, unused[1:], out=improve)
      np.copyto(minimum_tail, reduced, where=improve)
      way_tail[improve] = column
      next_column = int(np.argmin(minimum))
      delta = minimum[next_column]
      if not np.isfinite(delta):
        raise ValueError('cost matrix is infeasible')
      u[path_rows[:depth]] += delta
      np.subtract(v, delta, out=v, where=used)
      np.subtract(minimum, delta, out=minimum, where=unused)
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
  if potentials:
    # Restate the duals in the caller's orientation, dropping the sentinel slot.
    row_potential, column_potential = (v[1:], u[1:]) if transposed else (u[1:], v[1:])
    return rows[order], columns[order], row_potential, column_potential
  return rows[order], columns[order]


try:
  from scipy.optimize import linear_sum_assignment as bosch_linear_sum_assignment
except ImportError:
  bosch_linear_sum_assignment = bosch_numpy_linear_sum_assignment


def _bosch_integer(value: object, label: str, minimum: int, maximum: int | None = None) -> None:
  # An exact int type already excludes bool and every non-Integral, so the scan
  # path never pays for the abstract-base-class lookup. Anything else keeps the
  # original predicate, including its short-circuit before the comparisons.
  if type(value) is int:
    if minimum <= value and (maximum is None or value <= maximum):
      return
  elif not (isinstance(value, bool) or not isinstance(value, Integral) or value < minimum
            or (maximum is not None and value > maximum)):
    return
  raise ValueError(f"{label} must be an integer in [{minimum}, {maximum}]")


def _bosch_finite(value: object, label: str) -> None:
  if type(value) is float:
    if math.isfinite(value):
      return
  elif not (isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value)):
    return
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
    # Around twenty of these are built per scan and the decoder always presents
    # exact int/float, so recognise that shape inline and only enter the shared
    # predicate for anything else. Accepted values and messages are unchanged.
    timestamp_ns, slot, raw_word = self.timestamp_ns, self.slot, self.raw_word
    d_rel, y_rel, v_rel = self.d_rel, self.y_rel, self.v_rel
    if type(timestamp_ns) is not int or timestamp_ns < 0:
      _bosch_integer(timestamp_ns, "timestamp_ns", 0)
    if type(slot) is not int or not 0 <= slot <= 31:
      _bosch_integer(slot, "slot", 0, 31)
    if type(raw_word) is not int or not 0 <= raw_word <= 2**32 - 1:
      _bosch_integer(raw_word, "raw_word", 0, 2**32 - 1)
    # Every coordinate is checked for finiteness before any range is judged:
    # a value that fails both must still report the finiteness failure.
    if type(d_rel) is not float or not math.isfinite(d_rel):
      _bosch_finite(d_rel, "d_rel")
    if type(y_rel) is not float or not math.isfinite(y_rel):
      _bosch_finite(y_rel, "y_rel")
    if type(v_rel) is not float or not math.isfinite(v_rel):
      _bosch_finite(v_rel, "v_rel")
    if not 0 <= d_rel <= 255.75:
      raise ValueError("d_rel outside the supported Bosch decoded range")
    if not -32 <= y_rel <= 31.96875:
      raise ValueError("y_rel outside the supported Bosch decoded range")
    if not -128 <= v_rel <= 127.75:
      raise ValueError("v_rel outside the supported Bosch decoded range")
    if raw_word == BOSCH_INACTIVE_WORD or raw_word & (1 << 31):
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


@dataclass(frozen=True)
class BoschRawAssociationDecision:
  """One raw-return association, recorded for shadow diagnostics only.

  Positions are in the scan-time rotated frame the association actually uses,
  so residual_d_m and residual_bearing_rad are the quantities the distance and
  bearing gates were applied to. best_alternative_cost is the cheapest edge
  this detection did not take, which is what makes a swap visible offline.
  """
  timestamp_ns: int
  raw_track_id: int
  slot: int
  previous_slot: int | None
  created: bool
  recovered: bool
  age_scans: int
  d_rel: float
  y_rel: float
  v_rel: float
  predicted_d_rel: float | None
  predicted_y_rel: float | None
  residual_d_m: float | None
  residual_bearing_rad: float | None
  bearing_gate_rad: float | None
  chosen_cost: float | None
  best_alternative_cost: float | None
  candidate_count: int


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


def _bosch_admissible_cycle(size, adjacency, real):
  """Iterative Tarjan: True when a nontrivial component contains a real vertex.

  Every vertex of a nontrivial strongly connected component lies on a cycle, so
  such a component holding a real row or column means an alternating cycle of
  zero reduced cost re-matches that vertex: a second optimum. Cycles confined to
  the dummy block only permute which unmatched row pairs with which unmatched
  column, which no caller can observe.
  """
  index = [0] * size
  low = [0] * size
  seen = bytearray(size)
  on_stack = bytearray(size)
  stack = []
  counter = 1
  for root in range(size):
    if seen[root]:
      continue
    work = [(root, 0)]
    while work:
      vertex, position = work[-1]
      if not position:
        seen[vertex] = 1
        index[vertex] = low[vertex] = counter
        counter += 1
        stack.append(vertex)
        on_stack[vertex] = 1
      neighbours = adjacency[vertex]
      descended = False
      while position < len(neighbours):
        other = neighbours[position]
        position += 1
        if not seen[other]:
          work[-1] = (vertex, position)
          work.append((other, 0))
          descended = True
          break
        if on_stack[other] and index[other] < low[vertex]:
          low[vertex] = index[other]
      if descended:
        continue
      work.pop()
      if work:
        parent = work[-1][0]
        if low[vertex] < low[parent]:
          low[parent] = low[vertex]
      if low[vertex] == index[vertex]:
        members = []
        while True:
          other = stack.pop()
          on_stack[other] = 0
          members.append(other)
          if other == vertex:
            break
        if len(members) > 1 and any(real[member] for member in members):
          return True
  return False


def _bosch_rectangular_unique(rows, columns, row_edges, unmatched_cost, margin):
  """Certify an optional real matching without solving the square dummy block.

  For k real matches the square cost is sum(edges) + (a+b-2*k)*U.
  Keeping only real rows and charging 2*U for each private miss gives
  sum(edges) + (a-k)*2*U: the difference is the constant (b-a)*U.
  This preserves the optimum, but not the solver's choice among tied optima.
  Return only a strictly unique matching; every refusal keeps the square path.
  """
  a, b = len(rows), len(columns)
  tolerance = max(margin, 1e-12)
  # A square admissible cycle has at most a+b non-matching edges. Widen the
  # rectangular graph conservatively so a near tie refused by that certificate
  # cannot become a fast success merely because the solver found other duals.
  threshold = 4 * tolerance * (a + b + 1)
  if not math.isfinite(threshold):
    return None
  costs = np.full((a, a + b), np.inf)
  column_at = {vertex: j for j, vertex in enumerate(columns)}
  for i, vertex in enumerate(rows):
    costs[i, b + i] = 2 * unmatched_cost
    for other, cost in row_edges[vertex]:
      j = column_at.get(other)
      if j is None:
        return None
      costs[i, j] = cost
  try:
    solved_rows, solved_columns, u, v = bosch_numpy_linear_sum_assignment(costs, potentials=True)
  except ValueError:
    return None
  reduced = costs - u[:, None] - v[None, :]
  occupied = np.zeros(a + b, dtype=bool)
  occupied[solved_columns] = True
  if (len(solved_rows) != a or not np.isfinite(u).all() or not np.isfinite(v).all() or
      np.any(reduced < -tolerance) or np.any(np.abs(reduced[solved_rows, solved_columns]) > tolerance) or
      np.any(v > tolerance) or np.any(np.abs(v[~occupied]) > tolerance)):
    return None

  # Rectangular alternatives are alternating cycles OR alternating paths from
  # an occupied zero-potential column to a free column. A reservoir closes
  # exactly these paths into cycles; omitting it would falsely certify ties
  # that move a birth or miss. Each private dummy column belongs to only one
  # row, so any cycle containing a row changes an observable real assignment.
  reservoir = 2 * a + b
  adjacency = [[] for _ in range(reservoir + 1)]
  partner = dict(zip(solved_rows.tolist(), solved_columns.tolist()))
  for i, matched in partner.items():
    adjacency[a + matched].append(i)
    adjacency[i].extend(a + j for j in np.flatnonzero(reduced[i] <= threshold).tolist() if j != matched)
  for j in range(a + b):
    if not occupied[j]:
      adjacency[a + j].append(reservoir)
    elif v[j] >= -threshold:
      adjacency[reservoir].append(a + j)
  real = bytearray(reservoir + 1)
  real[:a] = b'\x01' * a
  if _bosch_admissible_cycle(reservoir + 1, adjacency, real):
    return None
  return {columns[j]: rows[i] for i, j in partner.items() if j < b}


def _bosch_component_matching(rows, columns, row_edges, unmatched_cost, margin=None):
  """Solve one component's augmented assignment, optionally certifying it.

  The augmented objective is a sum of independent per-component terms, so a
  component with a strictly unique optimum carries that same matching inside
  every global optimum. Certifying it therefore replaces the whole-scan solver
  without moving a single association.

  Returns (matching, unique), or None when the component is not self-contained
  or the solve fails. Passing no margin skips the certificate and reports
  unique False; the matching is still an optimum of this component, and with
  rows and columns in ascending order it is the one the whole-scan matrix
  reaches whenever that matrix's own tie choice is a property of the component.
  """
  if margin is not None:
    matching = _bosch_rectangular_unique(rows, columns, row_edges, unmatched_cost, margin)
    if matching is not None:
      return matching, True
  a, b = len(rows), len(columns)
  size = a + b
  costs = np.full((size, size), np.inf)
  column_at = {vertex: j for j, vertex in enumerate(columns)}
  for i, vertex in enumerate(rows):
    costs[i, b + i] = unmatched_cost
    for other, cost in row_edges[vertex]:
      j = column_at.get(other)
      if j is None:
        return None  # Not a closed component; leave it to the global solver.
      costs[i, j] = cost
  for j in range(b):
    costs[a + j, j] = unmatched_cost
  costs[a:, b:] = 0.
  try:
    if margin is None:
      solved_rows, solved_columns = bosch_numpy_linear_sum_assignment(costs)
    else:
      solved_rows, solved_columns, u, v = bosch_numpy_linear_sum_assignment(costs, potentials=True)
  except ValueError:
    return None
  partner = dict(zip(solved_rows.tolist(), solved_columns.tolist()))
  if len(partner) != size:
    return None
  matching = {columns[j]: rows[i] for i, j in partner.items() if i < a and j < b}
  if margin is None:
    return matching, False
  admissible = (costs - u[:, None] - v[None, :]) <= max(margin, 1e-12)
  adjacency = [[] for _ in range(2 * size)]
  for i in range(size):
    matched = partner[i]
    # Matching edges point column to row, admissible non-matching edges point
    # row to column, so every directed cycle is an alternating swap.
    adjacency[size + matched].append(i)
    outgoing = adjacency[i]
    for j in np.flatnonzero(admissible[i]).tolist():
      if j != matched:
        outgoing.append(size + j)
  real = bytearray(2 * size)
  for i in range(a):
    real[i] = 1
  for j in range(b):
    real[size + j] = 1
  return matching, not _bosch_admissible_cycle(2 * size, adjacency, real)


def _bosch_raw_unique_component(rows, columns, row_edges, column_edges, unmatched_cost):
  # Each real match replaces one birth and one miss, so maximizing the sum of
  # (2*unmatched_cost - edge_cost) is exactly the augmented objective minus a
  # constant. A unique real matching is independent of all dummy assignments.
  reward = 2 * unmatched_cost
  margin = 1e-12 * max(1.0, abs(reward)) * (len(rows) + len(columns) + 1)
  if not math.isfinite(margin):
    return None
  if len(rows) == 1 and len(columns) == 1:
    # The lone pair, which is most of them. It carries exactly one edge, so the
    # certificate is a single comparison; below that strictness the general
    # paths refuse as well, and the caller solves the pair itself.
    saving = reward - row_edges[rows[0]][0][1]
    if saving > margin:
      return {columns[0]: rows[0]}
    return {} if -saving > margin else None

  # Distinct strict vertex-best choices attain the sum of independent upper
  # bounds. This certifies the unique optimum, even in a large component.
  for vertices, adjacency, transposed in ((rows, row_edges, False), (columns, column_edges, True)):
    matching = {}
    used = set()
    for vertex in vertices:
      best, second, partner = 0.0, -math.inf, None
      for other, cost in adjacency[vertex]:
        saving = reward - cost
        if saving > best:
          best, second, partner = saving, best, other
        elif saving > second:
          second = saving
      if best - second <= margin or (partner is not None and partner in used):
        break
      if partner is not None:
        used.add(partner)
        if transposed:
          matching[vertex] = partner
        else:
          matching[partner] = vertex
    else:
      return matching

  # Exact bounded optional matching DP. Keep the runner-up, including equal
  # optima; ambiguous or numerically indistinguishable results are left to the
  # caller's tie path, which solves the component itself.
  if min(len(rows), len(columns)) > 6 or max(len(rows), len(columns)) > 12:
    # Too wide for the subset DP. Certify the component's own optimum instead;
    # only a genuinely tied component reaches the caller's tie path.
    solved = _bosch_component_matching(rows, columns, row_edges, unmatched_cost, margin)
    return solved[0] if solved is not None and solved[1] else None
  transposed = len(rows) < len(columns)
  small, large = (rows, columns) if transposed else (columns, rows)
  bits = {vertex: 1 << i for i, vertex in enumerate(small)}
  adjacency = column_edges if transposed else row_edges
  options = [[(bits[other], reward - cost,
               (vertex, other) if transposed else (other, vertex))
              for other, cost in adjacency[vertex]] for vertex in large]
  memo = {}

  def solve(index, occupied):
    if index == len(options):
      return 0.0, -math.inf, ()
    key = (index, occupied)
    cached = memo.get(key)
    if cached is not None:
      return cached
    best, second, matching = solve(index + 1, occupied)
    for bit, saving, pair in options[index]:
      if occupied & bit:
        continue
      child_best, child_second, child_matching = solve(index + 1, occupied | bit)
      candidate = saving + child_best
      if candidate > best:
        best, second, matching = candidate, best, (pair,) + child_matching
      elif candidate > second:
        second = candidate
      runner_up = saving + child_second
      if runner_up > second:
        second = runner_up
    result = (best, second, matching)
    memo[key] = result
    return result

  best, second, matching = solve(0, 0)
  return dict(matching) if best - second > margin else None


def _bosch_raw_assignment(n, m, row_edges, column_edges, unmatched_cost):
  components = []
  visited_rows, visited_columns = set(), set()
  for first in range(n):
    edges = row_edges[first]
    if not edges or first in visited_rows:
      continue
    if len(edges) == 1:
      column = edges[0][0]
      if len(column_edges[column]) == 1:
        # Two thirds of the components are one state and one return that
        # nothing else can reach; that is the component, so skip the walk.
        visited_rows.add(first)
        visited_columns.add(column)
        components.append(([first], [column]))
        continue
    rows, columns, pending = [], [], [first]
    while pending:
      vertex = pending.pop()
      if vertex >= 0:
        if vertex in visited_rows:
          continue
        visited_rows.add(vertex)
        rows.append(vertex)
        pending.extend(~column for column, _ in row_edges[vertex] if column not in visited_columns)
      else:
        column = ~vertex
        if column in visited_columns:
          continue
        visited_columns.add(column)
        columns.append(column)
        pending.extend(row for row, _ in column_edges[column] if row not in visited_rows)
    components.append((rows, columns))
  largest = max((len(rows) + len(columns) for rows, columns in components), default=0)
  assignment = {}
  certified = 0
  for rows, columns in components:
    result = _bosch_raw_unique_component(rows, columns, row_edges, column_edges, unmatched_cost)
    if result is not None:
      certified += 1
    else:
      # A genuinely tied component. Solve it on its own, presenting its rows and
      # columns in ascending order: the solver breaks ties by index, and inside
      # one component the whole-scan matrix presents exactly that order.
      solved = _bosch_component_matching(sorted(rows), sorted(columns), row_edges, unmatched_cost)
      if solved is not None:
        result = solved[0]
    if result is None:
      # Not self-contained, or the component solver refused it. Rebuild the
      # exact old matrix so this path stays a faithful last resort.
      costs = np.full((n + m, n + m), np.inf)
      for row, edges in enumerate(row_edges):
        costs[row, m + row] = unmatched_cost
        for column, cost in edges:
          costs[row, column] = cost
      for column in range(m):
        costs[n + column, column] = unmatched_cost
      costs[n:, m:] = 0.0
      ri, ci = bosch_linear_sum_assignment(costs)
      assignment = {int(column): int(row) for row, column in zip(ri, ci) if row < n and column < m}
      return assignment, len(components), largest, 0, True
    assignment.update(result)
  return assignment, len(components), largest, certified, False


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
    self.last_pair_possible = self.last_pair_candidates = 0
    self.last_component_count = self.last_largest_component = 0
    self.last_fast_components = self.last_tied_components = 0
    self.last_solver_fallback = False
    self.stats = {name: 0 for name in (
      "updates", "detections", "created", "deleted", "assignments",
      "cross_slot", "recovered", "coasted_track_scans", "max_active",
      "gated_pairs", "distance_rejections", "speed_rejections", "bearing_rejections",
      "yaw_compensated_updates", "yaw_unavailable_updates",
    )}
    # Shadow diagnostic switch, matching the group manager. Off leaves the scan
    # path unchanged; on only fills last_decisions. Neither setting gates a match.
    self.trace_decisions = False
    self.last_decisions: tuple[BoschRawAssociationDecision, ...] = ()

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
    states = self._states
    retained = []
    for key in sorted(states):
      state = states[key]
      if timestamp_ns - state.last_seen_scan_ns <= coast_ns:
        retained.append(state)
    expired_count = len(states) - len(retained)
    dt = 0.0 if self.last_timestamp_ns is None else (timestamp_ns - self.last_timestamp_ns) / 1e9
    angle = 0.0 if yaw_rate is None else yaw_rate * dt
    cosine, sine = math.cos(angle), math.sin(angle)
    predicted = []
    # Retained velocities and slots are read again by the gate loop; keep them
    # off the RawTrack/RawDetection property chain there.
    retained_speeds = []
    retained_slots = []
    for state in retained:
      detection = state.track.detection
      speed = detection.v_rel
      retained_speeds.append(speed)
      retained_slots.append(detection.slot)
      x = state.x + speed * dt
      predicted.append((cosine * x + sine * state.y, -sine * x + cosine * state.y))
    observed = []
    rotations = {}
    for point in current:
      rotation = rotations.get(point.timestamp_ns)
      if rotation is None:
        sample_dt = (timestamp_ns - point.timestamp_ns) / 1e9
        angle = 0.0 if yaw_rate is None else yaw_rate * sample_dt
        rotation = (sample_dt, math.cos(angle), math.sin(angle))
        rotations[point.timestamp_ns] = rotation
      sample_dt, cosine, sine = rotation
      x = point.d_rel + point.v_rel * sample_dt
      observed.append((cosine * x + sine * point.y_rel, -sine * x + cosine * point.y_rel))
    n, m = len(retained), len(current)
    predicted_bearings = [math.atan2(y, x) for x, y in predicted]
    observed_bearings = [math.atan2(y, x) for x, y in observed]
    predicted_x = [point[0] for point in predicted]
    observed_x = [point[0] for point in observed]
    row_edges, column_edges = [[] for _ in range(n)], [[] for _ in range(m)]
    distance_pairs = speed_rejections = bearing_rejections = gated_pairs = 0
    # Same keys the equivalent lambdas produced, read at C speed.
    order = sorted(range(m), key=observed_x.__getitem__)
    lower = upper = 0
    # Per-scan constants. bearing_gate_rad() is inlined below with exactly the
    # same thresholds and math.radians() values it would have returned.
    distance_gate = config.distance_gate_m
    speed_gate = config.speed_gate_mps
    same_slot_bonus = config.same_slot_bonus
    gate_near = math.radians(config.bearing_near_deg)
    gate_mid = math.radians(config.bearing_mid_deg)
    gate_far = math.radians(config.bearing_far_deg)
    gate_distant = math.radians(config.bearing_distant_deg)
    observed_speeds = [point.v_rel for point in current]
    observed_slots = [point.slot for point in current]
    window_gate = math.nextafter(distance_gate, math.inf)
    for row in sorted(range(n), key=predicted_x.__getitem__):
      pred_x = predicted_x[row]
      state_speed = retained_speeds[row]
      state_slot = retained_slots[row]
      predicted_bearing = predicted_bearings[row]
      row_edge = row_edges[row]
      # Cover values whose subtraction rounds onto the gate, including when
      # pred_x - gate cancels to zero. The actual gate below stays unchanged.
      minimum = math.nextafter(pred_x - window_gate, -math.inf)
      maximum = math.nextafter(pred_x + window_gate, math.inf)
      while lower < m and observed_x[order[lower]] < minimum:
        lower += 1
      upper = max(upper, lower)
      while upper < m and observed_x[order[upper]] <= maximum:
        upper += 1
      for index in range(lower, upper):
        col = order[index]
        obs_x = observed_x[col]
        delta_d = abs(obs_x - pred_x)
        if delta_d > distance_gate:
          continue
        distance_pairs += 1
        delta_v = abs(observed_speeds[col] - state_speed)
        if delta_v > speed_gate:
          speed_rejections += 1
          continue
        angle = observed_bearings[col] - predicted_bearing
        delta_bearing = abs(math.atan2(math.sin(angle), math.cos(angle)))
        nearest = pred_x if pred_x < obs_x else obs_x
        if nearest < 0.0:
          nearest = 0.0
        if nearest < 15:
          gate = gate_near
        elif nearest < 30:
          gate = gate_mid
        elif nearest < 60:
          gate = gate_far
        else:
          gate = gate_distant
        if delta_bearing > gate:
          bearing_rejections += 1
        else:
          gated_pairs += 1
          cost = ((delta_d / distance_gate)**2 + (delta_v / speed_gate)**2 + (delta_bearing / gate)**2) / 3
          if state_slot == observed_slots[col]:
            cost = max(0.0, cost - same_slot_bonus)
          row_edge.append((col, cost))
          column_edges[col].append((row, cost))
    assignment, components, largest, fast_components, fallback = _bosch_raw_assignment(
      n, m, row_edges, column_edges, config.unmatched_cost,
    )
    pending_stats = {"gated_pairs": gated_pairs, "distance_rejections": n * m - distance_pairs,
                     "speed_rejections": speed_rejections, "bearing_rejections": bearing_rejections}
    births = m - len(assignment)
    if self.next_id + births - 1 > BOSCH_MAX_RAW_TRACK_ID:
      raise OverflowError("raw return Int32 ID space exhausted; IDs must not wrap or be reused")

    # Commit only after all validation and assignment have succeeded.
    self.last_pair_possible, self.last_pair_candidates = n * m, distance_pairs
    self.last_component_count, self.last_largest_component = components, largest
    self.last_fast_components, self.last_solver_fallback = fast_components, fallback
    self.last_tied_components = 0 if fallback else components - fast_components
    self._update_count += 1
    update_count = self._update_count
    # A coasted state only advances its projection, so it is re-seated in place
    # rather than rebuilt. Rows that a detection claims are overwritten below,
    # so they are not carried over at all.
    claimed_rows = set(assignment.values())
    states = {}
    self._states = states
    for row, (state, (x, y)) in enumerate(zip(retained, predicted)):
      if row in claimed_rows:
        continue
      state.x = x
      state.y = y
      states[state.track.raw_track_id] = state
    output = []
    decisions = []
    assignments = cross_slot = recovered_count = created = 0
    for col, (point, (x, y)) in enumerate(zip(current, observed)):
      row = assignment.get(col)
      if row is not None:
        old = retained[row]
        old_track = old.track
        recovered = old.last_seen_update != update_count - 1
        track = BoschRawTrack(old_track.raw_track_id, point, old_track.age_scans + 1, recovered, old_track.slot)
        assignments += 1
        cross_slot += point.slot != old_track.slot
        recovered_count += recovered
      else:
        track = BoschRawTrack(self.next_id, point, 1)
        self.next_id += 1
        created += 1
      output.append(track)
      states[track.raw_track_id] = _BoschRawState(track, x, y, timestamp_ns, update_count)
      if self.trace_decisions:
        chosen = next((cost for other, cost in column_edges[col] if other == row), None)
        alternatives = sorted(cost for other, cost in column_edges[col] if other != row)
        bearing = None
        if row is not None:
          angle = observed_bearings[col] - predicted_bearings[row]
          bearing = math.atan2(math.sin(angle), math.cos(angle))
        decisions.append(BoschRawAssociationDecision(
          timestamp_ns=timestamp_ns, raw_track_id=track.raw_track_id, slot=point.slot,
          previous_slot=track.previous_slot, created=col not in assignment,
          recovered=track.recovered, age_scans=track.age_scans,
          d_rel=point.d_rel, y_rel=point.y_rel, v_rel=point.v_rel,
          predicted_d_rel=predicted[row][0] if row is not None else None,
          predicted_y_rel=predicted[row][1] if row is not None else None,
          residual_d_m=x-predicted[row][0] if row is not None else None,
          residual_bearing_rad=bearing,
          bearing_gate_rad=(config.bearing_gate_rad(max(0.0, min(predicted[row][0], x)))
                            if row is not None else None),
          chosen_cost=chosen, best_alternative_cost=alternatives[0] if alternatives else None,
          candidate_count=len(column_edges[col])))
    self.last_decisions = tuple(decisions)
    self.last_timestamp_ns = int(timestamp_ns)
    stats = self.stats
    for key, value in pending_stats.items():
      stats[key] += value
    stats["assignments"] += assignments
    stats["cross_slot"] += cross_slot
    stats["recovered"] += recovered_count
    stats["created"] += created
    stats["updates"] += 1
    stats["detections"] += m
    stats["deleted"] += expired_count
    stats["coasted_track_scans"] += n - len(assignment)
    stats["max_active"] = max(stats["max_active"], len(states))
    stats["yaw_compensated_updates" if yaw_rate is not None else "yaw_unavailable_updates"] += 1
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


@dataclass(frozen=True)
class BoschAssociationDecision:
  """One physical-ID assignment, recorded for shadow diagnostics only.

  The tracker never reads these records back, so enabling the trace cannot
  change a published object, ID, coordinate or timestamp. It exists to answer
  one question offline: how far is the state that inherits a physical ID from
  the state that ID last published, projected forward over the scan gap?

  member_overlap counts the raw members the assignment actually scored, which
  includes members retained only for coasting. observed_member_overlap counts
  the members of the previously published object. A carry with
  observed_member_overlap == 0 kept an ID through a coasted member alone.
  """
  timestamp_ns: int
  physical_track_id: int
  previous_physical_track_id: int | None
  assignment_mode: str
  dt_s: float
  member_raw_track_ids: tuple[int, ...]
  member_slots: tuple[int, ...]
  previous_member_raw_track_ids: tuple[int, ...]
  member_overlap: int
  observed_member_overlap: int
  representative_raw_track_id: int
  previous_representative_raw_track_id: int | None
  representative_changed: bool
  representative_still_a_member: bool
  d_rel: float
  y_rel: float
  v_rel: float
  predicted_d_rel: float | None
  predicted_y_rel: float | None
  predicted_v_rel: float | None
  residual_d_m: float | None
  residual_y_m: float | None
  residual_v_mps: float | None
  best_score: float | None
  second_score: float | None
  age_scans: int
  grouping_evidence: str


@dataclass
class _BoschPairEvidence:
  samples: deque = field(default_factory=deque)


@dataclass
class _BoschPhysicalState:
  observation: BoschPhysicalObject
  # Includes temporarily missing raw members, for internal coasting only.
  member_last_seen: dict[int, int]


def _bosch_physical_component(rows, columns, row_edges, column_edges, margin):
  """One component's maximum-weight matching, or None when it is not unique.

  Every score is strictly positive and an unassigned cluster costs nothing, so
  the augmented square matrix the whole-scan solver builds is minimised exactly
  by a maximum-weight matching of this graph. A component whose maximum is
  strictly unique therefore holds the same pairs inside every global optimum,
  whichever way that solver happened to break its own ties.
  """
  if len(rows) == 1:
    row = rows[0]
    best = second = 0.
    partner = None
    for column, score in row_edges[row]:
      if score > best:
        best, second, partner = score, best, column
      elif score > second:
        second = score
    return {row: partner} if best-second > margin else None
  if len(columns) == 1:
    column = columns[0]
    best = second = 0.
    partner = None
    for row, score in column_edges[column]:
      if score > best:
        best, second, partner = score, best, row
      elif score > second:
        second = score
    return {partner: column} if best-second > margin else None
  if len(rows) > 8 or len(columns) > 12:
    return None
  bits = {column: 1 << i for i, column in enumerate(columns)}
  options = [[(bits[column], score, (row, column)) for column, score in row_edges[row]] for row in rows]
  memo = {}

  def solve(index, occupied):
    # Keep the runner-up as well: equal optima are what the caller must not guess.
    if index == len(options):
      return 0., -math.inf, ()
    key = (index, occupied)
    cached = memo.get(key)
    if cached is not None:
      return cached
    best, second, matching = solve(index+1, occupied)
    for bit, score, pair in options[index]:
      if occupied & bit:
        continue
      child_best, child_second, child_matching = solve(index+1, occupied | bit)
      candidate = score+child_best
      if candidate > best:
        best, second, matching = candidate, best, (pair,)+child_matching
      elif candidate > second:
        second = candidate
      runner_up = score+child_second
      if runner_up > second:
        second = runner_up
    result = (best, second, matching)
    memo[key] = result
    return result

  best, second, matching = solve(0, 0)
  return dict(matching) if best-second > margin else None


def _bosch_physical_assignment(row_edges, column_edges):
  """Split the physical score graph into independent parts and solve each one.

  Returns the matching, or None when some part carries more than one optimum:
  that is the only case in which the whole-scan solver's own tie order is
  observable, and the caller then rebuilds exactly the matrix it always built.
  """
  assignment = {}
  seen_rows, seen_columns = set(), set()
  for first, edges in enumerate(row_edges):
    if not edges or first in seen_rows:
      continue
    rows, columns, pending, peak = [], [], [first], 0.
    while pending:
      vertex = pending.pop()
      if vertex >= 0:
        if vertex in seen_rows:
          continue
        seen_rows.add(vertex)
        rows.append(vertex)
        for column, score in row_edges[vertex]:
          if score > peak:
            peak = score
          if column not in seen_columns:
            pending.append(~column)
      else:
        column = ~vertex
        if column in seen_columns:
          continue
        seen_columns.add(column)
        columns.append(column)
        pending.extend(row for row, _ in column_edges[column] if row not in seen_rows)
    result = _bosch_physical_component(rows, columns, row_edges, column_edges,
                                       1e-12*peak*(len(rows)+len(columns)+1))
    if result is None:
      return None
    assignment.update(result)
  return assignment


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
    self.last_pair_possible = self.last_pair_candidates = 0
    self.last_conflicts = self.last_direct_carries = self.last_multi_count = 0
    # Shadow diagnostic switch. Off leaves the scan path byte-for-byte as it
    # was; on only fills last_decisions. Neither setting gates an assignment.
    self.trace_decisions = False
    self.last_decisions: tuple[BoschAssociationDecision, ...] = ()

  @property
  def last_diagnostics(self):
    # Research/debug consumers can request the old representation. Production
    # needs neither per-pair dictionaries nor rejection strings each scan.
    c = self.config
    result = []
    for key, evidence in sorted(self.pairs.items()):
      samples = evidence.samples
      if samples[-1][0] != self.now_ns:
        continue
      stable = (abs(samples[-1][1])-min(abs(s[1]) for s in samples) <= c.max_relative_distance_growth_m and
                abs(samples[-1][2])-min(abs(s[2]) for s in samples) <= c.max_relative_lateral_growth_m)
      mature = (len(samples) >= c.min_pair_observations and
                (self.now_ns-samples[0][0])/1e9 + 1e-6 >= c.min_pair_span_s)
      reason = 'relative_motion_drift' if not stable else ('geometry' if mature else 'insufficient_temporal_evidence')
      result.append(dict(a=key[0], b=key[1], compatible=stable and mature, reason=reason))
    return result

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
    coast_ns = round(c.coast_s * 1e9)
    for pid in list(self.states):
      state = self.states[pid]
      if timestamp_ns - state.observation.timestamp_ns > coast_ns:
        del self.states[pid]
        self.stats['deleted'] += 1
      else:
        # Drop in place: rebuilding every surviving member map each scan was
        # pure churn, since almost no scan expires a member.
        last_seen = state.member_last_seen
        expired = [rid for rid, ns in last_seen.items() if timestamp_ns - ns > coast_ns]
        for rid in expired:
          del last_seen[rid]

    n = len(raw_tracks)
    ids = [r.raw_track_id for r in raw_tracks]
    distances = [r.d_rel for r in raw_tracks]
    lateral = [r.y_rel for r in raw_tracks]
    velocities = [r.v_rel for r in raw_tracks]
    raw_index = {rid: i for i, rid in enumerate(ids)}
    for pair in list(self.pairs):
      i, j = raw_index.get(pair[0]), raw_index.get(pair[1])
      # The old all-pairs loop cleared evidence immediately when two observed
      # members failed distance. Preserve this even outside the new window.
      if (timestamp_ns - self.pairs[pair].samples[-1][0] > c.evidence_window_s * 1e9 or
          (i is not None and j is not None and abs(distances[i]-distances[j]) > c.distance_diameter_m)):
        del self.pairs[pair]

    compatible = [0] * n
    pair_cost = [0.] * (n*n)
    distance_order = sorted(range(n), key=distances.__getitem__)
    ground_speeds = [abs(v+v_ego) for v in velocities] if math.isfinite(v_ego) else None
    pair_candidates = lateral_rejections = velocity_rejections = motion_rejections = 0
    for position, a in enumerate(distance_order):
      for following in range(position+1, n):
        b = distance_order[following]
        if distances[b]-distances[a] > c.distance_diameter_m:
          break
        pair_candidates += 1
        i, j = (a, b) if a < b else (b, a)
        key = (ids[i], ids[j])
        delta_d, delta_y = distances[i]-distances[j], lateral[i]-lateral[j]
        delta_v = abs(velocities[i]-velocities[j])
        if abs(delta_y) > c.lateral_diameter_m:
          lateral_rejections += 1
          self.pairs.pop(key, None)
          continue
        if delta_v > c.velocity_diameter_mps:
          velocity_rejections += 1
          self.pairs.pop(key, None)
          continue
        if (ground_speeds is not None and min(ground_speeds[i], ground_speeds[j]) <= c.stationary_speed_mps
            and max(ground_speeds[i], ground_speeds[j]) >= c.moving_speed_mps):
          motion_rejections += 1
          self.pairs.pop(key, None)
          continue
        evidence = self.pairs.get(key)
        if evidence is None:
          evidence = self.pairs[key] = _BoschPairEvidence()
        samples = evidence.samples
        if samples and timestamp_ns - samples[-1][0] > c.pair_max_gap_s * 1e9:
          samples.clear()
        # Only the magnitudes are ever read back, and abs() is idempotent, so
        # store them once here instead of re-taking them over the whole window.
        abs_delta_d, abs_delta_y = abs(delta_d), abs(delta_y)
        samples.append((timestamp_ns, abs_delta_d, abs_delta_y))
        while timestamp_ns - samples[0][0] > c.evidence_window_s * 1e9:
          samples.popleft()
        # Widening from a recent minimum, not old larger converging separation.
        stable = (abs_delta_d-min(s[1] for s in samples) <= c.max_relative_distance_growth_m and
                  abs_delta_y-min(s[2] for s in samples) <= c.max_relative_lateral_growth_m)
        mature = (len(samples) >= c.min_pair_observations and
                  (timestamp_ns-samples[0][0])/1e9 + 1e-6 >= c.min_pair_span_s)
        if stable and mature:
          compatible[i] |= 1 << j
          compatible[j] |= 1 << i
          cost = abs_delta_d/c.distance_diameter_m + abs_delta_y/c.lateral_diameter_m + delta_v/c.velocity_diameter_mps
          pair_cost[i*n+j] = pair_cost[j*n+i] = cost
    self.last_pair_possible = n*(n-1)//2
    self.last_pair_candidates = pair_candidates
    self.stats['pair_rejected_distance_diameter'] += self.last_pair_possible-pair_candidates
    self.stats['pair_rejected_lateral_diameter'] += lateral_rejections
    self.stats['pair_rejected_velocity_diameter'] += velocity_rejections
    self.stats['pair_rejected_stationary_moving_conflict'] += motion_rejections

    owner = {rid: pid for pid, state in self.states.items() for rid in state.member_last_seen}
    previous = sorted(self.states)
    owner_bits = {pid: 1 << i for i, pid in enumerate(previous)}
    clusters = [[i] for i in range(n)]
    cluster_masks = [1 << i for i in range(n)]
    cluster_compatible = [compatible[i] | cluster_masks[i] for i in range(n)]
    cluster_owners = [owner_bits.get(owner.get(rid), 0) for rid in ids]
    member_cluster = list(range(n))
    max_members = c.max_members
    while True:
      best = None
      for i, a in enumerate(clusters):
        reach = cluster_compatible[i] & ~cluster_masks[i]
        if not reach:
          continue  # No compatible neighbor: singleton/group cannot merge.
        # Every cluster that could merge with i has all of its members here, so
        # walking these bits reaches exactly the candidates the full index scan
        # did. The minimum below compares whole (owner, cost, i, j) tuples, so
        # the order they are visited in cannot change which pair wins.
        limit = max_members-len(a)
        seen = 0
        while reach:
          bit = reach & -reach
          reach ^= bit
          j = member_cluster[bit.bit_length()-1]
          mark = 1 << j
          if j <= i or seen & mark:
            continue
          seen |= mark
          b = clusters[j]
          if len(b) > limit:
            continue
          if cluster_compatible[i] & cluster_masks[j] != cluster_masks[j]:
            continue
          # Same complete-link cost and original cluster-index tie order.
          cost = max(pair_cost[x*n+y] for x in a for y in b)
          choice = (not (cluster_owners[i] & cluster_owners[j]), cost, i, j)
          if best is None or choice < best:
            best = choice
      if best is None:
        break
      _, _, i, j = best
      clusters[i] += clusters.pop(j)
      cluster_masks[i] |= cluster_masks.pop(j)
      cluster_compatible[i] &= cluster_compatible.pop(j)
      cluster_owners[i] |= cluster_owners.pop(j)
      for index in range(j, len(clusters)):
        for member in clusters[index]:
          member_cluster[member] = index
      for member in clusters[i]:
        member_cluster[member] = i

    assigned = {}
    carry_mode = None
    solver_scores = None
    self.last_conflicts = self.last_direct_carries = 0
    if previous and clusters:
      # In a conflict-free graph every positive overlap edge must be selected;
      # zero-score dummy assignments cannot affect an output identity. Keep
      # the original whole solver on any merge/split, including its tie order.
      claims = {}
      ambiguous = set()
      for i, mask in enumerate(cluster_owners):
        if not mask:
          continue
        if mask & (mask-1):
          ambiguous.add(i)
        remaining = mask
        while remaining:
          bit = remaining & -remaining
          if bit in claims:
            ambiguous.update((i, claims[bit]))
          else:
            claims[bit] = i
          remaining ^= bit
      self.last_conflicts = len(ambiguous)
      if not ambiguous:
        assigned = {i: previous[mask.bit_length()-1] for i, mask in enumerate(cluster_owners) if mask}
        self.last_direct_carries = len(assigned)
        carry_mode = 'direct_carry'
      else:
        carry_mode = 'assignment_solver'
        # Invert membership once instead of intersecting every cluster against
        # every previous state; only the states a cluster actually touches can
        # score, and the score itself is unchanged.
        holders = {}
        for j, pid in enumerate(previous):
          for rid in self.states[pid].member_last_seen:
            entry = holders.get(rid)
            if entry is None:
              holders[rid] = [j]
            else:
              entry.append(j)
        # Fewer than five in a hundred of the matrix cells the old solver walked
        # could ever score. Collect the edges instead and let the decomposition
        # below reach the same matching without materialising the matrix.
        row_edges, column_edges = [], {}
        for i, cluster in enumerate(clusters):
          members = {ids[k] for k in cluster}
          overlaps = {}
          for rid in members:
            for j in holders.get(rid, ()):
              overlaps[j] = overlaps.get(j, 0) + 1
          edges = []
          for j, overlap in overlaps.items():
            pid = previous[j]
            observation = self.states[pid].observation
            score = (10*overlap +
                    3*(observation.representative_raw_track_id in members) +
                    min(observation.age_scans, 1000)*1e-5 + 1/(pid+1))
            edges.append((j, score))
            column = column_edges.get(j)
            if column is None:
              column_edges[j] = [(i, score)]
            else:
              column.append((i, score))
          row_edges.append(edges)
        # The shadow trace reports the whole score matrix, so a trace run keeps
        # the original path; production never needs the matrix at all.
        matched = None if self.trace_decisions else _bosch_physical_assignment(row_edges, column_edges)
        if matched is None:
          scores = np.zeros((len(clusters), len(previous)+len(clusters)))
          for i, edges in enumerate(row_edges):
            for j, score in edges:
              scores[i, j] = score
          ri, ci = bosch_linear_sum_assignment(-scores)
          assigned = {int(i): previous[int(j)] for i, j in zip(ri, ci) if j < len(previous) and scores[i, j] > 0}
          if self.trace_decisions:
            solver_scores = scores
        else:
          assigned = {i: previous[j] for i, j in matched.items()}

    result = []
    decisions = []
    current_ids = set(ids)
    # Whether a cue is usable at all depends on the cue alone. Test that once
    # per scan instead of once per return; the two tolerance comparisons that
    # actually decide support are untouched, and an unusable cue could never
    # reach them anyway.
    cues = [(cue.d_rel, cue.y_rel, cue.distance_tolerance_m, cue.lateral_tolerance_m) for cue in vision
            if math.isfinite(cue.d_rel) and math.isfinite(cue.y_rel) and cue.probability >= .7]
    if cues:
      vision_supported = []
      for track in raw_tracks:
        point = track.detection
        d_rel, y_rel = point.d_rel, point.y_rel
        vision_supported.append(any(abs(d_rel-cd) <= dt and abs(y_rel-cy) <= lt
                                    for cd, cy, dt, lt in cues))
    else:
      vision_supported = [False] * n
    # Nearly nine in ten clusters hold one return. Split that case out and hoist
    # the loop-invariant lookups; the emitted object is byte-for-byte the same.
    stats = self.stats
    states = self.states
    trace = self.trace_decisions
    v_ego_known = math.isfinite(v_ego)
    stationary_speed = c.stationary_speed_mps
    largest_group = stats['max_group_size']
    for i, cluster in enumerate(clusters):
      pid = assigned.get(i)
      old = states.get(pid)
      prior = old.observation if old else None
      if pid is None:
        if self.next_id >= 2**31:
          raise OverflowError('physical Int32 ID space exhausted; no reuse')
        pid, self.next_id = self.next_id, self.next_id+1
        stats['created'] += 1
      size = len(cluster)
      # Build the member sequence as the tuple the emitted object keeps, so the
      # object below stores it without a second copy.
      if size == 1:
        candidates = (raw_tracks[cluster[0]],)
        member_ids = {candidates[0].raw_track_id}
      else:
        candidates = tuple(raw_tracks[k] for k in sorted(cluster))
        member_ids = {m.raw_track_id for m in candidates}
      predicted = None
      # A single-member cluster picks its only member, so production skips the
      # projection there. The trace still needs it: that is exactly the case in
      # which no continuity term reaches the published state at all.
      if prior and (size > 1 or trace):
        dt = (timestamp_ns-prior.timestamp_ns)/1e9
        angle = -(yaw_rate or 0.)*dt
        dx = prior.d_rel + prior.v_rel*dt
        ca, sa = math.cos(angle), math.sin(angle)
        px = dx*ca-prior.y_rel*sa
        py = dx*sa+prior.y_rel*ca
        predicted = (dt, px, py)
      elif not prior and size > 1:
        # Same order statistic np.median returns, without entering NumPy for a
        # handful of floats: the even case averages the two middle values in
        # float64 exactly as np.mean of that pair does.
        ordered = sorted(m.d_rel for m in candidates)
        middle = size//2
        median = ordered[middle] if size % 2 else (ordered[middle-1]+ordered[middle])/2
      if size == 1:
        rep = candidates[0]
        oem_selected = rep.detection.slot == oem_slot
        supported = vision_supported[cluster[0]]
        evidence = 'single_return'
      else:
        def representative_cost(m):
          if prior:
            continuity = abs(m.d_rel-px) + .5*abs(m.y_rel-py) + .5*abs(m.v_rel-prior.v_rel)
            # Observed state wins over OEM changes; these are only ties.
            return (continuity, m.raw_track_id != prior.representative_raw_track_id,
                not vision_supported[raw_index[m.raw_track_id]], m.slot != oem_slot, -m.age_scans, m.raw_track_id)
          # Initially prefer a robust actual member near the group median.
          return (abs(m.d_rel-median), False, not vision_supported[raw_index[m.raw_track_id]], m.slot != oem_slot, -m.age_scans, m.raw_track_id)
        rep = min(candidates, key=representative_cost)
        oem_selected = any(m.slot == oem_slot for m in candidates)
        supported = any(vision_supported[k] for k in cluster)
        evidence = 'temporal_complete_link'
      point = rep.detection
      v_rel = point.v_rel
      obj = BoschPhysicalObject(pid, timestamp_ns, candidates, rep.raw_track_id,
                point.d_rel, point.y_rel, v_rel, oem_selected, supported,
                (prior.age_scans+1 if prior else 1), evidence)
      if trace:
        decisions.append(self._decision(obj, old, prior, carry_mode, predicted,
                                        solver_scores, i, previous))
      # A raw member currently assigned elsewhere cannot belong to this ID.
      # Dropping those in place leaves the surviving entries in the order the
      # rebuilt map had, and this state is the one states[pid] already holds.
      if old is None:
        last_seen = {}
        states[pid] = _BoschPhysicalState(obj, last_seen)
      else:
        last_seen = old.member_last_seen
        for rid in [rid for rid in last_seen if rid in current_ids and rid not in member_ids]:
          del last_seen[rid]
        old.observation = obj
      for m in candidates:
        last_seen[m.raw_track_id] = timestamp_ns
      if prior and prior.representative_raw_track_id != rep.raw_track_id:
        stats['representative_changes'] += 1
      if size == 1:
        # A lone member has at most one previous owner, so it can never record
        # a membership merge.
        recovered_all = old is not None and rep.recovered
      else:
        old_owners = set()
        recovered_all = old is not None
        for m in candidates:
          holder = owner.get(m.raw_track_id)
          if holder is not None:
            old_owners.add(holder)
          if not m.recovered:
            recovered_all = False
        if len(old_owners) > 1:
          stats['membership_merge_events'] += len(old_owners)-1
        stats['multi_member_groups'] += 1
      if recovered_all:
        stats['coasting_recoveries'] += 1
      if supported:
        stats['vision_supported_groups'] += 1
      if v_ego_known and abs(v_rel+v_ego) <= stationary_speed:
        stats['stationary_groups'] += 1
      if size > largest_group:
        largest_group = size
      result.append(obj)
    if largest_group != stats['max_group_size']:
      stats['max_group_size'] = largest_group

    # Retire absorbed physical IDs immediately, retaining only missing members
    # for genuine coasting. Prevent two IDs from owning the same raw member.
    live_owners = {m.raw_track_id: obj.physical_track_id for obj in result for m in obj.members}
    output_ids = {obj.physical_track_id for obj in result}
    for pid in list(states):
      state = states[pid]
      last_seen = state.member_last_seen
      stolen = [rid for rid in last_seen if live_owners.get(rid, pid) != pid]
      for rid in stolen:
        del last_seen[rid]
      if not last_seen and pid not in output_ids:
        del states[pid]
        stats['absorbed'] += 1
    stats['scans'] += 1
    stats['output_objects'] += len(result)
    multi = 0
    for obj in result:
      if len(obj.members) > 1:
        multi += 1
    self.last_multi_count = multi
    self.last_decisions = tuple(decisions)
    return tuple(sorted(result, key=lambda obj: obj.physical_track_id))

  def _decision(self, obj, old, prior, carry_mode, predicted, solver_scores, cluster_index, previous):
    """Build one shadow record. Called only while trace_decisions is set."""
    member_ids = {m.raw_track_id for m in obj.members}
    best = second = None
    if solver_scores is not None:
      ranked = sorted((float(v) for v in solver_scores[cluster_index][:len(previous)] if v > 0), reverse=True)
      best = ranked[0] if ranked else None
      second = ranked[1] if len(ranked) > 1 else None
    dt_s = px = py = residual_d = residual_y = residual_v = None
    if predicted is not None:
      dt_s, px, py = predicted
      residual_d, residual_y = obj.d_rel-px, obj.y_rel-py
      residual_v = obj.v_rel-prior.v_rel
    elif prior is not None:
      dt_s = (obj.timestamp_ns-prior.timestamp_ns)/1e9
    return BoschAssociationDecision(
      timestamp_ns=obj.timestamp_ns,
      physical_track_id=obj.physical_track_id,
      previous_physical_track_id=prior.physical_track_id if prior else None,
      assignment_mode='created' if prior is None else carry_mode,
      dt_s=dt_s if dt_s is not None else 0.,
      member_raw_track_ids=tuple(m.raw_track_id for m in obj.members),
      member_slots=obj.member_slots,
      previous_member_raw_track_ids=tuple(m.raw_track_id for m in prior.members) if prior else (),
      member_overlap=len(member_ids.intersection(old.member_last_seen)) if old else 0,
      observed_member_overlap=len(member_ids.intersection(m.raw_track_id for m in prior.members)) if prior else 0,
      representative_raw_track_id=obj.representative_raw_track_id,
      previous_representative_raw_track_id=prior.representative_raw_track_id if prior else None,
      representative_changed=bool(prior and prior.representative_raw_track_id != obj.representative_raw_track_id),
      representative_still_a_member=bool(prior and prior.representative_raw_track_id in member_ids),
      d_rel=obj.d_rel, y_rel=obj.y_rel, v_rel=obj.v_rel,
      predicted_d_rel=px, predicted_y_rel=py,
      predicted_v_rel=prior.v_rel if prior else None,
      residual_d_m=residual_d, residual_y_m=residual_y, residual_v_mps=residual_v,
      best_score=best, second_score=second,
      age_scans=obj.age_scans, grouping_evidence=obj.grouping_evidence)


class BoschPhysicalTracker:
  def __init__(self, raw_config: BoschRawTrackingConfig | None = None, group_config: BoschGroupingConfig | None = None):
    self.raw_manager = BoschRawTrackManager(raw_config)
    self.group_manager = BoschObjectGroupManager(group_config)

  def set_decision_trace(self, enabled: bool) -> None:
    """Turn the shadow association traces on both layers on or off.

    Diagnostics only: neither layer reads its trace back, so this never changes
    a raw match, a physical ID, a representative or a published coordinate.
    """
    self.raw_manager.trace_decisions = bool(enabled)
    self.group_manager.trace_decisions = bool(enabled)

  def update(self, timestamp_ns, detections, *, yaw_rate=None, v_ego=math.nan, oem_slot=None, vision=()):
    start_ns = time.perf_counter_ns()
    raw = self.raw_manager.update(timestamp_ns, detections, yaw_rate=yaw_rate)
    raw_done_ns = time.perf_counter_ns()
    result = self.group_manager.update(timestamp_ns, raw, yaw_rate=yaw_rate, v_ego=v_ego,
                                      oem_slot=oem_slot, vision=vision)
    self.raw_elapsed_ns = raw_done_ns - start_ns
    self.physical_elapsed_ns = time.perf_counter_ns() - raw_done_ns
    return result


class _BoschStaticOffPathFilter:
  """Bosch-only publication subset; raw/physical tracking remains untouched."""
  _EVIDENCE_NS = 800_000_000
  _GAP_NS = 160_000_000
  _ENTER_SPEED_MPS = 0.8
  _EXIT_SPEED_MPS = 1.0
  _ENTER_OFFSET_M = 5.5
  _EXIT_OFFSET_M = 5.0
  _LONGITUDINAL_RESIDUAL_M = 2.5
  _LATERAL_STEP_M = 0.375
  _INWARD_STEP_M = 0.35
  _MAX_LATERAL_EXCURSION_M = 2.0
  _MIN_EGO_SPEED_MPS = 3.0

  def __init__(self):
    from openpilot.selfdrive.carrot.radar_motion.predictor import model_path_y
    self._path_y = model_path_y
    self._checked_path = None
    self._checked_valid = False
    # physical PID가 현재 scan에서 사라지면 아래 update의 새 dict에 복사되지
    # 않는다. 따라서 state는 Bosch의 현재 살아 있는 physical object 수로 제한된다.
    self._states = {}
    self.state_peak = 0

  def update(self, objects, timestamp_ns, v_ego, path=(), yaw_rate=None):
    # Unknown ego speed remains the only whole-scan fail-open: without it the
    # world residual cannot be formed, so no static state can be established.
    if not math.isfinite(v_ego):
      self._states = {}
      return objects
    # Provider supplies only causal, fresh paths. An unusable path no longer
    # retains the whole scan; those objects fall back to a straight corridor.
    # The same model path is handed to several scans, so validate it once and
    # keep a reference alongside the verdict rather than rescanning every time.
    if path is self._checked_path:
      path_valid = self._checked_valid
    else:
      path_valid = (len(path) >= 2
                    and all(math.isfinite(x) and math.isfinite(y) for x, y in path)
                    and all(path[i][0] < path[i + 1][0] for i in range(len(path) - 1)))
      self._checked_path = path
      self._checked_valid = path_valid
    # In vehicle coordinates a world-static return moves at -vEgo + yaw*yRel,
    # so the rotational term must be removed before judging ground speed. A
    # missing or non-finite yaw degrades to the previous naive residual, which
    # over-keeps rather than dropping a real object.
    rotation = yaw_rate if yaw_rate is not None and math.isfinite(yaw_rate) else 0.0
    kept = []
    states = {}
    for obj in objects:
      # Representative velocity, not member votes.
      speed = abs(obj.v_rel + v_ego - rotation * obj.y_rel)
      # OEM/model support is an immediate fail-open veto. Multi-return physical
      # objects are excluded from temporal escalation, but still pass through
      # the pre-existing static/off-path qualification below.
      if obj.oem_selected or obj.vision_supported or not math.isfinite(speed):
        kept.append(obj)
        continue
      offset = obj.y_rel
      if path_valid and path[0][0] <= obj.d_rel <= path[-1][0]:
        # Only the part of the path that still has road behind it may overrule
        # the straight corridor. Past that the path is read only when the
        # corridor would already have dropped the object, so a terminal
        # prediction can argue to keep a return but never to remove one.
        if obj.d_rel <= path[-1][0] - BOSCH_PATH_TRUST_MARGIN_M:
          offset = obj.y_rel - self._path_y(path, obj.d_rel)
        elif abs(offset) > 3.0:
          path_offset = obj.y_rel - self._path_y(path, obj.d_rel)
          if abs(path_offset) < abs(offset):
            offset = path_offset
      if not math.isfinite(offset):
        kept.append(obj)
        continue

      previous = self._states.get(obj.physical_track_id) if len(obj.members) == 1 else None
      established = bool(previous and previous[8])
      speed_limit = self._EXIT_SPEED_MPS if established else self._ENTER_SPEED_MPS
      offset_limit = self._EXIT_OFFSET_M if established else self._ENTER_OFFSET_M
      # Very-low-speed roadside scenes cannot reliably distinguish a parked
      # vehicle from structure using radar-only static evidence. Likewise, a
      # cumulative lateral excursion is direct evidence that a far-side object
      # may be a turning/crossing vehicle even when each individual step is
      # small. Both cases fail open for the temporal extension while preserving
      # the pre-existing <=0.6 m/s immediate static/off-path decision below.
      eligible = (len(obj.members) == 1 and v_ego >= self._MIN_EGO_SPEED_MPS
                  and speed <= speed_limit and abs(offset) >= offset_limit)
      if eligible and previous is not None:
        since_ns, last_ns, member_id, representative_id, d_rel, y_rel, v_rel, old_offset, was_established, anchor_y = previous
        dt_s = (timestamp_ns - last_ns) * 1e-9
        eligible = (
          0.0 < dt_s <= self._GAP_NS * 1e-9
          and member_id == obj.members[0].raw_track_id
          and representative_id == obj.representative_raw_track_id
          and abs(obj.d_rel - (d_rel + v_rel * dt_s)) <= max(self._LONGITUDINAL_RESIDUAL_M, 7.0 * dt_s)
          and abs(obj.v_rel - v_rel) <= 1.0
          and abs(obj.y_rel - y_rel) <= self._LATERAL_STEP_M
          and abs(old_offset) - abs(offset) <= self._INWARD_STEP_M
          and abs(obj.y_rel - anchor_y) <= self._MAX_LATERAL_EXCURSION_M
        )
      if eligible:
        if previous is None:
          since_ns = timestamp_ns
          established = False
          anchor_y = obj.y_rel
        elif not established:
          established = timestamp_ns - since_ns >= self._EVIDENCE_NS
        states[obj.physical_track_id] = (
          since_ns, timestamp_ns, obj.members[0].raw_track_id, obj.representative_raw_track_id,
          obj.d_rel, obj.y_rel, obj.v_rel, offset, established, anchor_y)

      # 기존 <=0.6 static/off-path 동작은 유지한다. 새 상태는 그 관측도
      # evidence로 사용하되, 0.6 초과 표적만 충분한 이력 뒤 추가 억제한다.
      baseline_kept = speed > 0.6 or abs(offset) <= 3.0
      if baseline_kept and not (eligible and established):
        kept.append(obj)
    self._states = states
    self.state_peak = max(self.state_peak, len(states))
    return objects if len(kept) == len(objects) else tuple(kept)


@dataclass
class _BoschP91PairState:
  parent_pid: int
  since_ns: int
  last_ns: int
  samples: int
  candidate_raw_id: int
  parent_representative_raw_id: int
  d_offset_mean: float
  d_offset_m2: float
  y_offset_mean: float
  y_offset_m2: float
  dv_sq_sum: float
  anchor_candidate_y: float
  anchor_y_offset: float
  last_candidate_y: float
  last_d_offset: float
  last_y_offset: float


class _BoschPersistentSpatialCloneFilter:
  """Bosch-only persistent unsupported spatial-copy shadow detector.

  This is not an 0x601 authenticity filter. Supported objects form a small
  parent pool; support on the candidate is instead an immediate fail-open.
  One fixed-size state is retained per live candidate PID, never a full pair
  history. The Bosch research branch keeps SHADOW diagnostics while ACTIVE
  removes only a mature candidate from the final publication view.
  """
  _EVIDENCE_NS = 3_000_000_000
  # Bosch singleton returns may be absent for one 10 Hz scan; measured P91
  # component gaps peak just below 300 ms. Keep fixed state through that one
  # missing observation, while never suppressing an absent object.
  _GAP_NS = 320_000_000
  _MIN_SAMPLES = 25
  _MIN_EGO_SPEED_MPS = 3.0
  _MIN_WORLD_SPEED_MPS = 4.0
  _MAX_DV_MPS = .75
  _MAX_V_RMSE_MPS = .35
  _MIN_D_OFFSET_M = 6.0
  _MAX_D_OFFSET_M = 30.0
  _MIN_Y_SEPARATION_M = 3.5
  _MAX_D_OFFSET_STD_M = .75
  _MAX_Y_OFFSET_STD_M = .80
  _MAX_D_OFFSET_STEP_M = 1.5
  _MAX_Y_OFFSET_STEP_M = .75
  _MAX_CANDIDATE_Y_STEP_M = .75
  _MAX_LATERAL_EXCURSION_M = 2.5

  def __init__(self, mode=BOSCH_P91_MODE):
    if mode not in (BOSCH_P91_OFF, BOSCH_P91_SHADOW, BOSCH_P91_ACTIVE):
      raise ValueError('invalid Bosch Candidate P91 mode')
    self.mode = mode
    self._states: dict[int, _BoschP91PairState] = {}
    self._support_until: dict[int, int] = {}
    self.last_ns = None
    self.would_suppress = frozenset()
    self.decisions = {}
    self.parent_pool_last = self.parent_pool_peak = 0
    self.pair_evaluations_last = self.pair_evaluations_total = self.pair_evaluations_peak = 0
    self.state_peak = 0
    self.publication_suppressed = 0

  @staticmethod
  def _std(mean, m2, samples):
    return math.sqrt(max(0., m2 / samples)) if samples else math.inf

  @staticmethod
  def _welford(mean, m2, samples, value):
    delta = value - mean
    mean += delta / samples
    return mean, m2 + delta * (value - mean)

  def update(self, objects, timestamp_ns, v_ego, *, word0_pids=(), yaw_rate=None):
    self.last_ns = timestamp_ns
    self.would_suppress = frozenset()
    self.decisions = {}
    self.pair_evaluations_last = 0
    if self.mode == BOSCH_P91_OFF:
      self._states = {}
      self._support_until = {}
      return self.would_suppress
    if not math.isfinite(v_ego) or v_ego < self._MIN_EGO_SPEED_MPS or not objects:
      self._states = {}
      self._support_until = {}
      return self.would_suppress

    live = {obj.physical_track_id for obj in objects}
    word0 = frozenset(word0_pids)
    support_until = {pid: expiry for pid, expiry in self._support_until.items()
                     if pid in live and expiry >= timestamp_ns}
    for obj in objects:
      if obj.vision_supported or obj.oem_selected or obj.physical_track_id in word0:
        support_until[obj.physical_track_id] = timestamp_ns + BOSCH_P91_SUPPORT_HOLD_NS
    self._support_until = support_until
    supported = {pid for pid, expiry in support_until.items() if expiry >= timestamp_ns}
    parents = tuple(obj for obj in objects if obj.physical_track_id in supported)
    self.parent_pool_last = len(parents)
    self.parent_pool_peak = max(self.parent_pool_peak, len(parents))
    if not parents:
      self._states = {}
      return self.would_suppress

    rotation = yaw_rate if yaw_rate is not None and math.isfinite(yaw_rate) else 0.
    previous_states = self._states
    states = {}
    suppress = set()
    for candidate in objects:
      pid = candidate.physical_track_id
      # Any recent independent support is an immediate KEEP/reset. Multi-member
      # objects stay inside the existing large-vehicle/member-aware path.
      if pid in supported or len(candidate.members) != 1:
        continue
      world_speed = candidate.v_rel + v_ego - rotation * candidate.y_rel
      if not math.isfinite(world_speed) or world_speed < self._MIN_WORLD_SPEED_MPS:
        continue

      previous = previous_states.get(pid)
      plausible = []
      for parent in parents:
        if parent.physical_track_id == pid:
          continue
        self.pair_evaluations_last += 1
        d_offset = candidate.d_rel - parent.d_rel
        y_offset = candidate.y_rel - parent.y_rel
        dv = candidate.v_rel - parent.v_rel
        if (self._MIN_D_OFFSET_M <= d_offset <= self._MAX_D_OFFSET_M and
            abs(y_offset) >= self._MIN_Y_SEPARATION_M and abs(dv) <= self._MAX_DV_MPS):
          plausible.append((abs(dv), -parent.age_scans, parent.physical_track_id,
                            parent, d_offset, y_offset, dv))
      if not plausible:
        continue
      if previous is not None:
        chosen = next((item for item in plausible if item[3].physical_track_id == previous.parent_pid), None)
      else:
        chosen = None
      _, _, _, parent, d_offset, y_offset, dv = chosen or min(plausible)
      member_id = candidate.members[0].raw_track_id
      continuous = bool(
        previous is not None and previous.parent_pid == parent.physical_track_id and
        0 < timestamp_ns - previous.last_ns <= self._GAP_NS and
        previous.candidate_raw_id == member_id and
        previous.parent_representative_raw_id == parent.representative_raw_track_id and
        abs(d_offset - previous.last_d_offset) <= self._MAX_D_OFFSET_STEP_M and
        abs(y_offset - previous.last_y_offset) <= self._MAX_Y_OFFSET_STEP_M and
        abs(candidate.y_rel - previous.last_candidate_y) <= self._MAX_CANDIDATE_Y_STEP_M and
        abs(candidate.y_rel - previous.anchor_candidate_y) <= self._MAX_LATERAL_EXCURSION_M and
        abs(y_offset - previous.anchor_y_offset) <= self._MAX_LATERAL_EXCURSION_M)
      if not continuous:
        state = _BoschP91PairState(parent.physical_track_id, timestamp_ns, timestamp_ns, 1,
                                   member_id, parent.representative_raw_track_id,
                                   d_offset, 0., y_offset, 0., dv * dv,
                                   candidate.y_rel, y_offset, candidate.y_rel, d_offset, y_offset)
      else:
        samples = previous.samples + 1
        d_mean, d_m2 = self._welford(previous.d_offset_mean, previous.d_offset_m2, samples, d_offset)
        y_mean, y_m2 = self._welford(previous.y_offset_mean, previous.y_offset_m2, samples, y_offset)
        state = _BoschP91PairState(parent.physical_track_id, previous.since_ns, timestamp_ns, samples,
                                   member_id, parent.representative_raw_track_id,
                                   d_mean, d_m2, y_mean, y_m2, previous.dv_sq_sum + dv * dv,
                                   previous.anchor_candidate_y, previous.anchor_y_offset,
                                   candidate.y_rel, d_offset, y_offset)
      states[pid] = state
      age_ns = timestamp_ns - state.since_ns
      v_rmse = math.sqrt(state.dv_sq_sum / state.samples)
      d_std = self._std(state.d_offset_mean, state.d_offset_m2, state.samples)
      y_std = self._std(state.y_offset_mean, state.y_offset_m2, state.samples)
      established = (age_ns >= self._EVIDENCE_NS and state.samples >= self._MIN_SAMPLES and
                     v_rmse <= self._MAX_V_RMSE_MPS and d_std <= self._MAX_D_OFFSET_STD_M and
                     y_std <= self._MAX_Y_OFFSET_STD_M)
      if established:
        suppress.add(pid)
      self.decisions[pid] = {
        'would_suppress': established, 'reason': 'persistent_spatial_copy' if established else 'warming',
        'parent_pid': parent.physical_track_id, 'candidate_pid': pid,
        'evidence_age_s': age_ns * 1e-9, 'samples': state.samples,
        'v_rmse_mps': v_rmse, 'd_offset_mean_m': state.d_offset_mean,
        'd_offset_std_m': d_std, 'y_offset_mean_m': state.y_offset_mean,
        'y_offset_std_m': y_std, 'world_speed_mps': world_speed,
      }
    parent_ids = {parent.physical_track_id for parent in parents}
    for pid, state in previous_states.items():
      if (pid not in live and state.parent_pid in parent_ids and
          timestamp_ns - state.last_ns <= self._GAP_NS):
        states[pid] = state
    self._states = states
    self.would_suppress = frozenset(suppress)
    self.pair_evaluations_total += self.pair_evaluations_last
    self.pair_evaluations_peak = max(self.pair_evaluations_peak, self.pair_evaluations_last)
    self.state_peak = max(self.state_peak, len(states))
    return self.would_suppress


@dataclass
class _BoschOemGateState:
  """Bounded per-candidate probation state. No pair matrix, no history list."""
  tentative_scans: int
  since_ns: int
  last_ns: int
  evidence_ns: int
  ever_validated: bool
  in_path_scans: int
  last_abs_y: float
  inward_scans: int
  inward_from_abs_y: float
  inward_since_ns: int
  withheld: bool


class _BoschOemIntentTracker:
  """Short bounded window over the stock SCC longitudinal request.

  SCC12 runs at 50 Hz while the object list runs at 10 Hz, so a single 20 ms
  sample is never used. Freshness and a sane decoded band are required before
  the value counts; otherwise the caller fails open.
  """

  def __init__(self):
    self._samples = deque()
    self.last_ns = 0

  def reset(self):
    self._samples.clear()
    self.last_ns = 0

  def ingest(self, timestamp_ns, value):
    if value is None or not math.isfinite(value):
      return
    if not BOSCH_OEM_INTENT_MIN_MPS2 <= value <= BOSCH_OEM_INTENT_MAX_MPS2:
      return
    if self._samples and timestamp_ns < self._samples[-1][0]:
      self._samples.clear()
    self._samples.append((timestamp_ns, value))
    self.last_ns = timestamp_ns
    horizon = timestamp_ns - BOSCH_OEM_INTENT_WINDOW_NS
    while self._samples and self._samples[0][0] < horizon:
      self._samples.popleft()

  def median(self, timestamp_ns):
    """Window median, or None when the stream is stale or too sparse."""
    if not self._samples or timestamp_ns - self.last_ns > BOSCH_OEM_INTENT_FRESH_NS:
      return None
    horizon = timestamp_ns - BOSCH_OEM_INTENT_WINDOW_NS
    values = sorted(value for ns, value in self._samples if ns >= horizon)
    if len(values) < BOSCH_OEM_INTENT_MIN_SAMPLES:
      return None
    return values[len(values) // 2]


class _BoschOemValidationGate:
  """Withhold publication of a sustained unvalidated in-path candidate.

  The 0x601 pair is read as a state rather than one support flag. Only the
  single candidate the word0 record points at can be withheld, and only while
  every independent real-object cue is absent and the stock SCC is also not
  asking for deceleration. The physical track, its members, its ID and its
  alias are untouched: this is a publication-view decision that reverses on the
  first scan any evidence returns.
  """

  def __init__(self, mode=BOSCH_OEM_GATE_MODE):
    if mode not in (BOSCH_OEM_GATE_OFF, BOSCH_OEM_GATE_SHADOW, BOSCH_OEM_GATE_ACTIVE):
      raise ValueError('invalid Bosch OEM validation gate mode')
    self.mode = mode
    self.neutral_mps2 = BOSCH_OEM_GATE_NEUTRAL_MPS2
    self.persist_scans = BOSCH_OEM_GATE_PERSIST_SCANS
    self.min_speed_mps = BOSCH_OEM_GATE_MIN_SPEED_MPS
    self.min_range_m = BOSCH_OEM_GATE_MIN_RANGE_M
    self.min_ttc_s = BOSCH_OEM_GATE_MIN_TTC_S
    self.cutin_mps = BOSCH_OEM_GATE_CUTIN_MPS
    self.cutin_scans = BOSCH_OEM_GATE_CUTIN_SCANS
    self.evidence_hold_ns = BOSCH_OEM_EVIDENCE_HOLD_NS
    self.in_path_m = BOSCH_OEM_GATE_IN_PATH_M
    self.settled_scans = BOSCH_OEM_GATE_SETTLED_SCANS
    self.sibling_d_m = BOSCH_OEM_GATE_SIBLING_D_M
    self.sibling_y_m = BOSCH_OEM_GATE_SIBLING_Y_M
    self.sibling_v_mps = BOSCH_OEM_GATE_SIBLING_V_MPS
    self.intent = _BoschOemIntentTracker()
    self._states: dict[int, _BoschOemGateState] = {}
    self.last_ns = None
    self.state = BOSCH_OEM_STATE_NONE
    self.would_withhold = frozenset()
    self.reasons = {}
    self.publication_withheld = 0
    self.state_peak = 0

  @staticmethod
  def classify(word0_active, word1_active):
    if word1_active:
      return BOSCH_OEM_STATE_VALIDATED if word0_active else BOSCH_OEM_STATE_SELECTED
    return BOSCH_OEM_STATE_TENTATIVE if word0_active else BOSCH_OEM_STATE_NONE

  def update(self, objects, timestamp_ns, v_ego, *, state, word0_pids=(), oem_valid=None):
    self.last_ns = timestamp_ns
    self.state = state
    self.would_withhold = frozenset()
    self.reasons = {}
    if self.mode == BOSCH_OEM_GATE_OFF:
      self._states = {}
      return self.would_withhold

    live = {obj.physical_track_id for obj in objects}
    previous = self._states
    states = {}
    # Any independent cue refreshes the evidence clock, including on objects the
    # gate will never consider. A candidate that was validated moments ago keeps
    # its grace through a short word1 dropout.
    validated_word0 = state == BOSCH_OEM_STATE_VALIDATED and oem_valid is not False
    word0 = frozenset(word0_pids)
    for obj in objects:
      pid = obj.physical_track_id
      prior = previous.get(pid)
      evidence_ns = prior.evidence_ns if prior is not None else 0
      ever = prior.ever_validated if prior is not None else False
      if (obj.vision_supported or obj.oem_selected or len(obj.members) > 1 or
          (validated_word0 and pid in word0)):
        evidence_ns = timestamp_ns
      # Once the OEM has confirmed an object it is never a gate candidate again
      # for the rest of that track's life. The route274 S19 return was never
      # confirmed in sixty seconds; a real lead is confirmed within one.
      if obj.oem_selected or (validated_word0 and pid in word0):
        ever = True
      # A real cut-in closes on the path scan after scan. A large-vehicle ghost
      # wanders in and back out, so only an unbroken inward run counts, never a
      # single noisy step of a 0.03125 m lateral grid seen at seventy metres.
      abs_y = abs(obj.y_rel)
      in_path = (prior.in_path_scans + 1 if prior is not None and abs_y <= self.in_path_m
                 else int(abs_y <= self.in_path_m))
      if prior is None or timestamp_ns - prior.last_ns > BOSCH_OEM_GATE_LATERAL_WINDOW_NS:
        inward_scans, inward_from, inward_since = 0, abs_y, timestamp_ns
      elif abs_y < prior.last_abs_y:
        inward_scans = prior.inward_scans + 1
        inward_from = prior.inward_from_abs_y if prior.inward_scans else prior.last_abs_y
        inward_since = prior.inward_since_ns if prior.inward_scans else prior.last_ns
      else:
        inward_scans, inward_from, inward_since = 0, abs_y, timestamp_ns
      states[pid] = _BoschOemGateState(
        prior.tentative_scans if prior is not None else 0,
        prior.since_ns if prior is not None else timestamp_ns,
        timestamp_ns, evidence_ns, ever, in_path, abs_y, inward_scans, inward_from,
        inward_since, prior.withheld if prior is not None else False)

    withhold = set()
    # Exactly one unambiguous word0 candidate is eligible. An ambiguous match
    # cannot name a target and stays fail-open, as elsewhere in this provider.
    eligible = word0 & live if state == BOSCH_OEM_STATE_TENTATIVE and len(word0) == 1 else frozenset()
    intent = self.intent.median(timestamp_ns)
    confirmed = tuple(obj for obj in objects if states[obj.physical_track_id].ever_validated)
    for obj in objects:
      pid = obj.physical_track_id
      entry = states[pid]
      if pid not in eligible:
        entry.tentative_scans = 0
        entry.since_ns = timestamp_ns
        entry.withheld = False
        continue
      entry.tentative_scans += 1
      if entry.tentative_scans == 1:
        entry.since_ns = timestamp_ns
      lateral_rate = ((entry.inward_from_abs_y - entry.last_abs_y) /
                      max(1e-3, (timestamp_ns - entry.inward_since_ns) * 1e-9)
                      if entry.inward_scans >= self.cutin_scans else 0.)
      closing = -obj.v_rel
      ttc = obj.d_rel / closing if closing > .1 else math.inf
      fail_open = None
      if entry.ever_validated:
        fail_open = 'was_validated'
      elif entry.in_path_scans > self.settled_scans:
        fail_open = 'settled_in_path'
      elif abs(obj.y_rel) > self.in_path_m:
        fail_open = 'not_in_path'
      elif any(other.physical_track_id != pid and
               abs(obj.d_rel - other.d_rel) <= self.sibling_d_m and
               abs(obj.y_rel - other.y_rel) <= self.sibling_y_m and
               abs(obj.v_rel - other.v_rel) <= self.sibling_v_mps for other in confirmed):
        fail_open = 'validated_sibling'
      elif oem_valid is True:
        fail_open = 'scc_valid'
      elif not math.isfinite(v_ego) or v_ego < self.min_speed_mps:
        fail_open = 'low_speed'
      elif obj.d_rel < self.min_range_m:
        fail_open = 'close_range'
      elif ttc < self.min_ttc_s:
        fail_open = 'short_ttc'
      elif timestamp_ns - entry.evidence_ns <= self.evidence_hold_ns and entry.evidence_ns:
        fail_open = 'recent_evidence'
      elif lateral_rate >= self.cutin_mps:
        fail_open = 'lateral_cutin'
      elif intent is None:
        fail_open = 'oem_intent_unavailable'
      elif intent <= self.neutral_mps2:
        fail_open = 'oem_decelerating'
      elif entry.tentative_scans < self.persist_scans:
        fail_open = 'warming'
      if fail_open is None:
        withhold.add(pid)
      entry.withheld = fail_open is None
      self.reasons[pid] = {
        'withhold': fail_open is None, 'reason': fail_open or 'tentative_oem_disagreement',
        'tentative_scans': entry.tentative_scans, 'in_path_scans': entry.in_path_scans,
        'd_rel': obj.d_rel, 'y_rel': obj.y_rel,
        'v_rel': obj.v_rel, 'ttc_s': ttc, 'lateral_rate_mps': lateral_rate,
        'oem_intent_mps2': intent, 'scc_obj_valid': oem_valid,
        'evidence_age_s': (timestamp_ns - entry.evidence_ns) * 1e-9 if entry.evidence_ns else None,
      }
    self._states = states
    self.state_peak = max(self.state_peak, len(states))
    self.would_withhold = frozenset(withhold)
    return self.would_withhold


BOSCH_PUBLICATION_ALIAS_START = 32
BOSCH_PUBLICATION_ALIAS_COUNT = 64
BOSCH_PUBLICATION_ALIAS_GRACE_S = 0.5


class BoschPublicationAliasAllocator:
  """Physical-ID bindings for the existing Hyundai front publication namespace.

  Tracking never reads these aliases. Unused aliases, then released aliases in
  FIFO order, are assigned independently of raw slots and representatives.
  """
  def __init__(self):
    self.physical_to_alias: dict[int, int] = {}
    self.alias_to_physical: dict[int, int] = {}
    self.last_published_ns: dict[int, int] = {}
    self.free_aliases = deque(range(BOSCH_PUBLICATION_ALIAS_START,
                                   BOSCH_PUBLICATION_ALIAS_START + BOSCH_PUBLICATION_ALIAS_COUNT))
    self.peak_usage = 0
    self.denial_count = 0
    self.grace_eviction_count = 0

  @property
  def current_usage(self):
    return len(self.physical_to_alias)

  def _release(self, physical_id):
    alias = self.physical_to_alias.pop(physical_id)
    del self.alias_to_physical[alias]
    del self.last_published_ns[physical_id]
    self.free_aliases.append(alias)

  def update(self, timestamp_ns, published_ids, live_ids) -> dict[int, int]:
    published = set(published_ids)
    # Bosch has at most 32 raw returns per scan, hence at most 32 physical
    # objects. Do not silently truncate or alias two objects if that invariant
    # is ever violated by a future producer using this 64-entry allocator.
    if len(published) > BOSCH_PUBLICATION_ALIAS_COUNT:
      self.denial_count += len(published) - BOSCH_PUBLICATION_ALIAS_COUNT
      raise ValueError('Bosch publication exceeds the front radar alias pool')
    grace_ns = int(BOSCH_PUBLICATION_ALIAS_GRACE_S * 1e9)
    for physical_id in list(self.physical_to_alias):
      if physical_id not in live_ids or timestamp_ns - self.last_published_ns[physical_id] > grace_ns:
        self._release(physical_id)

    needed = len(published.difference(self.physical_to_alias)) - len(self.free_aliases)
    if needed > 0:
      # Under pressure only unpublished grace bindings may be reclaimed;
      # every currently published object keeps its binding and is emitted.
      dormant = sorted(physical_id for physical_id in self.physical_to_alias if physical_id not in published)
      dormant.sort(key=self.last_published_ns.__getitem__)
      for physical_id in dormant[:needed]:
        self._release(physical_id)
        self.grace_eviction_count += 1

    for physical_id in sorted(published):
      if physical_id not in self.physical_to_alias:
        alias = self.free_aliases.popleft()
        self.physical_to_alias[physical_id] = alias
        self.alias_to_physical[alias] = physical_id
      self.last_published_ns[physical_id] = timestamp_ns
    self.peak_usage = max(self.peak_usage, self.current_usage)
    return {physical_id: self.physical_to_alias[physical_id] for physical_id in published}


def bosch_fill_point(point, obj, v_ego, alias=None):
  point.trackId = obj.physical_track_id if alias is None else alias[obj.physical_track_id]
  point.dRel, point.yRel, point.vRel = obj.d_rel, obj.y_rel, obj.v_rel
  point.aRel = point.yvRel = point.aLead = point.jLead = math.nan
  point.vLead = v_ego + obj.v_rel
  point.radarSource = 'frontRadar'
  point.trackState = 0
  point.measured = True


def bosch_append_points(radar, objects, v_ego, now_ns, alias=None):
  """Append directly to the final native list, retaining SCC aliasing safety."""
  if not objects:
    return
  previous = [point.to_dict() for point in radar.points]
  offset = len(previous)
  points = radar.init('points', offset + len(objects))
  for index, values in enumerate(previous):
    points[index] = values
  for index, obj in enumerate(objects, offset):
    point = points[index]
    bosch_fill_point(point, obj, v_ego, alias)
    members = obj.members
    if len(members) == 1:
      representative = members[0]
    else:
      representative = next(member for member in members
                            if member.raw_track_id == obj.representative_raw_track_id)
    # Read the native Float32 fields before projection, exactly as the original
    # temporary RadarData path did. This also preserves rounding for test inputs.
    age_s = (now_ns - representative.timestamp_ns) * 1e-9
    point.dRel += point.vRel * age_s


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
    bosch_fill_point(p, obj, v_ego)
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
  # One little-endian read covers both records: in that order the first word is
  # the low half of the payload, so no slice is copied to reach it.
  pair = int.from_bytes(payload, 'little')
  slot = (address - 0x602) * 2
  detections = []
  unsupported = False
  for half in range(2):
    word = (pair >> (half * 32)) & 0xFFFFFFFF
    if word == BOSCH_INACTIVE_WORD:
      continue
    if word & (1 << 31):
      unsupported = True
      continue
    detections.append(BoschRawDetection(timestamp_ns, slot + half,
                                   (word & 0x3FF) * .25,
                                   ((word >> 10) & 0x7FF) * .03125 - 32,
                                   ((word >> 21) & 0x3FF) * .25 - 128, word))
  return tuple(detections), unsupported


def _bosch_match_processed_target_word(objects, word):
  """Conservatively map 0x601 bytes0..3 decoded geometry to physical PIDs.

  The record is not assumed to be a raw-track copy or assigned a protocol
  name. Ambiguous near matches all count as fail-open support.
  """
  if word is None or word == BOSCH_INACTIVE_WORD or word & (1 << 31):
    return frozenset()
  d_rel = (word & 0x3FF) * .25
  y_rel = ((word >> 10) & 0x7FF) * .03125 - 32
  v_rel = ((word >> 21) & 0x3FF) * .25 - 128
  return frozenset(obj.physical_track_id for obj in objects
                   if abs(obj.d_rel - d_rel) <= 1.0 and abs(obj.y_rel - y_rel) <= .5 and
                   abs(obj.v_rel - v_rel) <= .5)


def bosch_make_points(objects, v_ego=math.nan):
  """Native physical IDs and original observations; unknown fields stay NaN."""
  if not objects:
    return []
  data = bosch_to_native_radar_data(objects, objects[0].timestamp_ns, v_ego=v_ego)
  return list(data.points)


class BoschRadarProvider:
  def __init__(self, bus: int, *, qualification=True, camera_bus=1,
               camera_extended_mode=BOSCH_CAMERA_EXTENDED_MODE, p91_mode=BOSCH_P91_MODE,
               oem_gate_mode=BOSCH_OEM_GATE_MODE, scc_bus=BOSCH_SCC_BUS):
    self.bus = bus
    self.camera_bus = camera_bus
    self.scc_bus = scc_bus
    self.tracker = BoschPhysicalTracker()
    self.camera_extended = BoschCameraExtendedGrouping(camera_extended_mode)
    self.publication_aliases = BoschPublicationAliasAllocator()
    self.qualifier = _BoschStaticOffPathFilter() if qualification else None
    self.p91 = _BoschPersistentSpatialCloneFilter(p91_mode)
    self.oem_gate = _BoschOemValidationGate(oem_gate_mode)
    self.scc_obj_valid = None
    self.scc_obj_valid_ns = 0
    self.can_error = False
    self.wrong_config = False
    self.last_scan_timestamp_ns = None
    self._debug_objects = ()
    self._debug_phase_ns = None
    self._debug_closed_ns = None
    self._debug_tick = None
    self._debug_complete = False
    self._debug_oem_word = None
    self._debug_oem_slot = None
    self._debug_oem_matches = 0
    self._debug_processed_word = None
    self._debug_processed_pids = frozenset()
    self._debug_word0_active = False
    self._debug_word1_active = False
    self._debug_oem_state = BOSCH_OEM_STATE_NONE
    self._debug_gate_suppress = frozenset()
    self._debug_gate_reasons = {}
    self._debug_oem_intent = None
    self._debug_p91_ns = 0
    self._debug_timeout = False
    self._frames = []
    self._anchors = []
    self._order = 0
    self._start_ns = None
    self._last_now_ns = None
    self._last_closed_anchor_ns = None
    self._last_output_ns = None
    self._pending_error = False
    self._perf_raw = self._perf_qualified = 0
    self.test_publications = self.test_suppressed_points = self.test_active_groups = 0
    self.test_last_suppressed = ()
    self.test_last_active_groups = 0
    self._reset_perf()

  def publication_view(self, objects, timestamp_ns=None):
    # Candidate P91 ACTIVE is the Bosch research-branch production path. The
    # independent camera-extended ACTIVE_TEST path below remains experimental.
    """Apply Bosch-only final-publication filters after alias allocation.

    P91 ACTIVE is enabled for this Bosch research branch. Camera-extended
    ACTIVE_TEST remains supervised-test-only and independently gated.
    """
    p91_suppressed = (self.p91.would_suppress if self.p91.mode == BOSCH_P91_ACTIVE and
                      objects and all(obj.timestamp_ns == self.p91.last_ns for obj in objects) else frozenset())
    if p91_suppressed:
      self.p91.publication_suppressed += len(p91_suppressed)
      objects = tuple(obj for obj in objects if obj.physical_track_id not in p91_suppressed)
    gate = self.oem_gate
    withheld = (gate.would_withhold if gate.mode == BOSCH_OEM_GATE_ACTIVE and
                objects and all(obj.timestamp_ns == gate.last_ns for obj in objects) else frozenset())
    if withheld:
      gate.publication_withheld += len(withheld)
      objects = tuple(obj for obj in objects if obj.physical_track_id not in withheld)
    ext = self.camera_extended
    if ext.mode != BOSCH_CAMERA_EXTENDED_ACTIVE_TEST:
      return objects
    if timestamp_ns is not None:
      self.test_publications += 1
    if not ext.mature_groups or not objects:
      self.test_last_suppressed = ()
      self.test_last_active_groups = 0
      return objects
    # 다른 scan의 tuple 또는 qualification에서 대표가 빠진 그룹은 baseline으로 연다.
    by_pid = {obj.physical_track_id: obj for obj in objects}
    suppressed = set()
    active = []
    for rep in ext.representatives:
      members = tuple(sorted(rep.members))
      if members not in ext.mature_groups or not all(
          p in by_pid and by_pid[p].timestamp_ns == ext.last_ns for p in members):
        continue
      suppressed.update(p for p in members if p != rep.representative_pid)
      active.append(rep)
    self.test_last_suppressed = tuple(sorted(suppressed))
    self.test_last_active_groups = len(active)
    if timestamp_ns is not None:
      self.test_suppressed_points += len(suppressed)
      self.test_active_groups += len(active)
      # 기존 carlog/logMessage 경로. 활성 publication만 기록하며 payload는 현재 그룹으로 제한한다.
      # 각 PID/alias/좌표와 ns를 CAN, liveTracks, radarState와 결합해 재등장 이력을 offline 복원한다.
      for rep in active:
        members = tuple(sorted(rep.members))
        state = ext.histories[members]
        snapshot = ext.camera.snapshot(ext.last_ns)
        camera_id = next((c.obj_id for c in snapshot[0][:snapshot[1]] if c.episode == state.cam_key), -1) if snapshot else -1
        detail = ';'.join(f'{p}:{self.publication_aliases.physical_to_alias.get(p, -1)}:'
                          f'{by_pid[p].d_rel}:{by_pid[p].y_rel}:{by_pid[p].v_rel}' for p in members)
        carlog.info(f'BoschActiveTest mode=ACTIVE_TEST ns={timestamp_ns} scan_ns={ext.last_ns} '
                    f'maturity={state.stable_intervals} rep_pid={rep.representative_pid} '
                    f'suppressed_pids={",".join(str(p) for p in members if p != rep.representative_pid)} '
                    f'suppressed_count={len(members) - 1} camera_id={camera_id} episode={state.cam_key} '
                    f'camera_ns={ext.last_camera_ns} members_pid_alias_d_y_v={detail}')
    return tuple(obj for obj in objects if obj.physical_track_id not in suppressed) if suppressed else objects

  def _reset_perf(self):
    self._perf_scans = 0
    self._perf_raw_sum = self._perf_raw_max = 0
    self._perf_physical_sum = self._perf_physical_max = 0
    self._perf_qualify_sum = self._perf_qualify_max = 0
    self._perf_gate_sum = self._perf_gate_max = 0
    self._perf_total_sum = self._perf_total_max = 0
    self._perf_native_count = self._perf_native_sum = self._perf_native_max = 0
    self._perf_raw_pairs = self._perf_raw_possible = 0
    self._perf_physical_pairs = self._perf_physical_possible = 0
    self._perf_components = self._perf_largest_component = self._perf_fallbacks = self._perf_conflicts = 0
    self._perf_ties = 0
    self._perf_camera_decode_count = self._perf_camera_decode_sum = self._perf_camera_decode_max = 0
    self._perf_p91_sum = self._perf_p91_max = 0

  def record_native_time(self, elapsed_ns):
    self._perf_native_count += 1
    self._perf_native_sum += elapsed_ns
    self._perf_native_max = max(self._perf_native_max, elapsed_ns)

  def perf_message(self):
    """Format/reset only at the 1 Hz logging boundary, never once per scan.

    total is decode + tracking + qualification per completed scan. Native list
    append is measured separately at its publication cadence. This excludes
    other card work, log I/O and modeld; it is not a whole-device CPU estimate.
    Pair/component/conflict values are interval totals, object counts are last.
    """
    scale = 1e-6 / max(self._perf_scans, 1)
    raw = self.tracker.raw_manager
    physical = self.tracker.group_manager
    backend = 'numpy' if bosch_linear_sum_assignment is bosch_numpy_linear_sum_assignment else 'scipy'
    extended_fields = self.camera_extended.perf_fields() if self.camera_extended.mode != BOSCH_CAMERA_EXTENDED_OFF else ''
    message = (
      f'BoschPerf solver={backend} scans={self._perf_scans} raw={self._perf_raw} raw_active={raw.active_count} '
      f'physical={len(self._debug_objects)} qualified={self._perf_qualified} '
      f'suppressed={len(self._debug_objects) - self._perf_qualified} '
      f'multi={physical.last_multi_count} '
      f'pairs_raw={self._perf_raw_pairs}/{self._perf_raw_possible} '
      f'pairs_physical={self._perf_physical_pairs}/{self._perf_physical_possible} '
      f'raw_ms_avg={self._perf_raw_sum * scale:.3f} raw_ms_max={self._perf_raw_max * 1e-6:.3f} '
      f'physical_ms_avg={self._perf_physical_sum * scale:.3f} physical_ms_max={self._perf_physical_max * 1e-6:.3f} '
      f'qualify_ms_avg={self._perf_qualify_sum * scale:.3f} qualify_ms_max={self._perf_qualify_max * 1e-6:.3f} '
      f'oemgate={self.oem_gate.mode} oemstate={self._debug_oem_state} '
      f'oemgate_withheld={self.oem_gate.publication_withheld} oemgate_state={self.oem_gate.state_peak} '
      f'oemgate_ms_avg={self._perf_gate_sum * scale:.3f} oemgate_ms_max={self._perf_gate_max * 1e-6:.3f} '
      f'total_ms_avg={self._perf_total_sum * scale:.3f} total_ms_max={self._perf_total_max * 1e-6:.3f} '
      f'native_ms_avg={self._perf_native_sum * 1e-6 / max(self._perf_native_count, 1):.3f} '
      f'native_ms_max={self._perf_native_max * 1e-6:.3f} '
      f'raw_components={self._perf_components} largest_raw_component={self._perf_largest_component} '
      f'raw_fallbacks={self._perf_fallbacks} raw_ties={self._perf_ties} '
      f'physical_conflicts={self._perf_conflicts} '
      f'alias_usage={self.publication_aliases.current_usage}/{BOSCH_PUBLICATION_ALIAS_COUNT} '
      f'alias_peak={self.publication_aliases.peak_usage} alias_denial={self.publication_aliases.denial_count} '
      f'alias_grace_evictions={self.publication_aliases.grace_eviction_count} '
      f'can_error={int(self.can_error)} '
      f'camera_decode_ms_avg={self._perf_camera_decode_sum * 1e-6 / max(self._perf_camera_decode_count, 1):.3f} '
      f'camera_decode_ms_max={self._perf_camera_decode_max * 1e-6:.3f} {extended_fields}'
      f' camera_ext_mode={self.camera_extended.mode} camera_ext_test_publications={self.test_publications}'
      f' camera_ext_suppressed_points={self.test_suppressed_points} camera_ext_active_group_publications={self.test_active_groups}'
      f' camera_ext_active_groups={self.test_last_active_groups} camera_ext_suppressed_pid_count={len(self.test_last_suppressed)}'
      f' p91_mode={self.p91.mode} p91_parent_pool={self.p91.parent_pool_last}/{self.p91.parent_pool_peak}'
      f' p91_pairs={self.p91.pair_evaluations_last}/{self.p91.pair_evaluations_peak}'
      f' p91_states={len(self.p91._states)}/{self.p91.state_peak}'
      f' p91_would_suppress={len(self.p91.would_suppress)} p91_publication_suppressed={self.p91.publication_suppressed}'
      f' p91_ms_avg={self._perf_p91_sum * scale:.3f} p91_ms_max={self._perf_p91_max * 1e-6:.3f}'
    )
    self._reset_perf()
    return message

  @property
  def slot_to_ids(self):
    return {member.slot: (member.raw_track_id, obj.physical_track_id)
            for obj in self._debug_objects for member in obj.members}

  @property
  def debug_snapshot(self):
    # The production caller logs at 1 Hz; no nested representation is built on
    # the 10 Hz scan path. Replay can still request the full mapping explicitly.
    if self._debug_timeout:
      return {'bus': self.bus, 'timeout': True, 'last_scan_timestamp_ns': self.last_scan_timestamp_ns,
              'objects': [], 'slot_to_ids': {}}
    if self._debug_phase_ns is None:
      return {}
    return {
      'bus': self.bus, 'phase_ns': self._debug_phase_ns, 'scan_timestamp_ns': self.last_scan_timestamp_ns,
      'closed_ns': self._debug_closed_ns, 'tick': self._debug_tick, 'complete': self._debug_complete,
      'can_error': self.can_error, 'wrong_config': self.wrong_config,
      'oem_word': self._debug_oem_word, 'oem_selected_slot': self._debug_oem_slot,
      'oem_match_count': self._debug_oem_matches, 'slot_to_ids': self.slot_to_ids,
      'processed_target_word': self._debug_processed_word,
      'processed_target_pids': sorted(self._debug_processed_pids),
      'p91_mode': self.p91.mode, 'p91_would_suppress': sorted(self.p91.would_suppress),
      'p91_decisions': self.p91.decisions,
      'p91_elapsed_ns': self._debug_p91_ns,
      'oem_state': self._debug_oem_state, 'oem_word0_active': self._debug_word0_active,
      'oem_word1_active': self._debug_word1_active, 'scc_obj_valid': self._scc_validity(self.last_scan_timestamp_ns or 0),
      'oem_intent_mps2': self._debug_oem_intent, 'oem_gate_mode': self.oem_gate.mode,
      'oem_gate_withhold': sorted(self._debug_gate_suppress), 'oem_gate_reasons': self._debug_gate_reasons,
      'objects': [{'physicalTrackId': obj.physical_track_id,
                   'rawTrackIds': [member.raw_track_id for member in obj.members],
                   'slots': list(obj.member_slots),
                   'measurement_ns': [member.timestamp_ns for member in obj.members],
                   'representative_rawTrackId': obj.representative_raw_track_id,
                   'representative_slot': next(member.slot for member in obj.members
                                               if member.raw_track_id == obj.representative_raw_track_id),
                   'oem_selected': obj.oem_selected} for obj in self._debug_objects],
    }

  def _ingest_scc(self, timestamp_ns, address, payload):
    """Decode the two stock-SCC observations this provider consumes.

    Both are read as evidence about the OEM's own target decision. A short or
    malformed frame is dropped rather than reported: a missing observation must
    fail open, never fault the radar.
    """
    if len(payload) < 8:
      return
    value = int.from_bytes(bytes(payload), 'little')
    if address == BOSCH_SCC11_ADDR:
      self.scc_obj_valid = bool((value >> BOSCH_SCC11_OBJ_VALID_BIT) & 1)
      self.scc_obj_valid_ns = timestamp_ns
    else:
      self.oem_gate.intent.ingest(timestamp_ns, ((value >> BOSCH_SCC12_AREQ_RAW_BIT) & 0x7FF) * .01 - 10.23)

  def _scc_validity(self, timestamp_ns):
    """SCC11.ObjValid at this scan, or None when the stream is absent/stale."""
    if self.scc_obj_valid is None or timestamp_ns - self.scc_obj_valid_ns > BOSCH_SCC_STALE_NS:
      return None
    return self.scc_obj_valid

  def update(self, can_packets, now_ns: int, v_ego: float, yaw_rate_left=None, vision=(), *, path=(), path_ns=None, path_source_ns=None):
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
    # Roughly six thousand frames a second reach this loop and about two hundred
    # are ours, so the reject path stays two indexed comparisons: no unpacking,
    # no attribute lookups and no payload copy until a frame is actually kept.
    bus = self.bus
    camera = self.camera_extended.camera
    camera_bus = self.camera_bus
    scc_bus = self.scc_bus
    camera_start_ns = 0
    frames = self._frames
    anchors = self._anchors
    order = self._order
    for timestamp_ns, messages in can_packets:
      future = timestamp_ns > now_ns
      for message in messages:
        address = message[0]
        if (camera is not None and message[2] == camera_bus and
            BOSCH_CAMERA_HEADER <= address <= BOSCH_CAMERA_LAST_FAMILY):
          if not camera_start_ns:
            camera_start_ns = time.perf_counter_ns()
          if future:
            camera.fault_ns = timestamp_ns
          else:
            camera.ingest(timestamp_ns, address, bytes(message[1]))
          continue
        if message[2] != bus:
          # Stock SCC observation only. It never becomes a detection, an object
          # or an actuator command; it can only withhold an unvalidated one.
          if message[2] == scc_bus and address in BOSCH_SCC_ADDRESSES and not future:
            self._ingest_scc(timestamp_ns, address, message[1])
          continue
        if address < 0x601 or address > 0x612:
          continue
        if future:
          self._pending_error = True
          continue
        payload = bytes(message[1])
        if address == 0x612:
          if len(payload) != 8:
            self._pending_error = True
            continue
          tick = int.from_bytes(payload[1:4], 'little')
          if tick % 10 == 0:
            if self._last_closed_anchor_ns is not None and timestamp_ns <= self._last_closed_anchor_ns:
              continue
            if not any(ns == timestamp_ns for ns, _ in anchors):
              anchors.append((timestamp_ns, tick))
        else:
          frames.append(_BoschCanFrame(timestamp_ns, address, payload, order))
          order += 1
    self._order = order
    if camera_start_ns:
      elapsed = time.perf_counter_ns() - camera_start_ns
      self._perf_camera_decode_count += 1
      self._perf_camera_decode_sum += elapsed
      self._perf_camera_decode_max = max(self._perf_camera_decode_max, elapsed)
    if len(self._anchors) > 1:
      self._anchors.sort()
    output = None
    while self._anchors and self._anchors[0][0] + BOSCH_WINDOW_NS <= now_ns:
      phase_ns, tick = self._anchors[0]
      assigned, retained = [], []
      for frame in self._frames:
        closest = (phase_ns if len(self._anchors) == 1 else
                   min(self._anchors, key=lambda anchor: abs(anchor[0] - frame.timestamp_ns))[0])
        if closest == phase_ns and abs(phase_ns - frame.timestamp_ns) <= BOSCH_WINDOW_NS:
          assigned.append(frame)
        else:
          retained.append(frame)
      self._frames = retained
      self._anchors.pop(0)
      self._last_closed_anchor_ns = phase_ns
      # Context must be fresh at receipt and close to this particular scan, also
      # when a batch closes several old windows. No path extrapolation in time.
      # modelV2.logMonoTime is a publication time: the scene the path describes
      # was captured a model-pipeline latency earlier, 43.6 ms at the median on
      # route259. Judge freshness on that capture time when the caller supplies
      # it, so a path is used exactly when its scene precedes this scan and it
      # had already been received. Without the source time the previous
      # publication-time gate stands unchanged.
      if path_ns is None or not (0 <= now_ns - path_ns <= 200_000_000):
        scan_path = ()
      elif path_source_ns:
        scan_path = path if phase_ns - 200_000_000 <= path_source_ns <= phase_ns else ()
      else:
        scan_path = path if phase_ns - 200_000_000 <= path_ns <= phase_ns + BOSCH_WINDOW_NS else ()
      output = self._finish_scan(phase_ns, tick, assigned, v_ego, yaw_rate_left, vision, scan_path)
      self._last_output_ns = now_ns

    # Preserve the pre-anchor half-window. Pending future anchors may already
    # own older frames when update() receives a large batch.
    oldest = (self._anchors[0][0] if self._anchors else now_ns) - BOSCH_WINDOW_NS
    if self._frames:
      self._frames = [frame for frame in self._frames if frame.timestamp_ns >= oldest]
    since = self.last_scan_timestamp_ns if self.last_scan_timestamp_ns is not None else self._start_ns
    if now_ns - since >= BOSCH_STALE_NS and (output is not None or self._last_output_ns is None or now_ns - self._last_output_ns >= BOSCH_OUTPUT_INTERVAL_NS):
      self.can_error = True
      self._debug_objects = ()
      self._debug_timeout = True
      self.p91.update((), now_ns, v_ego, yaw_rate=yaw_rate_left)
      self.oem_gate.update((), now_ns, v_ego, state=BOSCH_OEM_STATE_NONE)
      self._debug_gate_suppress = frozenset()
      self._debug_gate_reasons = {}
      self._perf_raw = self._perf_qualified = 0
      self._last_output_ns = now_ns
      return ()
    return output

  def _finish_scan(self, phase_ns, tick, frames, v_ego, yaw_rate_left, vision, path=()):
    start_ns = time.perf_counter_ns()
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
        self._debug_objects = ()
        self._perf_raw = self._perf_qualified = 0
        return ()
      fresh = [detection for detection in detections if detection.timestamp_ns > previous_ns]
      malformed |= len(fresh) != len(detections)
      detections = fresh
    oem = bucket.get(0x601)
    oem_word = int.from_bytes(oem.payload[4:8], 'little') if oem else None
    processed_word = int.from_bytes(oem.payload[0:4], 'little') if oem else None
    matches = [detection.slot for detection in detections if oem_word == detection.raw_word]
    oem_slot = matches[0] if matches else None
    word1_active = oem_word is not None and oem_word != BOSCH_INACTIVE_WORD and not oem_word & (1 << 31)
    word0_active = (processed_word is not None and processed_word != BOSCH_INACTIVE_WORD and
                    not processed_word & (1 << 31))
    oem_state = _BoschOemValidationGate.classify(word0_active, word1_active)
    objects = self.tracker.update(availability_ns, detections, yaw_rate=yaw_rate_left,
                                  v_ego=v_ego, oem_slot=oem_slot, vision=vision)
    self.last_scan_timestamp_ns = availability_ns
    self.can_error = not complete or malformed or unsupported
    self.wrong_config = unsupported
    self._debug_objects = objects
    self._debug_phase_ns = phase_ns
    self._debug_closed_ns = self._last_now_ns
    self._debug_tick = tick
    self._debug_complete = complete
    self._debug_oem_word = oem_word
    self._debug_oem_slot = oem_slot
    self._debug_oem_matches = len(matches)
    self._debug_processed_word = processed_word
    self._debug_timeout = False
    self.camera_extended.update(availability_ns, objects, v_ego, yaw_rate_left)
    qualify_start_ns = time.perf_counter_ns()
    # 모든 모드에서 qualifier 이력과 alias 할당에는 동일한 physical 집합을 전달한다.
    # P91 ACTIVE veto는 allocator 이후 최종 publication에만 적용한다.
    qualified = (self.qualifier.update(objects, availability_ns, v_ego, path, yaw_rate=yaw_rate_left)
                 if self.qualifier is not None else objects)
    processed_pids = _bosch_match_processed_target_word(qualified, processed_word)
    self._debug_processed_pids = processed_pids
    scc_valid = self._scc_validity(availability_ns)
    self._debug_word0_active = word0_active
    self._debug_word1_active = word1_active
    self._debug_oem_state = oem_state
    gate_start_ns = time.perf_counter_ns()
    self.oem_gate.update(qualified, availability_ns, v_ego, state=oem_state,
                         word0_pids=processed_pids, oem_valid=scc_valid)
    gate_ns = time.perf_counter_ns() - gate_start_ns
    self._debug_gate_suppress = self.oem_gate.would_withhold
    self._debug_gate_reasons = self.oem_gate.reasons
    self._debug_oem_intent = self.oem_gate.intent.median(availability_ns)
    self._perf_gate_sum += gate_ns
    self._perf_gate_max = max(self._perf_gate_max, gate_ns)
    # word0 alone no longer counts as P91 support. Only a word0 record that the
    # OEM also validated in the same scan can clear a clone suspicion.
    p91_word0 = processed_pids if oem_state == BOSCH_OEM_STATE_VALIDATED and scc_valid is not False else frozenset()
    p91_start_ns = time.perf_counter_ns()
    self.p91.update(qualified, availability_ns, v_ego, word0_pids=p91_word0, yaw_rate=yaw_rate_left)
    p91_ns = time.perf_counter_ns() - p91_start_ns
    self._debug_p91_ns = p91_ns
    self._perf_p91_sum += p91_ns
    self._perf_p91_max = max(self._perf_p91_max, p91_ns)
    done_ns = time.perf_counter_ns()
    qualify_ns, total_ns = done_ns - qualify_start_ns, done_ns - start_ns
    raw, physical = self.tracker.raw_manager, self.tracker.group_manager
    self._perf_scans += 1
    self._perf_raw, self._perf_qualified = len(detections), len(qualified)
    self._perf_raw_sum += self.tracker.raw_elapsed_ns
    self._perf_raw_max = max(self._perf_raw_max, self.tracker.raw_elapsed_ns)
    self._perf_physical_sum += self.tracker.physical_elapsed_ns
    self._perf_physical_max = max(self._perf_physical_max, self.tracker.physical_elapsed_ns)
    self._perf_qualify_sum += qualify_ns
    self._perf_qualify_max = max(self._perf_qualify_max, qualify_ns)
    self._perf_total_sum += total_ns
    self._perf_total_max = max(self._perf_total_max, total_ns)
    self._perf_raw_pairs += raw.last_pair_candidates
    self._perf_raw_possible += raw.last_pair_possible
    self._perf_physical_pairs += physical.last_pair_candidates
    self._perf_physical_possible += physical.last_pair_possible
    self._perf_components += raw.last_component_count
    self._perf_largest_component = max(self._perf_largest_component, raw.last_largest_component)
    self._perf_fallbacks += int(raw.last_solver_fallback)
    self._perf_ties += raw.last_tied_components
    self._perf_conflicts += physical.last_conflicts
    return qualified

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
    self._bosch_path_ns = None
    self._bosch_path_source_ns = 0
    self._bosch_path = ()
    if self.radar_tracks and CP.extFlags & HyundaiExtFlags.BOSCH_RADAR:
      CAN = CanBus(CP)
      bus = CAN.ACAN if CP.extFlags & HyundaiExtFlags.BOSCH_RADAR_BUS1 else CAN.CAM
      # Same bus the stock SCC parser uses, so the gate reads the factory
      # module rather than openpilot's own SCC11/SCC12 transmissions.
      scc_bus = CAN.CAM if CP.flags & HyundaiFlags.CAMERA_SCC else CAN.ECAN
      self.bosch = BoschRadarProvider(bus, camera_bus=CAN.ACAN, scc_bus=scc_bus,
                                      camera_extended_mode=BOSCH_CAMERA_EXTENDED_MODE)
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
    path = ()
    source_ns = 0
    if model is not None and 0 <= now_ns - model_ns <= 200_000_000:
      if model.leadsV3:
        lead = model.leadsV3[0]
        if lead.x and lead.y:
          # Same model-to-radar coordinates as the existing primary matcher.
          cues = (BoschVisionCue(float(lead.x[0]) - 1.52, -float(lead.y[0]), float(lead.prob)),)
      if self._bosch_path_ns != model_ns:
        position = getattr(model, 'position', None)
        self._bosch_path = tuple(zip(position.x, position.y)) if position is not None else ()
        # The camera frame the model consumed, not the time it finished.
        self._bosch_path_source_ns = int(getattr(model, 'timestampEof', 0) or 0)
        self._bosch_path_ns = model_ns
      path = self._bosch_path
      source_ns = self._bosch_path_source_ns
    self._bosch_context = (int(now_ns), yaw, cues, path, model_ns, source_ns)

  def update_carrot(self, v_ego, a_ego, rcv_time, can_packets):
    # Keep the legacy SCC/corner MyTrack processing intact. Bosch objects are
    # deliberately outside self.pts: MyTrack synthesizes lateral velocity and
    # acceleration, which have not been decoded for this sensor.
    ret = super().update_carrot(v_ego, a_ego, rcv_time, can_packets)
    if ret is not None and self.bosch is not None:
      scan_ns = self.bosch.last_scan_timestamp_ns
      objects = (self._bosch_objects if scan_ns is not None and 0 <= self._bosch_now_ns - scan_ns <= BOSCH_SAMPLE_HOLD_NS
                 else ())
      start_ns = time.perf_counter_ns()
      alias = self.bosch.publication_aliases.update(
        self._bosch_now_ns, (obj.physical_track_id for obj in objects), self.bosch.tracker.group_manager.states)
      objects = self.bosch.publication_view(objects, self._bosch_now_ns)
      bosch_append_points(ret, objects, self.v_ego, self._bosch_now_ns, alias)
      self.bosch.record_native_time(time.perf_counter_ns() - start_ns)
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
      now_ns, yaw, cues, path, path_ns, path_source_ns = (
        self._bosch_context or (time.monotonic_ns(), None, (), (), None, 0))
      self._bosch_now_ns = now_ns
      self._bosch_context = None
      objects = self.bosch.update(can_strings, now_ns=now_ns, v_ego=self.v_ego,
                                  yaw_rate_left=yaw, vision=cues, path=path, path_ns=path_ns,
                                  path_source_ns=path_source_ns)
      if objects is not None:
        self._bosch_objects = objects
        track_ready = True
        if now_ns - self._bosch_debug_ns >= 1_000_000_000:
          carlog.info(self.bosch.perf_message())
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
