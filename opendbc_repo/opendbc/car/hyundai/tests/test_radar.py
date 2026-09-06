import math
from dataclasses import replace

import pytest

from opendbc.can import CANParser
from opendbc.car import Bus, structs
import opendbc.car.hyundai.hyundaicanfd as hyundaicanfd
import opendbc.car.hyundai.radar_interface as radar_interface_module
from opendbc.car.hyundai.radar_interface import (
  CORNER_OBJECT_STABLE_TRACK_ID_START,
  RADAR_MSG_COUNT,
  RADAR_MSG_COUNT3,
  RADAR_MSG_COUNT4,
  RADAR_REQUIRED_MSG_COUNT,
  RADAR_START_ADDR_CANFD3,
  BoschObjectGroupManager,
  BoschPhysicalTracker,
  BoschPublicationAliasAllocator,
  BoschRadarProvider,
  BoschRawDetection,
  BoschRawTrack,
  BoschRawTrackManager,
  BoschRawTrackingConfig,
  CornerObjectTrackIdManager,
  RadarInterface,
  bosch_append_points,
  bosch_fill_point,
  bosch_to_native_radar_data,
  canfd_group2_track_status,
  corner_object_position_valid,
  deduplicate_corner_candidates,
)
from opendbc.car.hyundai.values import CAR, HyundaiExtFlags, HyundaiFlags


class TestMandoRadar:
  @staticmethod
  def make_interface(monkeypatch):
    class FakeParams:
      def get_int(self, key):
        return 1 if key == "EnableRadarTracks" else 0

    monkeypatch.setattr(radar_interface_module, "Params", FakeParams)
    cp = structs.CarParams()
    cp.carFingerprint = CAR.HYUNDAI_GRANDEUR_IG
    cp.flags = 0
    cp.extFlags = 0
    cp.radarUnavailable = False
    cp.safetyConfigs = [structs.CarParams.SafetyConfig()]
    return RadarInterface(cp)

  def test_optional_upper_track_bank_and_32_slot_compatibility(self, monkeypatch):
    radar_interface = self.make_interface(monkeypatch)

    assert RADAR_MSG_COUNT == 64
    assert RADAR_REQUIRED_MSG_COUNT == 32
    assert radar_interface.radar_msg_count == 64
    assert radar_interface.radar_required_msg_count == 32
    assert radar_interface.trigger_msg_tracks == 0x51F
    assert not radar_interface.rcp_tracks.message_states[0x51F].ignore_alive
    assert radar_interface.rcp_tracks.message_states[0x520].ignore_alive
    assert radar_interface.rcp_tracks.message_states[0x53F].ignore_alive

    # This confirmed 0x52d target is the missing lead observed in a real
    # 64-slot Grandeur IG log. The lower bank still provides the cycle trigger.
    active_dat = bytes.fromhex("0060589b03fec0b2")
    lower_bank = [(addr, bytes(8), 1) for addr in range(0x500, 0x520)]
    radar_data = radar_interface.update([0, lower_bank + [(0x52D, active_dat, 1)]])
    point = next(point for point in radar_data.points if point.trackId == 77)

    assert not radar_data.errors.canError
    assert point.measured
    assert point.dRel == pytest.approx(math.cos(math.radians(2.2)) * 15.5)
    assert point.yRel == pytest.approx(-0.5 * math.sin(math.radians(2.2)) * 15.5)
    assert point.vRel == pytest.approx(1.78)
    assert point.aRel == pytest.approx(-0.04)

    # A 32-slot radar sends only the required lower bank. It must keep
    # publishing without a CAN error, and the optional point must not go stale.
    radar_data = radar_interface.update([0, lower_bank])
    assert not radar_data.errors.canError
    assert all(point.trackId != 77 for point in radar_data.points)


class TestCanfdGroup2Radar:
  @staticmethod
  def parse(dat):
    name = "RADAR_TRACK_3ac"
    parser = CANParser("hyundai_canfd_radar_generated", [(name, 20)], 1)
    parser.update([0, [(0x3AC, bytes.fromhex(dat), 1)]])
    return parser.vl[name]

  def test_tentative_overpass_reflection_is_not_confirmed(self):
    track = self.parse("cb6706810c0f2069acf0ff5cbc0680330200003dd0020000")

    assert track["VALID_CNT"] == 15
    assert track["VALID"] == 1
    assert track["LONG_DIST"] == pytest.approx(17.2)
    assert canfd_group2_track_status(track) == (True, 1)

  def test_confirmed_vehicle_remains_eligible(self):
    track = self.parse("dbc1db5261ff308db8230030be0b400100000000d0020000")

    assert track["VALID_CNT"] == 255
    assert track["VALID"] == 2
    assert track["LONG_DIST"] == pytest.approx(95.25)
    assert canfd_group2_track_status(track) == (True, 2)


class TestDensoRadar:
  @staticmethod
  def parse(addr, dat):
    name = f"RADAR_TRACK_{addr:x}"
    parser = CANParser("hyundai_kia_denso_front_radar_generated", [(name, 20)], 1)
    parser.update([0, [(addr, bytes.fromhex(dat), 1)]])
    return parser.vl[name]

  def test_active_track_signals(self):
    # Person walking toward the parked car, left of the camera center.
    track = self.parse(0x503, "bc047efcc1fe8b00")

    assert track["LONG_DIST"] == pytest.approx(7.1875)
    assert track["LAT_DIST"] == pytest.approx(-1.625)
    assert track["REL_SPEED"] == pytest.approx(-0.734375)
    assert track["OBJECT_STATE"] == 3

  def test_empty_track(self):
    track = self.parse(0x507, "53fff80000000081")

    assert track["LONG_DIST"] == pytest.approx(409.55)
    assert track["LAT_DIST"] == 0
    assert track["REL_SPEED"] == 0
    assert track["OBJECT_STATE"] == 0

  def test_long_range_lateral_distance(self):
    # Real driving sample: treating the signed field as -12 degrees would put
    # this target about 34 m sideways at 161 m. It is instead -3.0 m lateral.
    track = self.parse(0x506, "b664eafa00cd230b")

    assert track["LONG_DIST"] == pytest.approx(161.4625)
    assert track["LAT_DIST"] == pytest.approx(-3.0)
    assert track["OBJECT_STATE"] == 3

  def test_parser_selection_and_point_conversion(self, monkeypatch):
    class FakeParams:
      def get_int(self, key):
        return 1 if key == "EnableRadarTracks" else 0

    monkeypatch.setattr(radar_interface_module, "Params", FakeParams)
    cp = structs.CarParams()
    cp.carFingerprint = CAR.KIA_SORENTO
    cp.flags = 0
    cp.extFlags = HyundaiExtFlags.RADAR_GROUP4.value
    cp.radarUnavailable = False
    cp.safetyConfigs = [structs.CarParams.SafetyConfig()]

    radar_interface = RadarInterface(cp)

    assert radar_interface.radar_group4
    assert RADAR_MSG_COUNT4 == 8
    assert radar_interface.radar_msg_count == RADAR_MSG_COUNT4
    assert radar_interface.trigger_msg_tracks == 0x507

    active_dat = bytes.fromhex("bc047efcc1fe8b00")
    empty_dat = bytes.fromhex("bcfff80000000081")
    packets = [(addr, active_dat if addr == 0x503 else empty_dat, 1) for addr in range(0x500, 0x508)]
    radar_data = radar_interface.update([0, packets])
    point = next(point for point in radar_data.points if point.trackId == 35)

    assert point.measured
    assert point.dRel == pytest.approx(7.1875)
    assert point.yRel == pytest.approx(1.625)
    assert point.vRel == pytest.approx(-0.734375)
    assert math.isnan(point.aRel)

    # EN: Confirm that the long-range sample survives the filter and converts
    #     radar-left-negative to openpilot-left-positive coordinates.
    # KO: 장거리 샘플의 필터 통과와 레이더 좌측 음수 좌표가 openpilot 좌측
    #     양수 좌표로 변환되는지 확인함.
    long_range_dat = bytes.fromhex("b664eafa00cd230b")
    packets = [(addr, long_range_dat if addr == 0x506 else empty_dat, 1) for addr in range(0x500, 0x508)]
    radar_data = radar_interface.update([0, packets])
    point = next(point for point in radar_data.points if point.trackId == 38)

    assert point.dRel == pytest.approx(161.4625)
    assert point.yRel == pytest.approx(3.0)

    # EN: A state-0 raw detection must not enter a stable tracked-object slot.
    # KO: 상태 0인 raw detection이 안정적인 추적 객체 슬롯에 들어오지 않음을 확인함.
    raw_detection = bytes.fromhex("d702f4fc200000e4")
    packets = [(addr, raw_detection if addr == 0x503 else empty_dat, 1) for addr in range(0x500, 0x508)]
    radar_data = radar_interface.update([0, packets])
    assert not radar_data.points

    # EN: A real confirmed track beyond the former 205 m limit remains valid.
    # KO: 기존 205m 상한을 넘는 실제 확정 트랙도 유효하게 유지됨.
    confirmed_213m_track = bytes.fromhex("35854c0780f163e0")
    packets = [(addr, confirmed_213m_track if addr == 0x503 else empty_dat, 1) for addr in range(0x500, 0x508)]
    radar_data = radar_interface.update([0, packets])
    point = next(point for point in radar_data.points if point.trackId == 35)
    assert point.dRel == pytest.approx(213.275)
    assert point.yRel == pytest.approx(-3.75)

    # EN: The 325 m boundary is rejected, leaving ample separation from the
    #     409.55 m empty-slot sentinel.
    # KO: 325m 경계값을 제외해 409.55m 빈 슬롯 값과 충분한 간격을 확보함.
    boundary_track = bytes.fromhex("bccb200000000300")
    packets = [(addr, boundary_track if addr == 0x503 else empty_dat, 1) for addr in range(0x500, 0x508)]
    radar_data = radar_interface.update([0, packets])
    assert not radar_data.points

    # EN: The wider profile keeps a real stable track at 4.875 m, covering more
    #     of the outer adjacent lane than the conservative 4.5 m profile.
    # KO: 넓어진 필터에서 4.875m의 실제 안정 트랙을 유지해 보수적인 4.5m
    #     설정보다 바깥쪽 인접 차선을 더 넓게 포함함.
    outer_lane_track = bytes.fromhex("d80b66f640000300")
    packets = [(addr, outer_lane_track if addr == 0x503 else empty_dat, 1) for addr in range(0x500, 0x508)]
    radar_data = radar_interface.update([0, packets])
    point = next(point for point in radar_data.points if point.trackId == 35)
    assert point.yRel == pytest.approx(4.875)

    # EN: Tracks beyond the widened envelope are rejected as roadside clutter;
    #     this payload differs only in lateral distance (-7.0 m).
    # KO: 넓어진 범위를 벗어난 트랙은 도로변 잡음으로 제외함. 이 payload는
    #     횡방향 거리(-7.0m)만 다름.
    far_side_reflection = bytes.fromhex("d80b66f200000300")
    packets = [(addr, far_side_reflection if addr == 0x503 else empty_dat, 1) for addr in range(0x500, 0x508)]
    radar_data = radar_interface.update([0, packets])
    assert not radar_data.points


class TestRadarGroup3:
  @staticmethod
  def parse(addr, dat):
    name = f"RADAR_TRACK_{addr:x}"
    parser = CANParser("hyundai_canfd_radar_generated", [(name, 20)], 1)
    parser.update([0, [(addr, bytes.fromhex(dat), 1)]])
    return parser.vl[name]

  def test_group3_active_track(self):
    track = self.parse(0x406, "e1043b0f02590e692a227e16f80fe00f28fcc753a20a0000")

    assert track["OBJECT_LENGTH"] == pytest.approx(4.4)
    assert track["LONG_DIST"] == pytest.approx(55.4)
    assert track["LAT_DIST"] == pytest.approx(-3.0)
    assert track["REL_SPEED"] == pytest.approx(4.4)

  def test_group3_empty_track(self):
    track = self.parse(0x407, "c03d3b0000000000ff0700000000000000d0020000000000")

    assert track["OBJECT_LENGTH"] == 0
    assert track["LONG_DIST"] == pytest.approx(204.7)
    assert track["LAT_DIST"] == 0
    assert track["REL_SPEED"] == 0

  def test_group3_parser_selection(self, monkeypatch):
    class FakeParams:
      def get_int(self, key):
        return 1 if key == "EnableRadarTracks" else 0

    monkeypatch.setattr(radar_interface_module, "Params", FakeParams)
    monkeypatch.setattr(hyundaicanfd, "Params", FakeParams)
    cp = structs.CarParams()
    cp.carFingerprint = next(car for car, dbc in radar_interface_module.DBC.items() if "hyundai_canfd" in dbc[Bus.pt])
    cp.flags = HyundaiFlags.CANFD.value
    cp.extFlags = HyundaiExtFlags.RADAR_GROUP3.value
    cp.radarUnavailable = False
    cp.safetyConfigs = [structs.CarParams.SafetyConfig()]

    radar_interface = RadarInterface(cp)

    assert radar_interface.radar_group3
    assert radar_interface.radar_start_addr == RADAR_START_ADDR_CANFD3
    assert radar_interface.radar_msg_count == RADAR_MSG_COUNT3
    assert radar_interface.trigger_msg_tracks == 0x41D

    active_dat = bytes.fromhex("e1043b0f02590e692a227e16f80fe00f28fcc753a20a0000")
    empty_dat = bytes.fromhex("c03d3b0000000000ff0700000000000000d0020000000000")
    packets = [(addr, active_dat if addr == 0x406 else empty_dat, 1) for addr in range(0x400, 0x41E)]
    radar_data = radar_interface.update([0, packets])
    point = next(point for point in radar_data.points if point.trackId == 38)

    assert point.measured
    assert point.dRel == pytest.approx(53.1)
    assert point.yRel == pytest.approx(-3.0)
    assert point.vRel == pytest.approx(4.4)


class TestCornerRadarObjectIdentity:
  @staticmethod
  def set_bits(data, start, size, value):
    for offset in range(size):
      bit = start + offset
      data[bit // 8] |= ((value >> offset) & 1) << (bit % 8)

  @pytest.mark.parametrize(
    "dbc,msg_name,addr,age_signal,id_signal,age_start,id_start",
    (
      ("hyundai_canfd_corner_radar_180_generated", "CORNER_RADAR_180_OBJECTS_180", 0x180,
       "SLOT1_AGE", "SLOT1_OBJECT_ID", 32, 44),
      ("hyundai_canfd_corner_radar_235_generated", "CORNER_RADAR_235_OBJECTS_235", 0x235,
       "OBJ_AGE", "OBJ_OBJECT_ID", 32, 44),
    ),
  )
  def test_object_identity_signals(self, dbc, msg_name, addr, age_signal, id_signal, age_start, id_start):
    data = bytearray(32)
    self.set_bits(data, age_start, 8, 23)
    self.set_bits(data, id_start, 7, 46)
    parser = CANParser(dbc, [(msg_name, 33)], 1)
    parser.update([0, [(addr, bytes(data), 1)]])

    assert parser.vl[msg_name][age_signal] == 23
    assert parser.vl[msg_name][id_signal] == 46

  def test_track_id_survives_slot_move_and_resets_with_age(self):
    manager = CornerObjectTrackIdManager()
    first = [(240, 108, 240, 40, 25.0, 2.8, 1.0, -0.2, 0.0)]
    first_id = manager.get_track_ids("corner180", first)[240]
    next_age = [(240, 108, 241, 40, 25.1, 2.8, 1.0, -0.2, 0.0)]
    other_source = [(201, 108, 241, 40, 25.1, 2.8, 1.0, -0.2, 0.0)]
    reset_age = [(240, 108, 2, 40, 25.2, 2.8, 1.0, -0.2, 0.0)]

    assert first_id == CORNER_OBJECT_STABLE_TRACK_ID_START
    assert manager.get_track_ids("corner180", next_age)[240] == first_id
    assert manager.get_track_ids("corner235", other_source)[201] != first_id
    assert manager.get_track_ids("corner180", reset_age)[240] != first_id

  def test_same_object_id_at_different_positions_remains_distinct(self):
    manager = CornerObjectTrackIdManager()
    candidates = [
      (201, 32, 111, 35, 13.75, 2.90, 5.00, -0.65, 0.0),
      (202, 32, 191, 35, 149.35, -2.90, 3.05, 0.00, 0.0),
    ]

    objects = deduplicate_corner_candidates(candidates)
    track_ids = manager.get_track_ids("corner235", objects)

    assert len(objects) == 2
    assert track_ids[201] != track_ids[202]

  def test_interface_publishes_distant_same_id_objects(self):
    radar_interface = RadarInterface.__new__(RadarInterface)
    radar_interface.v_ego = 20.0
    radar_interface.corner_object_track_ids = CornerObjectTrackIdManager()
    radar_interface.pts = {}
    for slot_id in (201, 202):
      radar_interface.pts[slot_id] = structs.RadarData.RadarPoint()

    candidates = [
      (201, 32, 111, 35, 13.10, 2.90, 4.90, -0.65, 0.0),
      (202, 32, 191, 35, 149.70, -2.90, 3.05, 0.00, 0.0),
    ]
    radar_interface._apply_corner_objects(
      "corner235", candidates, (201, 202),
    )

    near = radar_interface.pts[201]
    far = radar_interface.pts[202]
    assert near.measured and far.measured
    assert near.trackId != far.trackId
    assert near.dRel == pytest.approx(13.10)
    assert far.dRel == pytest.approx(149.70)

  def test_physical_slot_handoff_is_deduplicated_and_keeps_track_id(self):
    manager = CornerObjectTrackIdManager()
    first = [(201, 46, 23, 40, 25.0, 2.8, 1.0, -0.2, 0.0)]
    first_id = manager.get_track_ids("corner235", first)[201]
    handoff = [
      (201, 46, 24, 38, 25.1, 2.75, 1.0, -0.2, 0.0),
      (207, 46, 25, 42, 25.2, 2.70, 1.0, -0.2, 0.0),
    ]

    objects = deduplicate_corner_candidates(handoff)
    track_ids = manager.get_track_ids("corner235", objects)

    assert len(objects) == 1
    assert track_ids[207] == first_id

  def test_clipped_side_object_position_is_valid(self):
    assert corner_object_position_valid(0.0, 2.8)
    assert corner_object_position_valid(25.0, 0.2)
    assert not corner_object_position_valid(0.0, 0.0)
    assert not corner_object_position_valid(0.0, 5.0)


class TestCornerRadar430CandidateFilter:
  @staticmethod
  def slot_word(distance_raw, meta13=0, b2=10, b3=2):
    return distance_raw | (meta13 << 13) | (b2 << 16) | (b3 << 24)

  @classmethod
  def message(cls, slots):
    words = [0x010d1f40] * 7
    for slot, word in slots.items():
      words[slot - 1] = word

    dat = bytearray(32)
    for idx, word in enumerate(words):
      dat[4 + idx * 4:8 + idx * 4] = int(word).to_bytes(4, "little")
    return bytes(dat)

  @staticmethod
  def build_interface(monkeypatch):
    class FakeParams:
      def get_int(self, key):
        return 1 if key == "EnableCornerRadar" else 0

    monkeypatch.setattr(radar_interface_module, "Params", FakeParams)
    monkeypatch.setattr(hyundaicanfd, "Params", FakeParams)
    cp = structs.CarParams()
    cp.carFingerprint = next(car for car, dbc in radar_interface_module.DBC.items() if "hyundai_canfd" in dbc[Bus.pt])
    cp.flags = HyundaiFlags.CANFD.value
    cp.extFlags = HyundaiExtFlags.CORNER_RADAR_OBJECTS_430.value
    cp.radarUnavailable = True
    cp.safetyConfigs = [structs.CarParams.SafetyConfig()]
    return RadarInterface(cp)

  @staticmethod
  def update_frames(radar_interface, packets, frames=5):
    radar_data = None
    for _ in range(frames):
      radar_data = radar_interface.update([0, packets])
    return radar_data

  def test_430_bins_are_not_promoted_to_live_tracks(self, monkeypatch):
    radar_interface = self.build_interface(monkeypatch)
    assert radar_interface.rcp_corner_objects_430 is None

    empty = self.message({})
    supported_bins = self.message({
      6: self.slot_word(1000),
      7: self.slot_word(1004),
    })
    packets = [(addr, supported_bins if addr == 0x436 else empty, 1) for addr in range(0x430, 0x438)]
    packets += [(addr, empty, 1) for addr in range(0x440, 0x448)]

    radar_data = self.update_frames(radar_interface, packets)

    assert all(str(point.radarSource) != "corner430" for point in radar_data.points)

  def test_430_expires_noncenter_inward_yvrel(self, monkeypatch):
    radar_interface = self.build_interface(monkeypatch)
    empty = self.message({})
    frame_defs = (
      (0x431, 4, 5),
      (0x433, 3, 4),
      (0x435, 2, 3),
      (0x430, 5, 6),
      (0x432, 4, 5),
      (0x434, 3, 4),
      (0x436, 2, 3),
    )

    radar_data = None
    for addr, first_slot, second_slot in frame_defs:
      msg = self.message({
        first_slot: self.slot_word(1000),
        second_slot: self.slot_word(1004),
      })
      packets = [(a, msg if a == addr else empty, 1) for a in range(0x430, 0x438)]
      packets += [(a, empty, 1) for a in range(0x440, 0x448)]
      radar_data = radar_interface.update([0, packets])
    radar_data = self.update_frames(radar_interface, packets, frames=3)
    points = {point.trackId: point for point in radar_data.points}

    assert 300 not in points
    assert all(str(point.radarSource) != "corner430" for point in points.values())


class TestBoschPublicationAlias:
  PID = 1_000_475

  @classmethod
  def objects(cls):
    return BoschPhysicalTracker().update(1_000_000_000, (
      BoschRawDetection(1_000_000_000, 9, 37.125, -1.25, -3.125, raw_word=1),))

  def test_slot_member_and_representative_changes_keep_alias(self):
    allocator = BoschPublicationAliasAllocator()
    obj = replace(self.objects()[0], physical_track_id=self.PID)
    aliases = []
    for index, slot in enumerate((9, 14, 21)):
      member = replace(obj.members[0], raw_track_id=101 + index,
                       detection=replace(obj.members[0].detection, slot=slot))
      changed = replace(obj, members=(member,), representative_raw_track_id=member.raw_track_id)
      alias = allocator.update(index * 100_000_000, [changed.physical_track_id], {self.PID})
      point = structs.RadarData.RadarPoint()
      bosch_fill_point(point, changed, 10., alias)
      aliases.append(point.trackId)
      assert changed.physical_track_id == self.PID
    assert aliases == [32, 32, 32]

  def test_short_publication_gap_preserves_alias(self):
    allocator = BoschPublicationAliasAllocator()
    first = allocator.update(0, [self.PID], {self.PID})
    assert allocator.update(200_000_000, [], {self.PID}) == {}
    assert allocator.current_usage == 1
    assert allocator.update(360_000_000, [self.PID], {self.PID}) == first

  def test_grace_boundary_and_expiry(self):
    allocator = BoschPublicationAliasAllocator()
    allocator.update(0, [self.PID], {self.PID})
    allocator.update(500_000_000, [], {self.PID})
    assert allocator.current_usage == 1
    allocator.update(600_000_000, [], {self.PID})
    assert allocator.current_usage == 0
    assert allocator.alias_to_physical == allocator.last_published_ns == {}
    assert allocator.update(610_000_000, [self.PID], {self.PID})[self.PID] == 33

  def test_return_after_grace_without_intermediate_update(self):
    allocator = BoschPublicationAliasAllocator()
    allocator.update(0, [self.PID], {self.PID})
    assert allocator.update(600_000_000, [self.PID], {self.PID})[self.PID] == 33

  def test_dead_physical_state_releases_immediately(self):
    allocator = BoschPublicationAliasAllocator()
    allocator.update(0, [self.PID], {self.PID})
    allocator.update(1, [], {})
    assert allocator.current_usage == 0
    assert allocator.alias_to_physical == allocator.last_published_ns == {}
    assert allocator.free_aliases[-1] == 32

  def test_fifo_reuses_release_order(self):
    allocator = BoschPublicationAliasAllocator()
    ids = set(range(self.PID, self.PID + 64))
    first = allocator.update(0, ids, ids)
    # Release alias 95 first, then 32: reuse must follow release time, not ID.
    ids.remove(self.PID + 63)
    allocator.update(1, ids, ids)
    ids.remove(self.PID)
    allocator.update(2, ids, ids)
    ids.update((2_000_000, 2_000_001))
    alias = allocator.update(3, ids, ids)
    assert alias[2_000_000] == first[self.PID + 63] == 95
    assert alias[2_000_001] == first[self.PID] == 32

  def test_full_pool_is_unique_in_range_without_denial(self):
    allocator = BoschPublicationAliasAllocator()
    ids = set(range(self.PID, self.PID + 64))
    alias = allocator.update(0, ids, ids)
    assert set(alias.values()) == set(range(32, 96))
    assert allocator.current_usage == allocator.peak_usage == 64
    assert allocator.denial_count == 0
    assert allocator.alias_to_physical == {a: pid for pid, a in alias.items()}
    for pid in ids:
      point = structs.RadarData.RadarPoint()
      bosch_fill_point(point, replace(self.objects()[0], physical_track_id=pid), 10., alias)
      assert 32 <= point.trackId <= 95
      assert str(point.radarSource) == 'frontRadar'

  def test_pool_pressure_reclaims_only_unpublished_grace_binding(self):
    allocator = BoschPublicationAliasAllocator()
    ids = set(range(self.PID, self.PID + 64))
    first = allocator.update(0, ids, ids)
    active = ids - {self.PID}
    allocator.update(1, active, ids)
    active.add(2_000_000)
    alias = allocator.update(2, active, ids | active)
    assert len(alias) == 64
    assert alias[2_000_000] == first[self.PID]
    assert all(alias[pid] == first[pid] for pid in active if pid in first)
    assert allocator.denial_count == 0
    assert allocator.grace_eviction_count == 1

  def test_impossible_oversized_scan_is_not_silently_truncated(self):
    allocator = BoschPublicationAliasAllocator()
    first = allocator.update(0, [self.PID], {self.PID})
    ids = set(range(self.PID, self.PID + 65))
    with pytest.raises(ValueError, match='alias pool'):
      allocator.update(1, ids, ids)
    assert allocator.denial_count == 1
    assert allocator.physical_to_alias == first

  def test_optional_alias_preserves_native_fields_and_scc(self):
    objects = self.objects()
    physical_id = objects[0].physical_track_id
    native = bosch_to_native_radar_data(objects, 1_000_000_000, v_ego=10.)
    assert native.points[0].trackId == physical_id
    before, after = structs.RadarData.new_message(), structs.RadarData.new_message()
    scc = dict(trackId=0, dRel=80., vRel=-.5, radarSource='scc', measured=True)
    before.points, after.points = [scc], [scc]
    bosch_append_points(before, objects, 10., 1_013_219_371, alias=None)
    bosch_append_points(after, objects, 10., 1_013_219_371, alias={physical_id: 37})
    assert after.points[1].trackId == 37
    assert before.points[1].trackId == physical_id
    after.points[1].trackId = physical_id
    assert before.to_bytes() == after.to_bytes()

  def test_perf_counters_survive_logging_reset(self):
    provider = BoschRadarProvider(1, qualification=False)
    provider.publication_aliases.update(0, [self.PID], {self.PID})
    for _ in range(2):
      message = provider.perf_message()
      assert 'alias_usage=1/64' in message
      assert 'alias_peak=1' in message
      assert 'alias_denial=0' in message

  def test_qualified_publication_and_stale_cleanup_preserve_scc(self, monkeypatch):
    provider = BoschRadarProvider(1)
    interface = RadarInterface.__new__(RadarInterface)
    interface.bosch, interface.v_ego = provider, 0.
    def base_update(*args):
      data = structs.RadarData.new_message()
      data.points = [dict(trackId=0, dRel=80., vRel=0., radarSource='scc', measured=True)]
      return data
    monkeypatch.setattr(radar_interface_module.RadarInterfaceBase, 'update_carrot', base_update)
    for index, lateral in enumerate((-4.5, 1.5)):
      now = 1_000_000_000 + index * 100_000_000
      objects = provider.tracker.update(now, [BoschRawDetection(now, 9, 30., lateral, 0., raw_word=1)])
      interface._bosch_objects = provider.qualifier.update(objects, now, 0.)
      provider.last_scan_timestamp_ns = interface._bosch_now_ns = now
      data = interface.update_carrot(0., 0., now * 1e-9, [])
      assert data.points[0].to_dict() == base_update().points[0].to_dict()
      assert len(data.points) == index + 1
      if index:
        assert data.points[1].trackId == 32
        assert provider.publication_aliases.alias_to_physical[32] == objects[0].physical_track_id
    # Empty/stale publications still age bindings, including a stalled provider
    # whose group state has not been advanced by another CAN scan.
    interface._bosch_now_ns += 600_000_000
    data = interface.update_carrot(0., 0., interface._bosch_now_ns * 1e-9, [])
    assert data.to_bytes() == base_update().to_bytes()
    assert provider.publication_aliases.current_usage == 0

  @pytest.mark.parametrize(('mode', 'missing_scc'), ((1, False), (2, False), (3, False), (1, True)))
  def test_production_boundary_uses_alias_and_preserves_tracking(self, monkeypatch, mode, missing_scc):
    class FakeParams:
      def get_int(self, key):
        return mode if key == 'EnableRadarTracks' else 0

    monkeypatch.setattr(radar_interface_module, 'Params', FakeParams)
    monkeypatch.setattr(hyundaicanfd, 'Params', FakeParams)
    cp = structs.CarParams()
    cp.carFingerprint = CAR.HYUNDAI_GRANDEUR_IG
    cp.extFlags = int(HyundaiExtFlags.BOSCH_RADAR | HyundaiExtFlags.BOSCH_RADAR_BUS1)
    cp.radarTimeStep = .05
    cp.safetyConfigs = [structs.CarParams.SafetyConfig()]
    interface = RadarInterface(cp)
    assert interface.rcp_tracks is None
    if missing_scc and mode == 1:
      interface.rcp_scc = None
    publications = []
    inactive = radar_interface_module.BOSCH_INACTIVE_WORD.to_bytes(4, 'little')
    active = (120 | (1024 << 10) | (512 << 21)).to_bytes(4, 'little')
    for frame in range(30):
      now = 1_000_000_000 + frame * 10_000_000
      packets = []
      if frame in (0, 10, 20):
        slot = (9, 14, 21)[frame // 10]
        messages = []
        for address in range(0x602, 0x612):
          words = [active if (address - 0x602) * 2 + half == slot else inactive for half in (0, 1)]
          messages.append((address, b''.join(words), interface.bosch.bus))
        messages.append((0x612, bytes([0]) + frame.to_bytes(3, 'little') + bytes(4), interface.bosch.bus))
        packets = [(now, messages)]
      interface.set_bosch_context(now)
      result = interface.update_carrot(10., 0., now * 1e-9, packets)
      if result is not None:
        front = [p for p in result.points if str(p.radarSource) == 'frontRadar']
        assert len(front) == 1
        point = front[0]
        assert point.trackId == 32 and point.measured
        assert (point.dRel, point.yRel, point.vRel, point.vLead) == (30., 0., 0., 10.)
        assert point.trackState == 0
        if mode == 1:
          assert not result.errors.canError
        physical_id = interface.bosch._debug_objects[0].physical_track_id
        assert physical_id == 1_000_000
        assert interface.bosch.publication_aliases.alias_to_physical[point.trackId] == physical_id
        assert interface.bosch.slot_to_ids[slot][1] == physical_id
        publications.append(frame)
    assert publications == [9, 14, 19, 24, 29]
    assert interface.bosch.publication_aliases.denial_count == 0


class TestBoschPhysicalIdContinuityTrace:
  """The physical layer carries an ID on raw-member overlap alone.

  These cover the shadow trace that records how far the inheriting state is
  from the state that ID last published. The trace is diagnostic only: it must
  never change an assignment, and it must not silently start rejecting one.
  """
  SCAN_NS = 100_000_000

  @staticmethod
  def track(raw_track_id, slot, d_rel, y_rel, v_rel, timestamp_ns, age_scans, recovered=False):
    detection = BoschRawDetection(timestamp_ns, slot, d_rel, y_rel, v_rel, raw_word=1)
    return BoschRawTrack(raw_track_id, detection, age_scans, recovered)

  @classmethod
  def coasted_carry_scans(cls):
    """Two returns group, one drops out, then the other reappears far away.

    This is the recorded route-252 shape: the physical ID follows a member kept
    only for coasting, to a position the previous object never occupied.
    """
    scans = []
    for index in range(3):
      ns = (index + 1) * cls.SCAN_NS
      scans.append((ns, (cls.track(55, 8, 11.50, -0.625, 0.50, ns, index + 1),
                         cls.track(56, 20, 9.25, -1.71875, 0.25, ns, index + 1))))
    ns = 4 * cls.SCAN_NS
    scans.append((ns, (cls.track(56, 20, 9.25, -1.84375, 0.25, ns, 4),)))
    ns = 5 * cls.SCAN_NS
    scans.append((ns, (cls.track(55, 7, 14.75, 1.78125, 0.75, ns, 4, recovered=True),)))
    return scans

  @classmethod
  def run(cls, scans, trace):
    manager = BoschObjectGroupManager()
    manager.trace_decisions = trace
    outputs, decisions = [], []
    for timestamp_ns, tracks in scans:
      outputs.append(manager.update(timestamp_ns, tracks))
      decisions.append(manager.last_decisions)
    return manager, outputs, decisions

  @staticmethod
  def signature(outputs):
    return [[(obj.physical_track_id, obj.timestamp_ns, obj.d_rel, obj.y_rel, obj.v_rel,
              obj.representative_raw_track_id, obj.age_scans, obj.member_slots,
              tuple(m.raw_track_id for m in obj.members), obj.grouping_evidence)
             for obj in scan] for scan in outputs]

  def test_trace_does_not_change_any_assignment(self):
    scans = self.coasted_carry_scans()
    off_manager, off_outputs, off_decisions = self.run(scans, False)
    on_manager, on_outputs, _ = self.run(scans, True)

    assert self.signature(off_outputs) == self.signature(on_outputs)
    assert dict(off_manager.stats) == dict(on_manager.stats)
    assert off_manager.next_id == on_manager.next_id
    # Off leaves nothing behind to read.
    assert all(scan == () for scan in off_decisions)

  def test_coasted_member_carries_the_id_without_a_continuity_check(self):
    scans = self.coasted_carry_scans()
    _, outputs, decisions = self.run(scans, True)

    carrier = next(obj for obj in outputs[3] if 56 in {m.raw_track_id for m in obj.members})
    inheritor = next(obj for obj in outputs[4] if 55 in {m.raw_track_id for m in obj.members})
    # Same physical ID, although raw 55 was never part of the published object.
    assert inheritor.physical_track_id == carrier.physical_track_id
    assert inheritor.d_rel == 14.75

    decision = next(d for d in decisions[4] if d.physical_track_id == inheritor.physical_track_id)
    assert decision.previous_member_raw_track_ids == (56,)
    assert decision.member_raw_track_ids == (55,)
    assert decision.member_overlap == 1          # scored against the coasting set
    assert decision.observed_member_overlap == 0  # but absent from the last published object
    assert decision.grouping_evidence == "single_return"
    # A single-member cluster skips representative selection, so production never
    # evaluates this residual; only the trace does.
    assert decision.predicted_d_rel == pytest.approx(9.25 + 0.25 * 0.1)
    assert decision.residual_d_m == pytest.approx(14.75 - (9.25 + 0.25 * 0.1))
    assert decision.residual_y_m > 3.0

  def test_trace_records_a_residual_for_every_inherited_id(self):
    scans = self.coasted_carry_scans()
    _, outputs, decisions = self.run(scans, True)

    for scan_outputs, scan_decisions in zip(outputs, decisions):
      assert len(scan_decisions) == len(scan_outputs)
      for decision in scan_decisions:
        if decision.assignment_mode == "created":
          assert decision.previous_physical_track_id is None
          assert decision.residual_d_m is None
        else:
          assert decision.residual_d_m is not None
          assert decision.residual_y_m is not None
          assert decision.residual_v_mps is not None
          assert decision.dt_s > 0.0


class TestBoschRawAssociationTrace:
  """Raw returns are matched globally, and a coasted track outbids a new one.

  unmatched_cost is charged twice when a track dies and a return is born, so a
  single expensive match can win. The trace records what that match cost and
  what the return could have had instead.
  """
  SCAN_NS = 100_000_000

  @staticmethod
  def detection(slot, d_rel, y_rel, v_rel, timestamp_ns):
    return BoschRawDetection(timestamp_ns, slot, d_rel, y_rel, v_rel, raw_word=1)

  @classmethod
  def coasted_recovery_scans(cls):
    """A return drops out for one scan and comes back 3.25 m further away."""
    first = cls.SCAN_NS
    second = 2 * cls.SCAN_NS
    third = 3 * cls.SCAN_NS
    return [
      (first, (cls.detection(7, 11.50, 0.0, 0.0, first),
               cls.detection(20, 9.25, -1.84375, 0.25, first))),
      (second, (cls.detection(20, 9.25, -1.84375, 0.25, second),)),
      (third, (cls.detection(7, 14.75, 0.0, 0.0, third),)),
    ]

  @classmethod
  def run(cls, scans, trace):
    manager = BoschRawTrackManager()
    manager.trace_decisions = trace
    outputs, decisions = [], []
    for timestamp_ns, detections in scans:
      outputs.append(manager.update(timestamp_ns, detections))
      decisions.append(manager.last_decisions)
    return manager, outputs, decisions

  @staticmethod
  def signature(outputs):
    return [[(track.raw_track_id, track.slot, track.previous_slot, track.age_scans,
              track.recovered, track.d_rel, track.y_rel, track.v_rel)
             for track in scan] for scan in outputs]

  def test_trace_does_not_change_any_match(self):
    scans = self.coasted_recovery_scans()
    off_manager, off_outputs, off_decisions = self.run(scans, False)
    on_manager, on_outputs, _ = self.run(scans, True)

    assert self.signature(off_outputs) == self.signature(on_outputs)
    assert off_manager.stats == on_manager.stats
    assert off_manager.next_id == on_manager.next_id
    assert all(scan == () for scan in off_decisions)

  def test_coasted_track_keeps_its_id_across_a_near_gate_jump(self):
    scans = self.coasted_recovery_scans()
    _, outputs, decisions = self.run(scans, True)

    first_id = next(track.raw_track_id for track in outputs[0] if track.slot == 7)
    recovered = outputs[2][0]
    assert recovered.raw_track_id == first_id
    assert recovered.recovered

    decision = decisions[2][0]
    assert decision.raw_track_id == first_id
    assert decision.created is False
    assert decision.predicted_d_rel == pytest.approx(11.50)
    assert decision.residual_d_m == pytest.approx(3.25)
    # Inside the 3.5 m distance gate, so nothing rejects it; the trace is the
    # only place the size of the step is written down.
    assert decision.residual_d_m < BoschRawTrackingConfig().distance_gate_m
    assert decision.chosen_cost == pytest.approx((3.25 / 3.5) ** 2 / 3 - 0.03)
    assert decision.candidate_count == 1
    assert decision.best_alternative_cost is None

  def test_trace_reports_the_candidate_a_new_track_declined(self):
    """A closer return takes the coasted track, so the far one starts a new ID."""
    scans = self.coasted_recovery_scans()[:2]
    third = 3 * self.SCAN_NS
    scans.append((third, (self.detection(3, 11.50, 0.0, 0.0, third),
                          self.detection(7, 14.75, 0.0, 0.0, third),
                          self.detection(20, 9.25, -1.84375, 0.25, third))))
    _, outputs, decisions = self.run(scans, True)

    born = next(d for d in decisions[2] if d.created)
    assert born.slot == 7
    assert born.chosen_cost is None
    # The coasted track was a gated candidate for this return and lost it to the
    # closer one; without the trace that alternative leaves no record at all.
    assert born.candidate_count == 1
    assert born.best_alternative_cost == pytest.approx((3.25 / 3.5) ** 2 / 3 - 0.03)
