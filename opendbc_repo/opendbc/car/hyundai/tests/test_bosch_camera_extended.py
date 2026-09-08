import math
from dataclasses import replace

import pytest

from opendbc.car.hyundai.radar_interface import (
  BOSCH_CAMERA_ASSOC_AMBIGUOUS,
  BOSCH_CAMERA_ASSOC_ASSIGNED,
  BOSCH_CAMERA_ASSOC_UNRESOLVED,
  BOSCH_CAMERA_E2_HOLD_NS,
  BOSCH_CAMERA_EXTENDED_ACTIVE,
  BOSCH_CAMERA_EXTENDED_ACTIVE_TEST,
  BOSCH_CAMERA_EXTENDED_OFF,
  BOSCH_CAMERA_EXTENDED_SHADOW,
  BOSCH_CAMERA_HEADER,
  BOSCH_TRUCK_A0_RECOVERY_HOLD_SCANS,
  BoschCameraCycleCache,
  BoschCameraExtendedGrouping,
  BoschCameraObject,
  BoschPhysicalObject,
  BoschRawDetection,
  BoschRawTrack,
  BoschRadarProvider,
  RadarInterface,
  bosch_append_points,
)


def signed(value, bits):
  return value & ((1 << bits) - 1)


def camera_frames(counter, objects):
  frames = [(BOSCH_CAMERA_HEADER, len(objects) | (counter << 52))]
  for slot, obj in enumerate(objects):
    obj_id, long_m, lat_m, vrel_mps, width_m, cls, ext, right, left = obj
    a = (obj_id | (round(long_m / .0625) << 8) |
         (signed(round(lat_m / .0625), 12) << 20) |
         (signed(round(vrel_mps / .0625), 12) << 40) | (counter << 52))
    b = round(width_m / .05) | (cls << 48) | (ext << 50) | (counter << 52)
    c = (signed(round(right * 4496.3), 13) << 18) | (signed(round(left * 4496.3), 13) << 31) | (counter << 52)
    frames.extend(((0x739 + 3 * slot, a), (0x73a + 3 * slot, b), (0x73b + 3 * slot, c)))
  return frames


def feed(cache, ns, counter, objects, omit=(), counter_override=None):
  for address, word in camera_frames(counter, objects):
    if address in omit:
      continue
    if counter_override and address in counter_override:
      word &= ~(0xf << 52)
      word |= counter_override[address] << 52
    cache.ingest(ns, address, word.to_bytes(8, 'little'))


def physical(pid, d_rel, y_rel=0., v_rel=0., ns=1_000_000_000, age=4):
  raw = pid - 999_000
  detection = BoschRawDetection(ns, raw % 32, float(d_rel), float(y_rel), float(v_rel), 1)
  member = BoschRawTrack(raw, detection, age, False)
  return BoschPhysicalObject(pid, ns, (member,), raw, float(d_rel), float(y_rel), float(v_rel),
                             False, False, age, 'single_return')


class TestBoschCameraCycleCache:
  OBJ = (7, 20., -1.25, -2.5, 2.65, 1, 0, -.08, .06)

  def test_scalar_decode_matches_frozen_bit_layout(self):
    cache = BoschCameraCycleCache()
    feed(cache, 1_000_000_000, 3, [self.OBJ])
    snapshot = cache.snapshot(1_001_000_000)
    assert snapshot is not None
    objects, count, cycle, complete_ns = snapshot
    obj = objects[0]
    assert (count, cycle, complete_ns) == (1, 0, 1_000_000_000)
    assert (obj.obj_id, obj.episode, obj.class_code) == (7, 1, 1)
    assert (obj.long_m, obj.lat_m, obj.vrel_mps) == (20., -1.25, -2.5)
    assert obj.width_m == pytest.approx(2.65)
    assert obj.angle_right == pytest.approx(round(-.08 * 4496.3) / 4496.3)
    assert obj.angle_left == pytest.approx(round(.06 * 4496.3) / 4496.3)

  def test_counter_mixing_and_incomplete_cycles_are_unavailable(self):
    for omit, override in [((0x73a,), None), ((), {0x73b: 4})]:
      cache = BoschCameraCycleCache()
      feed(cache, 1_000_000_000, 3, [self.OBJ], omit=omit, counter_override=override)
      assert cache.snapshot(1_010_000_000) is None

  def test_dense_count_and_occupied_slot_must_agree(self):
    cache = BoschCameraCycleCache()
    bad = list(self.OBJ)
    bad[0] = bad[1] = bad[2] = bad[3] = 0
    feed(cache, 1_000_000_000, 0, [tuple(bad)])
    assert cache.snapshot(1_010_000_000) is None

  def test_id_episode_survives_slot_handoff_but_not_two_missing_cycles(self):
    cache = BoschCameraCycleCache()
    feed(cache, 1_000_000_000, 0, [self.OBJ])
    first = cache.snapshot(1_001_000_000)[0][0].episode
    other = (9, 18., 0., 0., 1.7, 2, 0, -.1, .1)
    feed(cache, 1_040_000_000, 1, [other, self.OBJ])
    moved = cache.snapshot(1_041_000_000)
    assert moved[0][1].episode == first
    feed(cache, 1_080_000_000, 2, [other])
    feed(cache, 1_120_000_000, 3, [other])
    feed(cache, 1_160_000_000, 4, [self.OBJ])
    assert cache.snapshot(1_161_000_000)[0][0].episode != first

  def test_future_and_stale_snapshots_are_never_selected(self):
    cache = BoschCameraCycleCache()
    feed(cache, 1_000_000_000, 0, [self.OBJ])
    assert cache.snapshot(999_999_999) is None
    assert cache.snapshot(1_160_000_000) is not None
    assert cache.snapshot(1_160_000_001) is None


class TestBoschCameraAssociationAndGeometry:
  def test_frozen_a0_many_to_one_and_ambiguous(self):
    a, b = physical(1_000_001, 15.), physical(1_000_002, 19.)
    cam = BoschCameraObject(7, 11, 20., 0., 0., 1.7, 1, .1, -.1)
    assert BoschCameraExtendedGrouping._associate(a, [cam], 1) == (BOSCH_CAMERA_ASSOC_ASSIGNED, 11, 1)
    assert BoschCameraExtendedGrouping._associate(b, [cam], 1) == (BOSCH_CAMERA_ASSOC_ASSIGNED, 11, 1)
    twin = BoschCameraObject(8, 12, 20., 0., 0., 1.7, 1, .1, -.1)
    assert BoschCameraExtendedGrouping._associate(a, [cam, twin], 2)[0] == BOSCH_CAMERA_ASSOC_AMBIGUOUS
    far = physical(1_000_003, 25.1)
    assert BoschCameraExtendedGrouping._associate(far, [cam], 1)[0] == BOSCH_CAMERA_ASSOC_UNRESOLVED

  @pytest.mark.parametrize(('dd', 'expected'), ((3.0, False), (3.25, True), (12.0, True), (12.01, False)))
  def test_distance_boundaries(self, dd, expected):
    assert BoschCameraExtendedGrouping._geometry(physical(1_000_001, 20.), physical(1_000_002, 20. + dd), 10., 0.)[3] is expected

  @pytest.mark.parametrize(('dy', 'dv', 'expected'), ((1.5, 1.5, True), (1.5001, 0., False), (0., 1.5001, False)))
  def test_lateral_velocity_boundaries(self, dy, dv, expected):
    assert BoschCameraExtendedGrouping._geometry(physical(1_000_001, 20., 0., 0.), physical(1_000_002, 24., dy, dv), 10., 0.)[3] is expected

  def test_stationary_moving_conflict(self):
    a = physical(1_000_001, 20., v_rel=-10.)
    b = physical(1_000_002, 24., v_rel=-8.5)
    assert not BoschCameraExtendedGrouping._geometry(a, b, 10., 0.)[3]

  def test_complete_link_rejects_single_link_chain(self):
    objects = (physical(1_000_001, 10.), physical(1_000_002, 14.), physical(1_000_003, 18.))
    edges = {(1_000_001, 1_000_002): 1., (1_000_002, 1_000_003): 1.}
    groups = BoschCameraExtendedGrouping._complete_link(objects, edges, ())
    assert max(map(len, groups)) == 2


class TestBoschCameraE2Overlay:
  @staticmethod
  def statuses(grouping, mapping):
    grouping._associate = lambda obj, *_: mapping.get(obj.physical_track_id, (BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1))
    grouping.camera.snapshot = lambda ns: ([BoschCameraObject()], 1, 0, ns)

  def test_off_is_exact_identity_and_has_no_camera_state(self):
    objects = (physical(1_000_001, 15.), physical(1_000_002, 19.))
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_OFF)
    assert grouping.camera is None
    assert grouping.update(1_000_000_000, objects, 10.) is objects

  def test_shadow_and_active_preserve_observations_while_grouping(self):
    objects = (physical(1_000_001, 15.), physical(1_000_002, 19.))
    mapping = {pid: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1) for pid in (1_000_001, 1_000_002)}
    shadow = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_SHADOW)
    self.statuses(shadow, mapping)
    assert shadow.update(1_000_000_000, objects, 10.) is objects
    assert shadow.last_groups == ((1_000_001, 1_000_002),)
    active = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    self.statuses(active, mapping)
    output = active.update(1_000_000_000, objects, 10.)
    assert output is objects
    assert active.last_groups == shadow.last_groups
    self.statuses(active, {})
    split = active.update(1_100_000_000, objects, 10.)
    assert len(split) == 2

  @pytest.mark.parametrize(('elapsed', 'held'), ((249_000_000, True), (250_000_000, True), (250_000_001, False)))
  def test_e2_timeout_is_exact_nanoseconds(self, elapsed, held):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    objects = (physical(1_000_001, 15.), physical(1_000_002, 19.))
    confirmed = {pid: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1) for pid in (1_000_001, 1_000_002)}
    self.statuses(grouping, confirmed)
    grouping.update(1_000_000_000, objects, 10.)
    now = 1_000_000_000
    while now + 100_000_000 < 1_000_000_000 + elapsed:
      now += 100_000_000
      self.statuses(grouping, {1_000_001: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1)})
      grouping.update(now, objects, 10.)
    self.statuses(grouping, {1_000_001: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1)})
    output = grouping.update(1_000_000_000 + elapsed, objects, 10.)
    assert output is objects
    assert bool(grouping.last_groups) is held

  @pytest.mark.parametrize(('gap', 'held'), ((159_000_000, True), (160_000_000, True), (160_000_001, False)))
  def test_observation_gap_boundary(self, gap, held):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    objects = (physical(1_000_001, 15.), physical(1_000_002, 19.))
    self.statuses(grouping, {pid: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1) for pid in (1_000_001, 1_000_002)})
    grouping.update(1_000_000_000, objects, 10.)
    self.statuses(grouping, {1_000_001: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1)})
    assert grouping.update(1_000_000_000 + gap, objects, 10.) is objects
    assert bool(grouping.last_groups) is held

  @pytest.mark.parametrize('veto', ['different', 'ambiguous', 'class', 'geometry', 'no_anchor'])
  def test_e2_current_hard_vetoes(self, veto):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    objects = (physical(1_000_001, 15.), physical(1_000_002, 19.))
    self.statuses(grouping, {pid: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1) for pid in (1_000_001, 1_000_002)})
    grouping.update(1_000_000_000, objects, 10.)
    mapping = {1_000_001: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1)}
    current = objects
    if veto == 'different':
      mapping[1_000_002] = (BOSCH_CAMERA_ASSOC_ASSIGNED, 8, 1)
    elif veto == 'ambiguous':
      mapping[1_000_002] = (BOSCH_CAMERA_ASSOC_AMBIGUOUS, -1, -1)
    elif veto == 'class':
      mapping[1_000_001] = (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 2)
    elif veto == 'geometry':
      current = (objects[0], physical(1_000_002, 27.1))
    elif veto == 'no_anchor':
      mapping = {}
    self.statuses(grouping, mapping)
    assert grouping.update(1_100_000_000, current, 10.) is current
    assert grouping.last_groups == ()

  def test_coast_never_recruits_a_new_member(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    objects = (physical(1_000_001, 15.), physical(1_000_002, 19.), physical(1_000_003, 23.))
    self.statuses(grouping, {
      1_000_001: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1),
      1_000_002: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1),
    })
    grouping.update(1_000_000_000, objects, 10.)
    self.statuses(grouping, {
      1_000_001: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1),
      1_000_003: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1),
    })
    output = grouping.update(1_100_000_000, objects, 10.)
    assert grouping.last_groups == ((1_000_001, 1_000_002),)
    assert output is objects


class TestBoschCameraPublicationSafety:
  @staticmethod
  def group(overlay, objects, ns):
    TestBoschCameraE2Overlay.statuses(overlay, {
      obj.physical_track_id: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1) for obj in objects
    })
    return overlay.update(ns, objects, 10.)

  def test_original_representative_preserves_s12_oem_tie_policy(self):
    ns = 1_000_000_000
    near = replace(physical(1_000_002, 13., .5, 2., ns, 1), vision_supported=True)
    far = replace(physical(1_000_001, 16.25, .46875, 2., ns, 43), vision_supported=True, oem_selected=True)
    overlay = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    output = self.group(overlay, (far, near), ns)
    assert output == (far, near)
    assert overlay.representatives[0].representative_pid == far.physical_track_id
    from opendbc.car import structs
    data = structs.RadarData.new_message()
    bosch_append_points(data, output, 10., ns + 10_000_000)
    assert data.points[1].dRel == pytest.approx(13.02, abs=1e-6)
    assert (data.points[1].yRel, data.points[1].vRel) == (near.y_rel, near.v_rel)
    assert data.points[0].dRel == pytest.approx(16.27, abs=1e-6)

  def test_original_continuity_can_keep_farther_previous_representative(self):
    overlay = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    first = (physical(1_000_001, 16.25), physical(1_000_003, 20.))
    assert self.group(overlay, first, 1_000_000_000) is first
    assert overlay.representatives[0].representative_pid == first[0].physical_track_id
    current = (physical(1_000_001, 16.25, ns=1_100_000_000), physical(1_000_002, 13., ns=1_100_000_000))
    assert self.group(overlay, current, 1_100_000_000) is current
    assert overlay.representatives[0].representative_pid == current[0].physical_track_id
    assert overlay.representative_switches == 0

  def test_reordered_members_do_not_chatter(self):
    overlay = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(20):
      ns = 1_000_000_000 + i * 100_000_000
      members = (physical(1_000_001, 13. + i * .25, ns=ns), physical(1_000_002, 16.25 + i * .25, ns=ns))
      ordered = members if i % 2 else members[::-1]
      assert self.group(overlay, ordered, ns) is ordered
      assert overlay.representatives[0].representative_pid == members[0].physical_track_id
    assert overlay.representative_switches == 0

  @pytest.mark.parametrize('mode', (BOSCH_CAMERA_EXTENDED_OFF, BOSCH_CAMERA_EXTENDED_SHADOW, BOSCH_CAMERA_EXTENDED_ACTIVE))
  def test_provider_qualifier_receives_complete_baseline_tuple(self, monkeypatch, mode):
    provider = BoschRadarProvider(1, camera_extended_mode=mode)
    objects = (physical(1_000_001, 13.), physical(1_000_002, 16.25))
    provider.tracker.update(900_000_000, (), v_ego=10.)
    monkeypatch.setattr(provider.tracker, 'update', lambda *a, **kw: objects)
    if mode != BOSCH_CAMERA_EXTENDED_OFF:
      TestBoschCameraE2Overlay.statuses(provider.camera_extended, {
        obj.physical_track_id: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1) for obj in objects})
    seen = []
    original = provider.qualifier.update
    def qualify(got, *a, **kw):
      seen.append(got)
      return original(got, *a, **kw)
    monkeypatch.setattr(provider.qualifier, 'update', qualify)
    output = provider._finish_scan(1_000_000_000, 0, (), 10., 0., ())
    assert seen == [objects] and seen[0] is objects
    assert output is objects
    view = provider.publication_view(output)
    assert view is output

  def test_missing_qualified_representative_preserves_other_points(self):
    provider = BoschRadarProvider(1, camera_extended_mode=BOSCH_CAMERA_EXTENDED_ACTIVE)
    objects = (physical(1_000_001, 13.), physical(1_000_002, 16.25))
    self.group(provider.camera_extended, objects, 1_000_000_000)
    qualified = (objects[1],)
    assert provider.publication_view(qualified) is qualified

  @pytest.mark.parametrize('disappear_reappear', (False, True))
  def test_native_boundary_preserves_fifo_and_every_member_history_identity(self, monkeypatch, disappear_reappear):
    from opendbc.car import structs
    import opendbc.car.hyundai.radar_interface as module
    def base_update(*args):
      data = structs.RadarData.new_message()
      data.points = [dict(trackId=0, dRel=80., vRel=0., radarSource='scc', measured=True)]
      return data
    monkeypatch.setattr(module.RadarInterfaceBase, 'update_carrot', base_update)
    interfaces = []
    for mode in (BOSCH_CAMERA_EXTENDED_OFF, BOSCH_CAMERA_EXTENDED_ACTIVE):
      interface = RadarInterface.__new__(RadarInterface)
      interface.bosch = BoschRadarProvider(1, qualification=False, camera_extended_mode=mode)
      interface.v_ego = 10.
      interfaces.append(interface)
    unrelated_alias = None
    preserved_samples = 0
    for i in range(24):
      ns = 1_000_000_000 + i * 100_000_000
      objects = [physical(1_000_001, 13., ns=ns), physical(1_010_690, 30., 3., ns=ns)]
      if not disappear_reappear or i not in (8, 9, 10):
        objects.append(physical(1_000_002, 16.25, ns=ns))
      if i >= 5:
        objects.append(physical(1_001_000 + i // 3, 60., -4., ns=ns))
      objects = tuple(objects)
      outputs = []
      for interface in interfaces:
        p = interface.bosch
        p.tracker.group_manager.states = {o.physical_track_id: None for o in objects}
        p.last_scan_timestamp_ns = interface._bosch_now_ns = ns
        interface._bosch_objects = objects
        if p.camera_extended.mode == BOSCH_CAMERA_EXTENDED_ACTIVE:
          self.group(p.camera_extended, objects, ns)
        outputs.append(interface.update_carrot(10., 0., ns * 1e-9, []))
      a, b = (x.bosch.publication_aliases for x in interfaces)
      assert a.physical_to_alias == b.physical_to_alias
      assert a.last_published_ns == b.last_published_ns
      assert list(a.free_aliases) == list(b.free_aliases)
      alias = a.physical_to_alias[1_010_690]
      unrelated_alias = alias if unrelated_alias is None else unrelated_alias
      assert alias == unrelated_alias
      for output in outputs:
        assert output.points[0].to_dict() == base_update().points[0].to_dict()
        assert any(p.trackId == alias and p.dRel == 30. for p in output.points)
      if 1_000_002 in a.physical_to_alias:
        duplicate_alias = a.physical_to_alias[1_000_002]
        assert any(p.trackId == duplicate_alias for p in outputs[1].points)
        preserved_samples += 1
      assert len(a.physical_to_alias) <= 64
    assert preserved_samples > 10


class TestBoschCameraConservativePublication:
  @pytest.mark.parametrize('scans', (1, 2, 3, 4, 5, 6, 50))
  def test_group_age_never_authorizes_history_loss(self, scans):
    provider = BoschRadarProvider(1, camera_extended_mode=BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(scans):
      ns = 1_000_000_000 + i * 99_123_456
      objects = (physical(1_000_001, 15., ns=ns, age=i+1), physical(1_000_002, 19., ns=ns, age=i+1))
      assert TestBoschCameraPublicationSafety.group(provider.camera_extended, objects, ns) is objects
      assert provider.publication_view(objects) is objects
      assert provider.camera_extended.last_groups == ((1_000_001, 1_000_002),)
    assert provider.camera_extended.representatives[0].age == scans

  @pytest.mark.parametrize('split_after', (1, 2, 5, 50))
  def test_split_reappearance_has_no_camera_induced_observation_gap(self, split_after):
    provider = BoschRadarProvider(1, camera_extended_mode=BOSCH_CAMERA_EXTENDED_ACTIVE)
    seen = {1_000_001: [], 1_000_002: []}
    for i in range(split_after+3):
      ns = 1_000_000_000 + i * 100_000_000
      objects = (physical(1_000_001, 15., ns=ns), physical(1_000_002, 19., ns=ns, age=i+1))
      mapping = {o.physical_track_id: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1) for o in objects} if i < split_after else {}
      TestBoschCameraE2Overlay.statuses(provider.camera_extended, mapping)
      assert provider.camera_extended.update(ns, objects, 10.) is objects
      for obj in provider.publication_view(objects):
        seen[obj.physical_track_id].append(ns)
    assert seen[1_000_001] == seen[1_000_002]
    assert len(seen[1_000_002]) == split_after+3
    assert provider.camera_extended.histories == {}

  @pytest.mark.parametrize(('near_y', 'far_y', 'near_v', 'far_v'), (
    (-2.5625, -1.96875, -6., -5.75), (1.09375, -.09375, 2., 2.), (0., 0., 0., 0.)))
  def test_geometry_and_identical_lateral_values_do_not_authorize_pid_collapse(self, near_y, far_y, near_v, far_v):
    provider = BoschRadarProvider(1, camera_extended_mode=BOSCH_CAMERA_EXTENDED_ACTIVE)
    objects = (physical(1_000_001, 19.5, near_y, near_v), physical(1_000_002, 24., far_y, far_v))
    TestBoschCameraPublicationSafety.group(provider.camera_extended, objects, 1_000_000_000)
    assert provider.camera_extended.last_groups
    assert provider.publication_view(objects) is objects

  def test_new_member_replaces_group_without_hiding_its_first_observation(self):
    provider = BoschRadarProvider(1, camera_extended_mode=BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(8):
      ns = 1_000_000_000 + i * 100_000_000
      other = 1_000_002 if i < 6 else 1_000_003
      objects = (physical(1_000_001, 15., ns=ns), physical(other, 19., ns=ns, age=1 if i==6 else 4))
      TestBoschCameraPublicationSafety.group(provider.camera_extended, objects, ns)
      assert provider.publication_view(objects) is objects
      assert tuple(provider.camera_extended.histories) == ((1_000_001, other),)
    assert provider.camera_extended.max_state_count == 1


class TestBoschActiveTestPublication:
  @staticmethod
  def scan(provider, ns, objects=None, mapping=None):
    objects = objects or (physical(1_000_001, 15., ns=ns), physical(1_000_002, 19., ns=ns))
    ext = provider.camera_extended
    if ext.camera is not None:
      TestBoschCameraE2Overlay.statuses(ext, mapping if mapping is not None else {
        o.physical_track_id: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1) for o in objects})
    assert ext.update(ns, objects, 10.) is objects
    return objects, provider.publication_view(objects)

  @pytest.mark.parametrize('mode', (BOSCH_CAMERA_EXTENDED_OFF, BOSCH_CAMERA_EXTENDED_SHADOW,
                                   BOSCH_CAMERA_EXTENDED_ACTIVE, BOSCH_CAMERA_EXTENDED_ACTIVE_TEST))
  def test_two_completed_intervals_only_test_mode_suppresses(self, mode):
    p = BoschRadarProvider(1, camera_extended_mode=mode)
    for i, ns in enumerate((1_000_000_000, 1_099_123_456, 1_198_765_432, 1_299_000_000)):
      objects, view = self.scan(p, ns)
      assert len(view) == (1 if mode == BOSCH_CAMERA_EXTENDED_ACTIVE_TEST and i >= 2 else 2)
      if i < 2 or mode != BOSCH_CAMERA_EXTENDED_ACTIVE_TEST:
        assert view is objects
    if mode == BOSCH_CAMERA_EXTENDED_ACTIVE_TEST:
      assert p.camera_extended.histories[(1_000_001, 1_000_002)].stable_intervals == 2

  @pytest.mark.parametrize('failure', ('tuple', 'missing', 'p2', 'ambiguous', 'class', 'episode',
                                      'g0', 'conflict', 'stale_member', 'stale_camera', 'future_camera',
                                      'motion_jump', 'gap', 'duplicate_ns'))
  def test_current_failure_resets_maturity_and_immediately_opens_publication(self, failure):
    p = BoschRadarProvider(1, camera_extended_mode=BOSCH_CAMERA_EXTENDED_ACTIVE_TEST)
    for i in range(3):
      _, view = self.scan(p, 1_000_000_000 + i * 100_000_000)
    assert len(view) == 1
    ns = 1_300_000_000
    if failure == 'gap':
      ns = 1_360_000_001
    elif failure == 'duplicate_ns':
      ns = 1_200_000_000
    objects = (physical(1_000_001, 15., ns=ns), physical(1_000_002, 19., ns=ns))
    mapping = {o.physical_track_id: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1) for o in objects}
    if failure == 'tuple':
      objects = (objects[0], physical(1_000_003, 19., ns=ns))
      mapping[1_000_003] = (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1)
    elif failure == 'missing':
      objects = (objects[0],)
    elif failure == 'p2':
      del mapping[1_000_002]
    elif failure == 'ambiguous':
      mapping[1_000_002] = (BOSCH_CAMERA_ASSOC_AMBIGUOUS, -1, -1)
    elif failure == 'class':
      mapping[1_000_002] = (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 2)
    elif failure == 'episode':
      mapping = {p: (BOSCH_CAMERA_ASSOC_ASSIGNED, 8, 1) for p in mapping}
    elif failure == 'g0':
      objects = (objects[0], physical(1_000_002, 19., 2., ns=ns))
    elif failure == 'conflict':
      objects = (replace(objects[0], v_rel=-10.), replace(objects[1], v_rel=-8.5))
    elif failure == 'stale_member':
      objects = (objects[0], replace(objects[1], timestamp_ns=ns - 1))
    elif failure == 'motion_jump':
      objects = tuple(replace(o, d_rel=o.d_rel + 1.) for o in objects)
    ext = p.camera_extended
    TestBoschCameraE2Overlay.statuses(ext, mapping)
    if failure in ('stale_camera', 'future_camera'):
      cam_ns = ns - 160_000_001 if failure == 'stale_camera' else ns + 1
      ext.camera.snapshot = lambda _: ([BoschCameraObject()], 1, 0, cam_ns)
    ext.update(ns, objects, 10.)
    assert p.publication_view(objects) is objects
    assert not ext.mature_groups
    assert ext.last_maturity_resets == 1
    assert all(s.stable_intervals == 0 for s in ext.histories.values())

  def test_missing_qualified_representative_or_member_is_fail_open(self):
    p = BoschRadarProvider(1, camera_extended_mode=BOSCH_CAMERA_EXTENDED_ACTIVE_TEST)
    for i in range(3):
      objects, _ = self.scan(p, 1_000_000_000 + i * 100_000_000)
    for qualified in ((objects[0],), (objects[1],)):
      assert p.publication_view(qualified) is qualified
    stale = tuple(replace(o, timestamp_ns=1_100_000_000) for o in objects)
    assert p.publication_view(stale) is stale

  def test_real_decoder_m2_and_native_boundary_keep_scc_and_alias_reservation(self, monkeypatch):
    import opendbc.car.hyundai.radar_interface as module
    from opendbc.car import structs
    def baseline(*args):
      result = structs.RadarData.new_message()
      result.points = [dict(trackId=0, dRel=80., vRel=0., radarSource='scc', measured=True)]
      return result
    monkeypatch.setattr(module.RadarInterfaceBase, 'update_carrot', baseline)
    messages = []
    monkeypatch.setattr(module.carlog, 'info', messages.append)
    interfaces = []
    for mode in (BOSCH_CAMERA_EXTENDED_OFF, BOSCH_CAMERA_EXTENDED_ACTIVE_TEST):
      ri = RadarInterface.__new__(RadarInterface)
      ri.bosch = BoschRadarProvider(1, qualification=False, camera_extended_mode=mode)
      ri.v_ego = 10.
      interfaces.append(ri)
    visible = []
    for i in range(9):
      ns = 1_000_000_000 + i * 100_000_000
      objects = (physical(1_000_001, 15., ns=ns, age=i+1), physical(1_010_690, 19., ns=ns, age=i+1))
      outputs = []
      for ri in interfaces:
        p = ri.bosch
        if p.camera_extended.camera is not None:
          # camera 공백으로 split/reappearance를 유발한 뒤 maturity를 다시 시작한다.
          feed(p.camera_extended.camera, ns, i, [(7, 20., 0., 0., 1.7, 1, 0, -.1, .1)] if i != 5 else [])
        p.camera_extended.update(ns, objects, 10.)
        p.tracker.group_manager.states = {o.physical_track_id: None for o in objects}
        p.last_scan_timestamp_ns = ri._bosch_now_ns = ns
        ri._bosch_objects = objects
        outputs.append(ri.update_carrot(10., 0., ns * 1e-9, []))
      a, b = (ri.bosch.publication_aliases for ri in interfaces)
      assert a.physical_to_alias == b.physical_to_alias
      assert a.last_published_ns == b.last_published_ns
      assert list(a.free_aliases) == list(b.free_aliases)
      target = a.physical_to_alias[1_010_690]
      visible.append(any(point.trackId == target for point in outputs[1].points))
      for result in outputs:
        assert result.points[0].to_dict() == baseline().points[0].to_dict()
      assert target not in b.free_aliases
    assert visible == [True, True, False, False, False, True, True, True, False]
    assert messages and all('mode=ACTIVE_TEST' in line and 'rep_pid=' in line for line in messages)
    assert sum('suppressed_pids=1010690' in line for line in messages) == 4
    perf = interfaces[1].bosch.perf_message()
    assert 'camera_ext_test_scans=9' in perf and 'camera_ext_suppressed_points=4' in perf
    assert 'camera_ext_maturity_resets=1' in perf

  def test_route254_original_policy_keeps_24m_lateral_member(self):
    p = BoschRadarProvider(1, camera_extended_mode=BOSCH_CAMERA_EXTENDED_ACTIVE_TEST)
    for i in range(3):
      ns = 1_000_000_000 + i * 100_000_000
      far = replace(physical(1_001_303, 24. - .575 * i, -1.96875, -5.75, ns), oem_selected=True)
      near = physical(1_001_321, 19.5 - .6 * i, -2.5625, -6., ns)
      _, view = self.scan(p, ns, (far, near))
    assert view == (far,)
    assert p.camera_extended.representatives[0].representative_pid == far.physical_track_id


class TestBoschTruckAwareP2:
  IDS = (1_004_581, 1_004_624)

  @staticmethod
  def statuses(grouping, ns, mapping, *, camera_id=203, episode=186, width=2.45,
               camera_d=24., camera_y=0., camera_v=0., camera_ns=None):
    grouping._associate = lambda obj, *_: mapping.get(obj.physical_track_id, (BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1))
    camera = BoschCameraObject(camera_id, episode, camera_d, camera_y, camera_v, width, 6, .2, -.2)
    grouping.camera.snapshot = lambda _: ([camera], 1, 0, ns if camera_ns is None else camera_ns)

  @classmethod
  def objects(cls, ns, *, far_d=28., far_y=0., far_v=0., far_pid=None):
    return (physical(cls.IDS[0], 20., 0., 0., ns),
            physical(cls.IDS[1] if far_pid is None else far_pid, far_d, far_y, far_v, ns))

  @classmethod
  def assigned(cls, episode=186, class_code=6, far_pid=None):
    return {cls.IDS[0]: (BOSCH_CAMERA_ASSOC_ASSIGNED, episode, class_code),
            cls.IDS[1] if far_pid is None else far_pid: (BOSCH_CAMERA_ASSOC_ASSIGNED, episode, class_code)}

  def test_class1_baseline_p2_is_still_immediate(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = 1_000_000_000
    objects = self.objects(ns)
    self.statuses(grouping, ns, self.assigned(class_code=1))
    assert grouping.update(ns, objects, 10.) is objects
    assert grouping.last_groups == (self.IDS,)
    assert grouping.truck_pair_histories == {}

  def test_class6_does_not_open_p2_immediately(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = 1_000_000_000
    self.statuses(grouping, ns, self.assigned())
    grouping.update(ns, self.objects(ns), 10.)
    assert grouping.last_groups == ()
    assert grouping.truck_pair_histories[self.IDS].confirmations == 1

  def test_same_episode_class6_and_current_g0_still_require_history(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(9):
      ns = 1_000_000_000 + i * 100_000_000
      self.statuses(grouping, ns, self.assigned())
      grouping.update(ns, self.objects(ns), 10.)
      assert grouping.last_groups == ()

  def test_stable_large_vehicle_opens_only_on_tenth_confirmation(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(10):
      ns = 1_000_000_000 + i * 100_000_000
      self.statuses(grouping, ns, self.assigned())
      grouping.update(ns, self.objects(ns), 10.)
    assert grouping.last_groups == (self.IDS,)
    assert grouping.truck_pair_histories[self.IDS].confirmations == 10

  @pytest.mark.parametrize(('reset', 'kwargs'), (
    ('episode', {'episode': 187}),
    ('camera_id', {'camera_id': 204}),
    ('camera_d', {'camera_d': 25.}),
    ('camera_y', {'camera_y': .25}),
    ('camera_v', {'camera_v': 1.}),
    ('camera_width', {'width': 2.60}),
  ))
  def test_camera_identity_or_motion_discontinuity_resets(self, reset, kwargs):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(9):
      ns = 1_000_000_000 + i * 100_000_000
      self.statuses(grouping, ns, self.assigned())
      grouping.update(ns, self.objects(ns), 10.)
    ns += 100_000_000
    mapping = self.assigned(episode=kwargs.get('episode', 186))
    self.statuses(grouping, ns, mapping, **kwargs)
    grouping.update(ns, self.objects(ns), 10.)
    assert grouping.last_groups == (), reset
    assert grouping.truck_pair_histories[self.IDS].confirmations == 1

  @pytest.mark.parametrize(('name', 'objects'), (
    ('dd', lambda self, ns: self.objects(ns, far_d=29.)),
    ('dy', lambda self, ns: self.objects(ns, far_y=.75)),
    ('dv', lambda self, ns: self.objects(ns, far_v=.5)),
  ))
  def test_radar_geometry_discontinuity_resets(self, name, objects):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(9):
      ns = 1_000_000_000 + i * 100_000_000
      self.statuses(grouping, ns, self.assigned())
      grouping.update(ns, self.objects(ns, far_d=28.), 10.)
    ns += 100_000_000
    self.statuses(grouping, ns, self.assigned())
    grouping.update(ns, objects(self, ns), 10.)
    assert grouping.last_groups == (), name
    assert grouping.truck_pair_histories[self.IDS].confirmations == 1

  def test_member_pid_change_starts_a_new_pair_at_one(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(9):
      ns = 1_000_000_000 + i * 100_000_000
      self.statuses(grouping, ns, self.assigned())
      grouping.update(ns, self.objects(ns), 10.)
    ns += 100_000_000
    new_pid = 1_004_625
    self.statuses(grouping, ns, self.assigned(far_pid=new_pid))
    grouping.update(ns, self.objects(ns, far_pid=new_pid), 10.)
    assert tuple(grouping.truck_pair_histories) == ((self.IDS[0], new_pid),)
    assert next(iter(grouping.truck_pair_histories.values())).confirmations == 1

  @pytest.mark.parametrize('failure', ('stale', 'ambiguous', 'class_change'))
  def test_stale_ambiguity_or_class_change_clears_pair_state(self, failure):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(9):
      ns = 1_000_000_000 + i * 100_000_000
      self.statuses(grouping, ns, self.assigned())
      grouping.update(ns, self.objects(ns), 10.)
    ns += 100_000_000
    mapping = self.assigned()
    camera_ns = None
    if failure == 'stale':
      camera_ns = ns - 160_000_001
    elif failure == 'ambiguous':
      mapping[self.IDS[1]] = (BOSCH_CAMERA_ASSOC_AMBIGUOUS, -1, -1)
    else:
      mapping[self.IDS[1]] = (BOSCH_CAMERA_ASSOC_ASSIGNED, 186, 2)
    self.statuses(grouping, ns, mapping, camera_ns=camera_ns)
    grouping.update(ns, self.objects(ns), 10.)
    assert grouping.last_groups == ()
    assert grouping.truck_pair_histories == {}

  def test_two_distinct_convoy_members_with_different_episodes_never_group(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    mapping = {self.IDS[0]: (BOSCH_CAMERA_ASSOC_ASSIGNED, 186, 6),
               self.IDS[1]: (BOSCH_CAMERA_ASSOC_ASSIGNED, 187, 6)}
    for i in range(20):
      ns = 1_000_000_000 + i * 100_000_000
      self.statuses(grouping, ns, mapping)
      grouping.update(ns, self.objects(ns), 10.)
      assert grouping.last_groups == ()

  def test_adjacent_vehicle_pair_fails_absolute_lateral_gate(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(20):
      ns = 1_000_000_000 + i * 100_000_000
      self.statuses(grouping, ns, self.assigned())
      grouping.update(ns, self.objects(ns, far_y=1.), 10.)
    assert grouping.last_groups == ()

  def test_cut_in_motion_change_resets_before_authorization(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(20):
      ns = 1_000_000_000 + i * 100_000_000
      self.statuses(grouping, ns, self.assigned())
      grouping.update(ns, self.objects(ns, far_y=.75 if i % 2 else 0.), 10.)
      assert grouping.last_groups == ()

  def test_narrow_class6_phantom_never_enters_pair_state(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(20):
      ns = 1_000_000_000 + i * 100_000_000
      self.statuses(grouping, ns, self.assigned(), width=2.35)
      grouping.update(ns, self.objects(ns), 10.)
    assert grouping.last_groups == ()
    assert grouping.truck_pair_histories == {}

  def test_class1_box_truck_m2_publication_sequence_is_exact(self):
    provider = BoschRadarProvider(1, camera_extended_mode=BOSCH_CAMERA_EXTENDED_ACTIVE_TEST)
    visible = []
    for i in range(6):
      ns = 1_000_000_000 + i * 100_000_000
      objects = self.objects(ns)
      self.statuses(provider.camera_extended, ns, self.assigned(class_code=1))
      provider.camera_extended.update(ns, objects, 10.)
      visible.append(len(provider.publication_view(objects)))
    assert visible == [2, 2, 1, 1, 1, 1]

  @pytest.mark.parametrize(('field', 'value', 'accepted'), (
    ('width', 2.40, True), ('width', 2.35, False),
    ('dd', 5.50, True), ('dd', 5.49, False), ('dd', 9.00, True), ('dd', 9.01, False),
    ('dy', .875, True), ('dy', .876, False), ('dv', .50, True), ('dv', .501, False),
  ))
  def test_absolute_truck_gate_boundaries(self, field, value, accepted):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(10):
      ns = 1_000_000_000 + i * 100_000_000
      width = value if field == 'width' else 2.45
      far_d = 20. + value if field == 'dd' else 28.
      far_y = value if field == 'dy' else 0.
      far_v = value if field == 'dv' else 0.
      self.statuses(grouping, ns, self.assigned(), width=width)
      grouping.update(ns, self.objects(ns, far_d=far_d, far_y=far_y, far_v=far_v), 10.)
    assert bool(grouping.last_groups) is accepted

  def test_pair_state_is_bounded_and_retires_on_next_scan(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = 1_000_000_000
    objects = tuple(physical(1_000_000 + i, 10. + i, ns=ns) for i in range(24))
    mapping = {obj.physical_track_id: (BOSCH_CAMERA_ASSOC_ASSIGNED, 186, 6) for obj in objects}
    self.statuses(grouping, ns, mapping)
    grouping.update(ns, objects, 10.)
    assert len(grouping.truck_pair_histories) == 16
    ns += 100_000_000
    self.statuses(grouping, ns, {})
    grouping.update(ns, (), 10.)
    assert grouping.truck_pair_histories == {}


class TestBoschLargeVehicleA0Recovery:
  IDS = (1_004_581, 1_004_624)

  @classmethod
  def objects(cls, ns, *, far_pid=None, far_d=26., far_y=-4.5, far_v=0.):
    return (physical(cls.IDS[0], 20., -4., 0., ns),
            physical(cls.IDS[1] if far_pid is None else far_pid, far_d, far_y, far_v, ns))

  @staticmethod
  def camera(*, camera_id=203, episode=186, camera_d=23., camera_y=4.25,
             camera_v=0., width=2.45, class_code=6, left=.205, right=.197):
    return BoschCameraObject(camera_id, episode, camera_d, camera_y, camera_v,
                             width, class_code, left, right)

  @classmethod
  def configure(cls, grouping, ns, mapping, *, camera_ns=None, cameras=None, **camera_kwargs):
    grouping._associate = lambda obj, *_: mapping.get(obj.physical_track_id, (BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1))
    values = [cls.camera(**camera_kwargs)] if cameras is None else cameras
    grouping.camera.snapshot = lambda _: (values, len(values), 0, ns if camera_ns is None else camera_ns)

  @classmethod
  def assigned(cls, *, episode=186, class_code=6, far_pid=None):
    return {cls.IDS[0]: (BOSCH_CAMERA_ASSOC_ASSIGNED, episode, class_code),
            cls.IDS[1] if far_pid is None else far_pid: (BOSCH_CAMERA_ASSOC_ASSIGNED, episode, class_code)}

  @classmethod
  def one_sided(cls, *, episode=186, class_code=6, missing=1, far_pid=None, missing_status=BOSCH_CAMERA_ASSOC_UNRESOLVED):
    ids = (cls.IDS[0], cls.IDS[1] if far_pid is None else far_pid)
    mapping = {ids[0]: (BOSCH_CAMERA_ASSOC_ASSIGNED, episode, class_code),
               ids[1]: (BOSCH_CAMERA_ASSOC_ASSIGNED, episode, class_code)}
    mapping[ids[missing]] = (missing_status, -1, -1)
    return mapping

  @classmethod
  def seed(cls, grouping):
    for i in range(10):
      ns = 1_000_000_000 + i * 100_000_000
      cls.configure(grouping, ns, cls.assigned())
      grouping.update(ns, cls.objects(ns), 10.)
    return ns

  def test_frozen_a0_verdict_is_exact_and_near_miss_remains_unresolved(self):
    camera = self.camera()
    near, far = self.objects(1_000_000_000)
    assert BoschCameraExtendedGrouping._associate(near, [camera], 1) == (BOSCH_CAMERA_ASSOC_ASSIGNED, 186, 6)
    assert BoschCameraExtendedGrouping._associate(far, [camera], 1) == (BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1)

  def test_unseeded_one_side_a0_never_recovers(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = 1_000_000_000
    self.configure(grouping, ns, self.one_sided())
    grouping.update(ns, self.objects(ns), 10.)
    assert grouping.last_groups == () and grouping.truck_pair_histories == {}

  def test_seeded_large_pair_recovers_one_side_transient_unresolved(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = self.seed(grouping) + 100_000_000
    self.configure(grouping, ns, self.one_sided())
    grouping.update(ns, self.objects(ns), 10.)
    assert grouping.last_associations[self.IDS[1]][0] == BOSCH_CAMERA_ASSOC_UNRESOLVED
    assert grouping.last_groups == (self.IDS,)
    assert grouping.last_truck_recovery_count == 1

  def test_both_unresolved_never_recovers(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = self.seed(grouping) + 100_000_000
    self.configure(grouping, ns, {})
    grouping.update(ns, self.objects(ns), 10.)
    assert grouping.last_groups == () and grouping.truck_pair_histories == {}

  def test_camera_id_change_resets_recovery_state(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = self.seed(grouping) + 100_000_000
    self.configure(grouping, ns, self.one_sided(), camera_id=204)
    grouping.update(ns, self.objects(ns), 10.)
    assert grouping.truck_pair_histories == {} and grouping.last_truck_recovery_count == 0

  def test_episode_change_resets_recovery_state(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = self.seed(grouping) + 100_000_000
    self.configure(grouping, ns, self.one_sided(episode=187), episode=187)
    grouping.update(ns, self.objects(ns), 10.)
    assert grouping.truck_pair_histories == {} and grouping.last_groups == ()

  def test_class_change_resets_recovery_state(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = self.seed(grouping) + 100_000_000
    self.configure(grouping, ns, self.one_sided(class_code=2), class_code=2)
    grouping.update(ns, self.objects(ns), 10.)
    assert grouping.truck_pair_histories == {} and grouping.last_groups == ()

  def test_stale_camera_resets_recovery_state(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = self.seed(grouping) + 100_000_000
    self.configure(grouping, ns, self.one_sided(), camera_ns=ns - 160_000_001)
    grouping.update(ns, self.objects(ns), 10.)
    assert grouping.truck_pair_histories == {} and grouping.last_truck_recovery_count == 0

  def test_ambiguous_missing_side_never_recovers(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = self.seed(grouping) + 100_000_000
    self.configure(grouping, ns, self.one_sided(missing_status=BOSCH_CAMERA_ASSOC_AMBIGUOUS))
    grouping.update(ns, self.objects(ns), 10.)
    assert grouping.truck_pair_histories == {} and grouping.last_truck_recovery_count == 0

  def test_pair_g0_failure_resets_recovery_state(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = self.seed(grouping) + 100_000_000
    self.configure(grouping, ns, self.one_sided())
    grouping.update(ns, self.objects(ns, far_d=32.1), 10.)
    assert grouping.last_groups == () and grouping.truck_pair_histories == {}

  @pytest.mark.parametrize(('field', 'kwargs'), (
    ('dd', {'far_d': 27.}), ('dy', {'far_y': -4.}), ('dv', {'far_v': .5}),
  ))
  def test_radar_dd_dy_dv_discontinuity_resets_recovery(self, field, kwargs):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = self.seed(grouping) + 100_000_000
    self.configure(grouping, ns, self.one_sided())
    grouping.update(ns, self.objects(ns, **kwargs), 10.)
    assert grouping.truck_pair_histories == {}, field

  @pytest.mark.parametrize(('field', 'kwargs'), (
    ('d', {'camera_d': 24.}), ('y', {'camera_y': 4.5}),
    ('v', {'camera_v': 1.}), ('width', {'width': 2.60}),
  ))
  def test_camera_motion_or_width_jump_resets_recovery(self, field, kwargs):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = self.seed(grouping) + 100_000_000
    self.configure(grouping, ns, self.one_sided(), **kwargs)
    grouping.update(ns, self.objects(ns), 10.)
    assert grouping.truck_pair_histories == {}, field

  def test_member_pid_change_cannot_inherit_seed(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = self.seed(grouping) + 100_000_000
    new_pid = self.IDS[1] + 1
    self.configure(grouping, ns, self.one_sided(far_pid=new_pid), cameras=[self.camera()])
    grouping.update(ns, self.objects(ns, far_pid=new_pid), 10.)
    assert grouping.truck_pair_histories == {}

  def test_missing_representative_identity_cannot_recover(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = self.seed(grouping) + 100_000_000
    grouping.representatives.clear()
    self.configure(grouping, ns, self.one_sided())
    grouping.update(ns, self.objects(ns), 10.)
    assert grouping.truck_pair_histories == {} and grouping.last_truck_recovery_count == 0

  def test_recovery_hold_timeout_retires_pair_state(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = self.seed(grouping)
    for age in range(1, BOSCH_TRUCK_A0_RECOVERY_HOLD_SCANS + 1):
      ns += 100_000_000
      self.configure(grouping, ns, self.one_sided())
      grouping.update(ns, self.objects(ns), 10.)
      assert grouping.truck_pair_histories[self.IDS].recovery_age == age
    ns += 100_000_000
    self.configure(grouping, ns, self.one_sided())
    grouping.update(ns, self.objects(ns), 10.)
    assert grouping.truck_pair_histories == {} and grouping.last_truck_recovery_count == 0

  def test_different_vehicle_pair_without_seed_is_rejected(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(20):
      ns = 1_000_000_000 + i * 100_000_000
      mapping = {self.IDS[0]: (BOSCH_CAMERA_ASSOC_ASSIGNED, 186, 6),
                 self.IDS[1]: (BOSCH_CAMERA_ASSOC_ASSIGNED, 187, 6)}
      self.configure(grouping, ns, mapping)
      grouping.update(ns, self.objects(ns), 10.)
    assert grouping.last_groups == () and grouping.truck_pair_histories == {}

  def test_adjacent_or_cut_in_geometry_fails_open(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE_TEST)
    ns = self.seed(grouping) + 100_000_000
    self.configure(grouping, ns, self.one_sided())
    grouping.update(ns, self.objects(ns, far_y=-2.5), 10.)
    # 기존 E2 coast는 허용하지만 새 strict recovery/M2 suppression은 열지 않는다.
    assert grouping.truck_pair_histories == {} and grouping.last_truck_recovery_count == 0
    assert grouping.mature_groups == ()

  def test_narrow_phantom_pair_cannot_form_recovery_seed(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(12):
      ns = 1_000_000_000 + i * 100_000_000
      self.configure(grouping, ns, self.assigned(), width=2.35)
      grouping.update(ns, self.objects(ns), 10.)
    assert grouping.last_groups == () and grouping.truck_pair_histories == {}

  def test_class1_path_is_immediate_and_has_no_recovery_state(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = 1_000_000_000
    self.configure(grouping, ns, self.assigned(class_code=1), class_code=1)
    grouping.update(ns, self.objects(ns), 10.)
    assert grouping.last_groups == (self.IDS,) and grouping.truck_pair_histories == {}

  def test_existing_truck_n10_is_exact_without_recovery(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(10):
      ns = 1_000_000_000 + i * 100_000_000
      self.configure(grouping, ns, self.assigned())
      grouping.update(ns, self.objects(ns), 10.)
      assert bool(grouping.last_groups) is (i == 9)
      assert grouping.last_truck_recovery_count == 0

  def test_recovery_state_is_bounded_and_cleans_up(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = self.seed(grouping) + 100_000_000
    self.configure(grouping, ns, self.one_sided())
    grouping.update(ns, self.objects(ns), 10.)
    assert len(grouping.truck_pair_histories) == 1
    ns += 100_000_000
    self.configure(grouping, ns, {})
    grouping.update(ns, (), 10.)
    assert grouping.truck_pair_histories == {}
