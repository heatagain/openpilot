import pytest
import math
from types import SimpleNamespace

from openpilot.selfdrive.carrot.radar import (
  effective_radar_track_mode,
)
from openpilot.selfdrive.carrot.radar_motion.coordinates import device_yaw_to_radar
from openpilot.selfdrive.carrot.radar_motion.predictor import project_to_model_path, radar_target_velocity_in_ego_frame
from openpilot.selfdrive.carrot.radar_motion.controller import _model_path
from openpilot.selfdrive.carrot.radar.tools.radar_validation_replay import _yaw_metadata


@pytest.mark.parametrize("side", (-1.0, 1.0))
def test_device_and_steering_yaw_match_radar_rotation_convention(side):
  yaw = side * 0.025
  pose = SimpleNamespace(angularVelocityDevice=SimpleNamespace(valid=True, z=-yaw), inputsOK=True, sensorsOK=True)
  measured, estimated, source = _yaw_metadata(10.0, 0.0, pose, 0.01, 14.0, 2.8)
  steering = math.degrees(math.atan(yaw * 2.8 / 10.0)) * 14.0
  fallback, _, _ = _yaw_metadata(10.0, steering, None, 1.0, 14.0, 2.8)
  assert measured == pytest.approx(yaw)
  assert fallback == pytest.approx(yaw)
  assert not estimated and source == "livePose"
  # A parallel target's apparent lateral motion comes entirely from ego
  # rotation. Correcting it must produce zero physical lateral velocity.
  _, lateral = radar_target_velocity_in_ego_frame(10.0, -yaw * 20.0, 20.0, side * 3.0, measured)
  assert lateral == pytest.approx(0.0)
  _, entering = radar_target_velocity_in_ego_frame(10.0, -side * 0.4 - yaw * 20.0, 20.0, side * 3.0, measured)
  assert entering == pytest.approx(-side * 0.4)


@pytest.mark.parametrize("value", (math.nan, math.inf, -math.inf))
def test_invalid_device_yaw_is_not_used(value):
  assert device_yaw_to_radar(value) == 0.0


@pytest.mark.parametrize("side", (-1.0, 1.0))
def test_model_path_is_converted_once_inside_projection(side):
  model = SimpleNamespace(position=SimpleNamespace(x=(0.0, 20.0), y=(0.0, -side * 2.0)))
  path = _model_path(model)
  assert path[-1][1] == -side * 2.0
  assert project_to_model_path(path, 20.0, side * 2.0).d_path == pytest.approx(0.0)


@pytest.mark.parametrize("configured_mode", (-2, -1, 0, 1, 2, 3))
def test_hyundai_keeps_configured_radar_track_mode(configured_mode: int) -> None:
  assert effective_radar_track_mode(
    "hyundai", False, configured_mode,
  ) == configured_mode


@pytest.mark.parametrize(
  "brand", ("volkswagen", "honda", "toyota", "ford", "subaru"),
)
@pytest.mark.parametrize("configured_mode", (-2, -1, 0, 1, 2, 3))
def test_other_brands_ignore_option_and_use_front_radar(
  brand: str,
  configured_mode: int,
) -> None:
  assert effective_radar_track_mode(
    brand, False, configured_mode,
  ) == 1


@pytest.mark.parametrize("configured_mode", (-2, -1, 0, 1, 2, 3))
def test_other_brands_without_radar_use_vision(configured_mode: int) -> None:
  assert effective_radar_track_mode(
    "mazda", True, configured_mode,
  ) == -2


# --- Bosch RadarTracks liveTracks polling (candidate A') -------------------------

from openpilot.selfdrive.carrot.radar import radard_dpath
from openpilot.selfdrive.carrot.radar.radard_dpath import bosch_radar_tracks_active
from opendbc.car.hyundai.values import HyundaiExtFlags

BOSCH = int(HyundaiExtFlags.BOSCH_RADAR)


def _cp(brand="hyundai", ext_flags=BOSCH, radar_unavailable=False):
  return SimpleNamespace(brand=brand, extFlags=ext_flags, radarUnavailable=radar_unavailable)


@pytest.mark.parametrize("mode", (1, 2, 3))
def test_bosch_radar_tracks_active_for_hyundai_bosch(mode: int) -> None:
  assert bosch_radar_tracks_active(_cp(), mode)


@pytest.mark.parametrize(
  "cp, mode",
  (
    (_cp(ext_flags=0), 1),                      # Hyundai, but not the Bosch provider
    (_cp(), 0),                                 # Bosch hardware, RadarTracks disabled
    (_cp(), -1),
    (_cp(brand="volkswagen", ext_flags=0), 1),  # other brands
    (_cp(brand="toyota", ext_flags=0), 1),
    (_cp(brand="honda", ext_flags=0), 3),
  ),
)
def test_bosch_radar_tracks_inactive_elsewhere(cp, mode: int) -> None:
  assert not bosch_radar_tracks_active(cp, mode)


class _FakeSubMaster:
  """Scripted SubMaster: replays a list of `updated` dicts, then stops the loop."""

  class Stop(Exception):
    pass

  def __init__(self, services, poll=None, ignore_alive=None, ignore_valid=None):
    self.services = services
    self.poll = poll
    self.ignore_alive = ignore_alive
    self.ignore_valid = ignore_valid
    self.script: list[dict] = []
    self.updated: dict = {}
    self.data = {s: object() for s in services}
    self.logMonoTime = dict.fromkeys(services, 0)

  def update(self):
    if not self.script:
      raise _FakeSubMaster.Stop
    self.updated = self.script.pop(0)

  def __getitem__(self, s):
    return self.data[s]


class _SpyRadar:
  def __init__(self, CP):
    self.CP = CP
    self.updates = 0
    self.publishes = 0

  def update(self, sm, rr):
    self.updates += 1

  def publish(self, pm):
    self.publishes += 1


def _run_main(monkeypatch, cp, mode, script):
  """Run the real radard main() loop against fakes and return (submaster, radar)."""
  made = {}

  def make_sm(services, poll=None, ignore_alive=None, ignore_valid=None):
    sm = _FakeSubMaster(services, poll, ignore_alive, ignore_valid)
    sm.script = list(script)
    made["sm"] = sm
    return sm

  def make_radar(CP):
    made["radar"] = _SpyRadar(CP)
    return made["radar"]

  monkeypatch.setattr(radard_dpath, "config_realtime_process", lambda *a, **k: None)
  monkeypatch.setattr(radard_dpath.messaging, "SubMaster", make_sm)
  monkeypatch.setattr(radard_dpath.messaging, "PubMaster", lambda services: object())
  monkeypatch.setattr(radard_dpath.messaging, "log_from_bytes", lambda *a, **k: cp)
  monkeypatch.setattr(radard_dpath, "DPathRadarD", make_radar)
  monkeypatch.setattr(
    radard_dpath, "Params",
    lambda: SimpleNamespace(get=lambda *a, **k: b"", get_int=lambda _k: mode),
  )
  with pytest.raises(_FakeSubMaster.Stop):
    radard_dpath.main()
  return made["sm"], made["radar"]


@pytest.mark.parametrize("mode", (1, 2, 3))
def test_bosch_radar_tracks_also_polls_live_tracks(monkeypatch, mode: int) -> None:
  sm, _ = _run_main(monkeypatch, _cp(), mode, [])
  assert sm.poll == ["modelV2", "liveTracks"]
  # the rest of the subscription is untouched
  assert sm.services == ["modelV2", "carState", "liveTracks", "livePose"]
  assert sm.ignore_alive == ["livePose"]
  assert sm.ignore_valid == ["livePose"]


@pytest.mark.parametrize(
  "cp, mode",
  (
    (_cp(ext_flags=0), 1),
    (_cp(), 0),
    (_cp(brand="volkswagen", ext_flags=0), 1),
    (_cp(brand="toyota", ext_flags=0), 3),
    (_cp(brand="mazda", ext_flags=0, radar_unavailable=True), 1),
  ),
)
def test_non_bosch_cars_keep_the_single_model_poll(monkeypatch, cp, mode: int) -> None:
  sm, _ = _run_main(monkeypatch, cp, mode, [])
  assert sm.poll == "modelV2"


def test_live_tracks_only_wake_does_not_touch_radar_state(monkeypatch) -> None:
  script = [
    {"modelV2": False, "liveTracks": True},
    {"modelV2": False, "liveTracks": True},
  ]
  _, radar = _run_main(monkeypatch, _cp(), 1, script)
  assert radar.updates == 0
  assert radar.publishes == 0


def test_model_wake_updates_and_publishes_exactly_once(monkeypatch) -> None:
  script = [
    {"modelV2": True, "liveTracks": True},
    {"modelV2": False, "liveTracks": True},
    {"modelV2": True, "liveTracks": False},
    {"modelV2": False, "liveTracks": True},
  ]
  _, radar = _run_main(monkeypatch, _cp(), 1, script)
  assert radar.updates == 2
  assert radar.publishes == 2
