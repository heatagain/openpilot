from dataclasses import replace
from types import SimpleNamespace as NS

import pytest

from opendbc.car.hyundai.radar_bosch import BoschPhysicalObject, BoschRoadEdgePublicationFilter
from opendbc.car.hyundai.radar_interface import RadarInterface


T = 1_000_000_000


def model(stds=(0.5, 0.5), left=(-2.0, -2.0), right=(2.0, 2.0), xs=(0.0, 150.0)):
  return NS(roadEdges=[NS(x=xs, y=left), NS(x=xs, y=right)], roadEdgeStds=stds)


def point(pid=1, d=50.0, y=5.0, **kwargs):
  return BoschPhysicalObject(pid, T, (), 0, d, y, 0.0, False, False, 1, 'test', **kwargs)


def apply(objects, sample=None, **kwargs):
  f = BoschRoadEdgePublicationFilter()
  f.ingest_model(sample or model(), T)
  f.update(objects, T, **kwargs)
  return f, f.publication_view(objects, T)


def test_both_sides_margin_and_no_input_mutation():
  objects = (point(y=5), point(2, y=-5), point(3, y=4), point(4, y=-4), point(5, y=0))
  f, result = apply(objects)
  assert f.would_suppress == {1, 2}
  assert result == objects[2:]
  assert result[0] is objects[2]
  assert objects[0].y_rel == 5 and len(objects) == 5


@pytest.mark.parametrize('sample', [
  model(stds=(1.01, .1)), model(stds=(float('nan'), .1)), model(stds=(-.1, .1)),
  model(xs=(0., 0.)), model(left=(float('nan'), -2.)), model(left=(3., 3.)),
  model(left=(-2.,)), NS(roadEdges=[], roadEdgeStds=[]),
])
def test_bad_geometry_preserves_output(sample):
  objects = (point(),)
  _, result = apply(objects, sample)
  assert result is objects


def test_no_range_extrapolation():
  objects = (point(d=149), point(2, d=-2))
  _, result = apply(objects)
  assert result is objects


def test_camera_origin_offset_on_curved_edge():
  objects = (point(y=9.1), point(2, y=9.3))
  f, result = apply(objects, model(xs=(0., 100.), left=(-2., -12.), right=(2., 12.)))
  assert result == objects[:1]
  assert f.would_suppress == {2}


def test_independent_evidence_protected():
  objects = (replace(point(), oem_selected=True), replace(point(2), vision_supported=True), point(3), point(4), point(5))
  f, result = apply(objects, word0_pids={3}, camera_associations={4: (1, 12), 5: (0, -1)})
  assert f.would_suppress == {5}
  assert result == objects[:4]


def test_stale_future_missing_and_invalid_scan_fail_open():
  objects = (point(),)
  for model_ns, valid in [(T - 200_000_001, True), (T + 1, True), (T, False)]:
    f = BoschRoadEdgePublicationFilter()
    f.ingest_model(model(), model_ns)
    f.update(objects, T, valid=valid)
    assert f.publication_view(objects, T) is objects
  f = BoschRoadEdgePublicationFilter()
  f.update(objects, T)
  assert f.publication_view(objects, T) is objects


def test_latest_uncertain_model_does_not_use_older_good_model():
  f = BoschRoadEdgePublicationFilter()
  f.ingest_model(model(), T - 50_000_000)
  f.ingest_model(model(stds=(2., 2.)), T)
  objects = (point(),)
  f.update(objects, T)
  assert f.publication_view(objects, T) is objects


def test_future_model_cannot_influence_past_scan():
  f = BoschRoadEdgePublicationFilter()
  f.ingest_model(model(), T)
  f.ingest_model(model(left=(-20., -20.), right=(20., 20.)), T + 1)
  f.update((point(),), T)
  assert f.would_suppress == {1}


def test_no_stale_publication_or_reused_pid_suppression():
  objects = (point(),)
  f, _ = apply(objects)
  assert f.publication_view(objects, T + 200_000_001) is objects
  assert f.publication_view(objects, T - 1) is objects
  assert f.publication_view(objects, None) is objects
  newer = (replace(objects[0], timestamp_ns=T + 1),)
  assert f.publication_view(newer, T + 1) == newer
  f.invalidate_model()
  assert f.publication_view(objects, T) is objects


def test_new_scan_releases_old_decision_and_clock_reset_clears_context():
  f, _ = apply((point(),))
  inside = (point(y=0),)
  f.update(inside, T)
  assert f.publication_view(inside, T) is inside
  f.update((replace(point(), timestamp_ns=T-1),), T-1)
  assert not f.would_suppress and not f.model_edges


def test_disabled_keeps_all_objects():
  f = BoschRoadEdgePublicationFilter(enabled=False)
  f.ingest_model(model(), T)
  objects = (point(),)
  f.update(objects, T)
  assert f.publication_view(objects, T) is objects


def test_context_does_not_touch_non_bosch_interface():
  interface = NS(bosch=None)
  RadarInterface.set_bosch_context(interface, T, model=model(), model_ns=T)
  assert vars(interface) == {'bosch': None}
