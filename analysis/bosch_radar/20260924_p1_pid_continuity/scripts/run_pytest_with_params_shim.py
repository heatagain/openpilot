#!/usr/bin/env python3
"""Run pytest on Windows with the minimal Params shim used by Bosch studies."""

from __future__ import annotations

import sys
from pathlib import Path
from types import ModuleType

import pytest


class Params:
  def __init__(self, *_args, **_kwargs):
    pass

  def get(self, *_args, **_kwargs):
    return None

  def get_bool(self, *_args, **_kwargs):
    return False

  def put(self, *_args, **_kwargs):
    return None

  def put_bool(self, *_args, **_kwargs):
    return None

  def remove(self, *_args, **_kwargs):
    return None


def main() -> int:
  repo = Path(__file__).resolve().parents[4]
  sys.path.insert(0, str(repo))
  sys.path.insert(0, str(repo / "opendbc_repo"))
  shim = ModuleType("openpilot.common.params")
  shim.Params = Params
  sys.modules["openpilot.common.params"] = shim
  base_args = [
    "-o",
    "addopts=",
    "--confcutdir",
    str(repo / "opendbc_repo"),
  ]
  default_args = [
    str(repo / "opendbc_repo/opendbc/car/hyundai/tests/test_radar.py"),
    "-k",
    "not group3",
    "-q",
  ]
  return pytest.main(base_args + (sys.argv[1:] or default_args))  # noqa: TID251 - the shim must be installed in-process before collection


if __name__ == "__main__":
  raise SystemExit(main())
