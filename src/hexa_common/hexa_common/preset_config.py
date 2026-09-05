"""Pure-Python reader for tuning.yaml's ``gait_node.presets`` block.

A **preset** is the bundle the operator selects as one thing. It has two halves
and they live in different files, because they are owned by different layers:

- the **physical** half — which legs it stands on, where those feet sit, and the
  stride and swing times the walk lays down on it — is here, in
  ``hexa_description``'s ``tuning.yaml``, which the engine loads;
- the **operator** half — label, gait rotation, entry gait — is in the two
  teleop configs, and is ``hexa_teleop.presets``' business.

The two are keyed by the same ids. This module is what lets the teleop side read
the physical half without duplicating it: a preset's **leg set** is declared
once, here, and ``hexa_teleop.presets.load_presets`` reads it back rather than
deriving it from the gaits in a rotation. That derivation is what the old
two-preset arrangement used, and it cannot tell two six-leg presets apart.

rclpy-free, like the rest of ``hexa_common``.
"""

from __future__ import annotations

from pathlib import Path
from typing import Mapping

import yaml

# The two leg sets, matching hexa_common.gait_catalog and the wire strings
# hexa::gait::leg_set_value emits.
LEG_SETS = ("hexapod", "quadruped")


def gait_params(tuning_yaml: str | Path) -> dict:
    """tuning.yaml's ``gait_node.ros__parameters`` block."""
    with Path(tuning_yaml).open() as f:
        return yaml.safe_load(f)["gait_node"]["ros__parameters"]


def preset_table(params: Mapping) -> dict[str, dict]:
    """The ``presets:`` list keyed by id, in declaration order.

    Declaration order is load-bearing: it is the order the firmware's baked
    ``kPresets`` table is indexed by and the order the C++ loader reads, so the
    loaded-vs-baked parity test can compare them position by position.
    """
    entries = params.get("presets")
    if not isinstance(entries, list) or not entries:
        raise ValueError("gait_node.presets must be a non-empty list")
    out: dict[str, dict] = {}
    for entry in entries:
        pid = str(entry["id"])
        if pid in out:
            raise ValueError(f"gait_node.presets: duplicate id {pid!r}")
        leg_set = str(entry["leg_set"])
        if leg_set not in LEG_SETS:
            raise ValueError(
                f"gait_node.presets.{pid}.leg_set = {leg_set!r} must be one of "
                f"{list(LEG_SETS)}"
            )
        out[pid] = dict(entry)
    return out


def default_preset_id(params: Mapping) -> str:
    """The preset the robot boots on. Must stand on all six legs."""
    pid = str(params["default_preset"])
    table = preset_table(params)
    if pid not in table:
        raise ValueError(
            f"gait_node.default_preset = {pid!r} names no preset; "
            f"have {sorted(table)}"
        )
    if table[pid]["leg_set"] != "hexapod":
        raise ValueError(
            f"gait_node.default_preset = {pid!r} stands on the "
            f"{table[pid]['leg_set']} leg set; the boot preset must stand on "
            f"all six, since the cold start is folded on six and it is what "
            f"FAULT recovers to"
        )
    return pid


def preset_leg_sets(tuning_yaml: str | Path) -> dict[str, str]:
    """Preset id -> leg set, for the teleop side's own preset registry."""
    return {
        pid: str(entry["leg_set"])
        for pid, entry in preset_table(gait_params(tuning_yaml)).items()
    }
