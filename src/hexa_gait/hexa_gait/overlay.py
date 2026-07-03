"""Runtime-tuning overlay loader.

``hexa_description/config/tuning.yaml`` is a sparse overlay: each block
(``gait``, ``control``, …) overrides the same-named keys in the owning
package's base YAML, and anything absent falls through to the base
default. This module resolves that file and merges a single block over an
already-loaded base config.

Kept pure (no ``rclpy``): the node call sites pass the result of
``load_tuning_block(...)`` into the otherwise path-pure config loaders as
an *optional* argument, so unit tests that hand those loaders a temp YAML
stay unaffected (they simply don't pass an overlay).
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)


def deep_merge(base: dict, overlay: dict) -> dict:
    """Recursively merge ``overlay`` into ``base`` in place; overlay wins.

    Nested dicts merge key-by-key; every other value (scalars, lists) is
    replaced wholesale. Returns ``base`` for convenience.
    """
    for key, value in overlay.items():
        existing = base.get(key)
        if isinstance(existing, dict) and isinstance(value, dict):
            deep_merge(existing, value)
        else:
            base[key] = value
    return base


def load_tuning_block(domain: str) -> dict[str, Any]:
    """Return the ``domain`` block of the tuning overlay, or ``{}``.

    Resolves ``hexa_description/config/tuning.yaml`` via the ament share
    index. A missing package, missing file, empty file, or missing block
    all yield ``{}`` (no override) so the overlay is safe to omit — e.g.
    on a robot deploy that hasn't shipped one yet.
    """
    try:
        share = Path(get_package_share_directory("hexa_description"))
    except PackageNotFoundError:
        return {}
    path = share / "config" / "tuning.yaml"
    if not path.is_file():
        return {}
    with path.open() as f:
        data = yaml.safe_load(f) or {}
    block = data.get(domain, {})
    return block if isinstance(block, dict) else {}
