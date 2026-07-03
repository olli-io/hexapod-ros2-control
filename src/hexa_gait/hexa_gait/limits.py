"""Compatibility shim — velocity caps moved to ``hexa_common``.

The implementation now lives in :mod:`hexa_common.limits`; this re-export
keeps ``hexa_gait``'s public API (``hexa_gait.VelocityCaps`` etc.) and any
in-tree importers valid after the extraction to the leaf library.
"""

from hexa_common.limits import (
    VelocityCaps,
    load_velocity_caps,
    scale_to_envelope,
)

__all__ = ["VelocityCaps", "load_velocity_caps", "scale_to_envelope"]
