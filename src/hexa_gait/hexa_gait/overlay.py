"""Compatibility shim — the tuning overlay moved to ``hexa_common``.

The implementation now lives in :mod:`hexa_common.overlay`; this re-export
keeps ``hexa_gait``'s public API and in-tree importers (``gait_node``)
valid after the extraction to the leaf library.
"""

from hexa_common.overlay import deep_merge, load_tuning_block

__all__ = ["deep_merge", "load_tuning_block"]
