"""Compatibility shim — the animation-mode loader moved to ``hexa_common``.

The implementation now lives in :mod:`hexa_common.posture_config`; this
re-export keeps ``hexa_posture``'s public API (``hexa_posture.
load_animation_mode_animations``) and its consumers valid after the
extraction to the leaf library.
"""

from hexa_common.posture_config import load_animation_mode_animations

__all__ = ["load_animation_mode_animations"]
