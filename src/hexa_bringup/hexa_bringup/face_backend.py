"""Which face node the robot launch starts: one of three values, never both."""
import yaml

NONE = "none"
HEXA_DISPLAY = "hexa_display"
EYES_CPP = "eyes_cpp"
BACKENDS = (NONE, HEXA_DISPLAY, EYES_CPP)


def parse_backend(value) -> str:
    """The backend named by `value`, or ValueError listing the valid ones.

    Missing (None) means `none`. Anything else must be an exact name: a typo
    must stop the launch rather than silently pick a face (or none).
    """
    if value is None:
        return NONE
    if isinstance(value, str) and value in BACKENDS:
        return value
    raise ValueError(
        f"face.backend: unknown backend {value!r}; valid: {', '.join(BACKENDS)}"
    )


def load_backend(path: str) -> str:
    """Read face.yaml's `face.backend` and validate it."""
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    face = data.get("face") or {}
    if not isinstance(face, dict):
        raise ValueError("face: expected a mapping with a `backend` key")
    return parse_backend(face.get("backend"))


def face_package(backend: str):
    """(package, executable) of the one node to start, or None for `none`."""
    return {
        NONE: None,
        HEXA_DISPLAY: ("hexa_display", "display_node"),
        EYES_CPP: ("hexa_eyes", "eyes_node"),
    }[parse_backend(backend)]
