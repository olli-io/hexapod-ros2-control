"""The face chooser: three valid values, anything else is an error."""
import os
import tempfile

import pytest

from hexa_bringup.face_backend import (
    BACKENDS,
    EYES_CPP,
    HEXA_DISPLAY,
    NONE,
    face_package,
    load_backend,
    parse_backend,
)


def _write(text):
    f = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
    f.write(text)
    f.close()
    return f.name


def test_default_is_none():
    assert parse_backend(None) == NONE


@pytest.mark.parametrize("name", BACKENDS)
def test_each_valid_name_is_accepted(name):
    assert parse_backend(name) == name


@pytest.mark.parametrize(
    "bad", ["", "both", "display", "eyes", "Eyes_Cpp", "HEXA_DISPLAY", " none", 1, True, ["none"]]
)
def test_anything_else_is_rejected_and_names_the_valid_ones(bad):
    with pytest.raises(ValueError) as e:
        parse_backend(bad)
    for name in BACKENDS:
        assert name in str(e.value)


def test_exactly_one_node_per_backend_and_none_for_none():
    assert face_package(NONE) is None
    assert face_package(HEXA_DISPLAY) == ("hexa_display", "display_node")
    assert face_package(EYES_CPP) == ("hexa_eyes", "eyes_node")
    # no backend ever starts both: the three answers are distinct
    assert len({face_package(b) for b in BACKENDS}) == 3


def test_shipped_config_defaults_to_none():
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    assert load_backend(os.path.join(here, "config", "face.yaml")) == NONE


def test_load_reads_the_chosen_backend():
    path = _write("face:\n  backend: eyes_cpp\n")
    try:
        assert load_backend(path) == EYES_CPP
    finally:
        os.unlink(path)


@pytest.mark.parametrize("text", ["", "face:\n", "other: 1\n"])
def test_missing_key_means_none(text):
    path = _write(text)
    try:
        assert load_backend(path) == NONE
    finally:
        os.unlink(path)


def test_typo_in_the_file_stops_the_launch():
    path = _write("face:\n  backend: both\n")
    try:
        with pytest.raises(ValueError):
            load_backend(path)
    finally:
        os.unlink(path)


def test_non_mapping_face_is_rejected():
    path = _write("face: eyes_cpp\n")
    try:
        with pytest.raises(ValueError):
            load_backend(path)
    finally:
        os.unlink(path)
