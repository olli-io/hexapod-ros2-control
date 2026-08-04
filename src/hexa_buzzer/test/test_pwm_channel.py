"""Tests for the sysfs seam, against a directory standing in for the PWM tree.

The one suite that imports a submodule rather than the package root: sysfs is
just files, so tmp_path reproduces the seam faithfully, and a leaked export
locks out the next caller, host or container.
"""
import pytest

from hexa_buzzer import pwm


def make_tree(root, chip_name="pwmchip0", with_channel=True):
    """A fake PWM block. `with_channel` pre-creates the exported channel."""
    chip = root / "pwm" / chip_name
    chip.mkdir(parents=True)
    for name in ("export", "unexport"):
        (chip / name).write_text("")
    if with_channel:
        (chip / "pwm0").mkdir()
        for name in ("period", "duty_cycle", "enable"):
            (chip / "pwm0" / name).write_text("")
    return root / "pwm", chip


def test_the_chip_is_found_by_glob_not_by_number(tmp_path):
    """The pwmchipN number is probe order and has moved between releases."""
    dev, chip = make_tree(tmp_path, chip_name="pwmchip7")
    assert pwm.find_chip(str(dev)) == str(chip)


def test_a_missing_tree_says_so_rather_than_hanging(tmp_path):
    with pytest.raises(pwm.BuzzerHardwareError, match="no pwmchip"):
        pwm.find_chip(str(tmp_path / "absent"))


def test_a_tone_is_a_half_duty_square_wave(tmp_path):
    dev, chip = make_tree(tmp_path)
    with pwm.Channel(str(dev)) as line:
        line.tone(1000)
    assert (chip / "pwm0" / "period").read_text() == "1000000"
    assert (chip / "pwm0" / "duty_cycle").read_text() == "500000"


def test_a_rest_leaves_the_channel_disabled(tmp_path):
    dev, chip = make_tree(tmp_path)
    with pwm.Channel(str(dev)) as line:
        line.tone(0)
    assert (chip / "pwm0" / "enable").read_text() == "0"


def test_the_buzzer_is_silent_however_the_block_is_left(tmp_path):
    dev, chip = make_tree(tmp_path)
    with pytest.raises(RuntimeError):
        with pwm.Channel(str(dev)) as line:
            line.tone(1000)
            raise RuntimeError("something else went wrong mid-tune")
    assert (chip / "pwm0" / "enable").read_text() == "0"


def test_a_channel_somebody_else_exported_is_left_alone(tmp_path):
    """Host units and the node share one channel, so only its exporter frees
    it."""
    dev, chip = make_tree(tmp_path)
    with pwm.Channel(str(dev)):
        pass
    assert (chip / "unexport").read_text() == ""


def test_a_channel_we_exported_is_released(tmp_path, monkeypatch):
    dev, chip = make_tree(tmp_path, with_channel=False)

    # Stand in for the kernel: the directory appears once `export` is written.
    real_isdir = pwm.os.path.isdir

    def isdir(path):
        if path == str(chip / "pwm0") and (chip / "export").read_text() == "0":
            (chip / "pwm0").mkdir(exist_ok=True)
        return real_isdir(path)

    monkeypatch.setattr(pwm.os.path, "isdir", isdir)
    with pwm.Channel(str(dev)):
        pass

    assert (chip / "unexport").read_text() == "0"


def test_an_export_that_never_appears_does_not_leak_the_channel(tmp_path, monkeypatch):
    """__exit__ does not run when __enter__ raises, so the release has to be
    inside it, or the channel stays claimed against every later caller."""
    monkeypatch.setattr(pwm, "_EXPORT_TIMEOUT_S", 0.05)
    dev, chip = make_tree(tmp_path, with_channel=False)

    with pytest.raises(pwm.BuzzerHardwareError, match="never appeared"):
        with pwm.Channel(str(dev)):
            pass

    assert (chip / "export").read_text() == "0"
    assert (chip / "unexport").read_text() == "0"
