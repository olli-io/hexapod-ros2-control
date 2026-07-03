"""Guard the gait-descriptor catalog values.

These duty factors and stability flags are the single Python source of
truth — ``hexa_gait``'s strategy classes read them, and drift here would
silently retune every downstream velocity cap. The C++ engine keeps a
parallel copy in ``hexa_gait_cpp/gaits/registry.cpp``; if you change a
value here, change it there too.
"""

from hexa_common.gait_catalog import GAIT_DESCRIPTORS, GaitDescriptor


def test_catalog_has_the_five_registered_gaits():
    assert set(GAIT_DESCRIPTORS) == {
        "tripod",
        "surf",
        "tetrapod",
        "crawl",
        "ripple",
    }


def test_descriptor_name_matches_key():
    for name, descriptor in GAIT_DESCRIPTORS.items():
        assert isinstance(descriptor, GaitDescriptor)
        assert descriptor.name == name


def test_duty_factors():
    assert GAIT_DESCRIPTORS["tripod"].duty_factor == 0.5
    assert GAIT_DESCRIPTORS["surf"].duty_factor == 5.0 / 8.0
    assert GAIT_DESCRIPTORS["tetrapod"].duty_factor == 2.0 / 3.0
    assert GAIT_DESCRIPTORS["crawl"].duty_factor == 2.0 / 3.0
    assert GAIT_DESCRIPTORS["ripple"].duty_factor == 5.0 / 6.0


def test_unstable_flags():
    assert GAIT_DESCRIPTORS["tripod"].unstable is False
    assert GAIT_DESCRIPTORS["surf"].unstable is True
    assert GAIT_DESCRIPTORS["tetrapod"].unstable is False
    assert GAIT_DESCRIPTORS["crawl"].unstable is True
    assert GAIT_DESCRIPTORS["ripple"].unstable is False
