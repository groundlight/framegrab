import pytest

from stream import parse_crop_string


def test_good_crop():
    assert parse_crop_string("0,0,1,1") == (0, 0, 1, 1)
    assert parse_crop_string("0.5,0,0.5,1") == (0.5, 0, 0.5, 1)

    with pytest.raises(ValueError):
        # too short
        parse_crop_string("0.5,0,0.5")

    with pytest.raises(ValueError):
        # not numbers
        parse_crop_string("a,b,c,d")

    with pytest.raises(ValueError):
        # too big
        parse_crop_string("0,0,256,250")

    with pytest.raises(ValueError):
        # negative
        parse_crop_string("-1,0,1,1")

    with pytest.raises(ValueError):
        # Off both edges
        parse_crop_string("0.5,0.5,0.6,0.6")

    with pytest.raises(ValueError):
        # Off right
        parse_crop_string("0.3,0,0.8,1")

    with pytest.raises(ValueError):
        # Off bottom
        parse_crop_string("0,0.2,0.5,0.9")

    with pytest.raises(ValueError):
        # zero size
        parse_crop_string("0,0,1,0")
