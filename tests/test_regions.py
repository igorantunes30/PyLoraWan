"""Testes de parametros regionais."""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest
from regions import EU868, US915, AU915, AS923, get_region


class TestEU868:
    def test_default_channels(self):
        r = EU868()
        assert len(r.default_channels) == 3
        assert 868.1 in r.default_channels

    def test_frequency_range(self):
        r = EU868()
        assert r.frequency_range == (863, 870)

    def test_max_tx_power(self):
        r = EU868()
        assert r.max_tx_power_dbm == 16

    def test_duty_cycle_exists(self):
        r = EU868()
        assert r.duty_cycle is not None
        dc = r.get_duty_cycle_limit(868.1)
        assert dc == 0.01  # 1% para sub-banda g1

    def test_no_dwell_time(self):
        r = EU868()
        assert r.max_dwell_time_ms is None
        assert r.check_dwell_time(5.0)  # Qualquer airtime OK

    def test_dr_table(self):
        r = EU868()
        assert r.dr_table[0]["sf"] == 12
        assert r.dr_table[5]["sf"] == 7

    def test_max_payload(self):
        r = EU868()
        assert r.get_max_payload(12, 125000) == 51
        assert r.get_max_payload(7, 125000) == 222

    def test_sf_to_dr(self):
        r = EU868()
        assert r.sf_to_dr(12, 125000) == 0
        assert r.sf_to_dr(7, 125000) == 5


class TestUS915:
    def test_64_uplink_channels(self):
        r = US915()
        assert len(r.default_channels) == 64

    def test_dwell_time_400ms(self):
        r = US915()
        assert r.max_dwell_time_ms == 400
        assert r.check_dwell_time(0.3)
        assert not r.check_dwell_time(0.5)

    def test_no_duty_cycle(self):
        r = US915()
        assert r.duty_cycle is None

    def test_max_tx_power(self):
        r = US915()
        assert r.max_tx_power_dbm == 30


class TestAU915:
    def test_dwell_time(self):
        r = AU915()
        assert r.max_dwell_time_ms == 400


class TestAS923:
    def test_default_channels(self):
        r = AS923()
        assert 923.2 in r.default_channels
        assert 923.4 in r.default_channels


class TestGetRegion:
    def test_get_eu868(self):
        r = get_region("EU868")
        assert r.name == "EU868"

    def test_get_us915(self):
        r = get_region("US915")
        assert r.name == "US915"

    def test_case_insensitive(self):
        r = get_region("eu868")
        assert r.name == "EU868"

    def test_unknown_raises(self):
        with pytest.raises(ValueError):
            get_region("UNKNOWN_REGION")
