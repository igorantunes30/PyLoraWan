"""Testes de Time-on-Air vs valores Semtech LoRa Calculator."""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest
from enddevice import EndDevice


def make_device(sf, bw, pl, cr=1):
    """Cria device dummy para calculo de ToA."""
    class DummyNetwork:
        def find_best_gateway(self, d): return None, None, None
    net = DummyNetwork()
    d = EndDevice(
        device_id=0, x=0, y=0, lambda_rate=0.1, speed=0,
        sf_range=[sf], tx_power=14, bw=[bw], cr=cr, pl=pl,
        frequency_mhz=[868.1], network=net, mobility_enabled=False
    )
    d.sf = sf
    d.bw = bw
    d.pl = pl
    d.cr = cr
    return d


class TestTimeOnAir:
    def test_toa_sf7_125khz_20bytes(self):
        """ToA SF7/125kHz/20B deve ser ~51ms (Semtech Calculator)."""
        d = make_device(sf=7, bw=125000, pl=20)
        toa = d.calculate_airtime()
        assert 0.040 < toa < 0.070, f"ToA SF7 = {toa:.4f}s, esperado ~0.051s"

    def test_toa_sf12_125khz_51bytes(self):
        """ToA SF12/125kHz/51B deve estar em faixa razoavel (~1.8-2.5s)."""
        d = make_device(sf=12, bw=125000, pl=51)
        toa = d.calculate_airtime()
        assert 1.5 < toa < 3.0, f"ToA SF12 = {toa:.4f}s, esperado entre 1.5 e 3.0s"

    def test_toa_sf7_250khz_20bytes(self):
        """ToA SF7/250kHz deve ser ~metade do SF7/125kHz."""
        d125 = make_device(sf=7, bw=125000, pl=20)
        d250 = make_device(sf=7, bw=250000, pl=20)
        toa125 = d125.calculate_airtime()
        toa250 = d250.calculate_airtime()
        ratio = toa125 / toa250
        assert 1.8 < ratio < 2.2, f"Ratio 125/250 = {ratio:.2f}, esperado ~2.0"

    def test_toa_increases_with_sf(self):
        """ToA deve aumentar monotonicamente com SF."""
        toas = []
        for sf in range(7, 13):
            d = make_device(sf=sf, bw=125000, pl=20)
            toas.append(d.calculate_airtime())
        for i in range(len(toas) - 1):
            assert toas[i] < toas[i+1], f"ToA SF{7+i}={toas[i]:.4f} >= ToA SF{8+i}={toas[i+1]:.4f}"

    def test_toa_increases_with_payload(self):
        """ToA deve aumentar com payload."""
        d10 = make_device(sf=7, bw=125000, pl=10)
        d50 = make_device(sf=7, bw=125000, pl=50)
        assert d10.calculate_airtime() < d50.calculate_airtime()

    def test_toa_positive(self):
        """ToA deve ser sempre positivo."""
        for sf in range(7, 13):
            for pl in [1, 10, 20, 51]:
                d = make_device(sf=sf, bw=125000, pl=pl)
                assert d.calculate_airtime() > 0
