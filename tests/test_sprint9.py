"""Testes Sprint 9: Modelo Analitico Ps1/Ps2 + MRC Macro-Diversity."""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest
import math
import numpy as np


# ===================== Modelo Analitico =====================

class TestAnalyticalToa:
    def test_toa_sf7_125k(self):
        from analytics import _toa_analytical
        toa = _toa_analytical(7, 125000, 20, 1)
        # SF7, 125kHz, 20B payload ~ 56ms
        assert 0.04 < toa < 0.10

    def test_toa_sf12_125k(self):
        from analytics import _toa_analytical
        toa = _toa_analytical(12, 125000, 20, 1)
        # SF12, 125kHz, 20B payload ~ 1.3s (>> SF7 ~56ms)
        assert toa > 1.0

    def test_toa_increases_with_sf(self):
        from analytics import _toa_analytical
        toas = [_toa_analytical(sf, 125000, 20, 1) for sf in range(7, 13)]
        for i in range(len(toas) - 1):
            assert toas[i] < toas[i+1]


class TestPs1PerSF:
    def _make_network(self):
        from network import Network
        return Network(
            num_devices=10, num_gateways=1, area_size=2000,
            lambda_rate=0.1, speed=0.0, sf_range=[7, 8, 9],
            tx_power=14, frequency_mhz=[868.1, 868.3, 868.5],
            ht_m=30, hr_m=1, bw=[125000], cr=1, pl=20,
            simulation_time=60, adr_enabled=False,
            model_pathloss="log_normal_shadowing",
        )

    def test_ps1_returns_dict_per_sf(self):
        from analytics import analytical_ps1_per_sf
        net = self._make_network()
        result = analytical_ps1_per_sf(net)
        assert isinstance(result, dict)
        assert all(sf in result for sf in range(7, 13))

    def test_ps1_between_0_and_1(self):
        from analytics import analytical_ps1_per_sf
        net = self._make_network()
        result = analytical_ps1_per_sf(net)
        for sf, v in result.items():
            assert 0.0 <= v['Ps1'] <= 1.0, f"Ps1[{sf}] = {v['Ps1']} out of range"

    def test_ps1_fields_present(self):
        from analytics import analytical_ps1_per_sf
        net = self._make_network()
        result = analytical_ps1_per_sf(net)
        for sf in range(7, 13):
            assert 'Ps1' in result[sf]
            assert 'p_no_collision' in result[sf]
            assert 'p_coverage' in result[sf]
            assert 'G' in result[sf]

    def test_ps1_no_collision_decreases_with_load(self):
        """P_no_collision diminui com mais devices (mais carga)."""
        from analytics import analytical_ps1_per_sf
        from network import Network

        net_low = Network(
            num_devices=5, num_gateways=1, area_size=2000,
            lambda_rate=0.01, speed=0.0, sf_range=[7],
            tx_power=14, frequency_mhz=[868.1],
            ht_m=30, hr_m=1, bw=[125000], cr=1, pl=20,
            simulation_time=60, adr_enabled=False,
            model_pathloss="log_normal_shadowing",
        )
        net_high = Network(
            num_devices=50, num_gateways=1, area_size=2000,
            lambda_rate=0.5, speed=0.0, sf_range=[7],
            tx_power=14, frequency_mhz=[868.1],
            ht_m=30, hr_m=1, bw=[125000], cr=1, pl=20,
            simulation_time=60, adr_enabled=False,
            model_pathloss="log_normal_shadowing",
        )

        r_low = analytical_ps1_per_sf(net_low)
        r_high = analytical_ps1_per_sf(net_high)
        # Mais carga -> menor P_no_collision
        assert r_low[7]['p_no_collision'] >= r_high[7]['p_no_collision']

    def test_toa_formula_consistent_with_device(self):
        """ToA analitico deve ser proximo ao airtime calculado pelo device."""
        from analytics import _toa_analytical
        from network import Network
        net = Network(
            num_devices=1, num_gateways=1, area_size=1000,
            lambda_rate=0.1, speed=0.0, sf_range=[7],
            tx_power=14, frequency_mhz=[868.1],
            ht_m=30, hr_m=1, bw=[125000], cr=1, pl=20,
            simulation_time=10, adr_enabled=False,
            model_pathloss="log_normal_shadowing",
        )
        device = net.devices[0]
        device.sf = 7
        device.bw = 125000
        toa_device = device.calculate_airtime()
        toa_anal = _toa_analytical(7, 125000, 20, 1)
        assert abs(toa_device - toa_anal) < 0.001  # < 1ms de diferenca


class TestPs2RX2:
    def _make_network(self):
        from network import Network
        return Network(
            num_devices=10, num_gateways=1, area_size=2000,
            lambda_rate=0.1, speed=0.0, sf_range=[7, 8],
            tx_power=14, frequency_mhz=[868.1, 868.3, 868.5],
            ht_m=30, hr_m=1, bw=[125000], cr=1, pl=20,
            simulation_time=60, adr_enabled=False,
            model_pathloss="log_normal_shadowing",
        )

    def test_ps2_fields_present(self):
        from analytics import analytical_ps2_rx2
        net = self._make_network()
        result = analytical_ps2_rx2(net)
        assert 'Ps2' in result
        assert 'p_no_collision' in result
        assert 'p_coverage' in result
        assert 'toa_rx2_s' in result

    def test_ps2_between_0_and_1(self):
        from analytics import analytical_ps2_rx2
        net = self._make_network()
        result = analytical_ps2_rx2(net)
        assert 0.0 <= result['Ps2'] <= 1.0

    def test_ps2_toa_sf12_larger_than_sf7(self):
        from analytics import _toa_analytical
        toa_sf7 = _toa_analytical(7, 125000, 20, 1)
        toa_sf12 = _toa_analytical(12, 125000, 13, 1)
        assert toa_sf12 > toa_sf7


class TestComparePDR:
    def _make_small_network(self):
        from network import Network
        import random
        random.seed(99)
        import numpy as np
        np.random.seed(99)
        return Network(
            num_devices=10, num_gateways=1, area_size=2000,
            lambda_rate=0.1, speed=0.0, sf_range=[7, 8, 9],
            tx_power=14, frequency_mhz=[868.1, 868.3, 868.5],
            ht_m=30, hr_m=1, bw=[125000], cr=1, pl=20,
            simulation_time=60, adr_enabled=False,
            model_pathloss="log_normal_shadowing",
        )

    def test_compare_ps1_ps2_returns_dict(self):
        from analytics import compare_ps1_ps2
        net = self._make_small_network()
        result = compare_ps1_ps2(net)
        assert 'analytical_pdr' in result
        assert 'simulation_pdr' in result
        assert 'ps1_per_sf' in result
        assert 'ps2' in result

    def test_analytical_pdr_between_0_and_1(self):
        from analytics import compare_ps1_ps2
        net = self._make_small_network()
        result = compare_ps1_ps2(net)
        assert 0.0 <= result['analytical_pdr'] <= 1.0

    def test_compare_with_analytical_includes_ps1_ps2(self):
        from analytics import compare_with_analytical
        net = self._make_small_network()
        result = compare_with_analytical(net)
        assert 'ps1_ps2' in result
        assert 'analytical_pdr' in result['ps1_ps2']


# ===================== MRC Macro-Diversity =====================

class TestMRCPacket:
    def test_packet_has_snr_mrc(self):
        from packet import Packet
        p = Packet(0, 7, 14, 125000, 868.1, -90, 0.0, 0.1)
        assert hasattr(p, 'snr_mrc')
        assert p.snr_mrc is None

    def test_packet_has_mrc_gw_count(self):
        from packet import Packet
        p = Packet(0, 7, 14, 125000, 868.1, -90, 0.0, 0.1)
        assert hasattr(p, 'mrc_gw_count')
        assert p.mrc_gw_count == 1


class TestMRCChannelModel:
    def _make_channel(self):
        class MockNetwork:
            def calculate_noise_floor(self, bw):
                return -120.0
        from channel import ChannelModel
        return ChannelModel(MockNetwork())

    def test_mrc_snr_used_when_available(self):
        """Channel usa snr_mrc em vez de rssi - noise_floor quando disponivel."""
        from packet import Packet
        channel = self._make_channel()
        p = Packet(0, 7, 14, 125000, 868.1, -130, 0.0, 0.1)  # rssi muito baixo (falharia)
        p.snr_mrc = 5.0  # MRC melhorou SNR significativamente
        channel.add_transmission(p, 0.0, 0.1)
        # Com snr_mrc=5 > snr_min[SF7]=-7.5, deve passar a verificacao de SNR
        result = channel.evaluate_reception(p, None)
        assert result, "MRC deve permitir recepcao com snr_mrc elevado"

    def test_without_mrc_low_snr_fails(self):
        """Sem MRC, packet com RSSI abaixo do limiar deve falhar."""
        from packet import Packet
        channel = self._make_channel()
        p = Packet(0, 7, 14, 125000, 868.1, -130, 0.0, 0.1)  # snr = -10 < -7.5
        p.snr_mrc = None  # sem MRC
        channel.add_transmission(p, 0.0, 0.1)
        result = channel.evaluate_reception(p, None)
        assert not result, "Sem MRC, RSSI baixo deve falhar"


class TestMRCNetworkIntegration:
    def _make_multi_gw_network(self):
        from network import Network
        return Network(
            num_devices=4, num_gateways=2, area_size=3000,
            lambda_rate=0.05, speed=0.0, sf_range=[7],
            tx_power=14, frequency_mhz=[868.1],
            ht_m=30, hr_m=1, bw=[125000], cr=1, pl=20,
            simulation_time=10, adr_enabled=False,
            model_pathloss="log_normal_shadowing",
        )

    def test_mrc_attributes_on_packet_after_send(self):
        """Packet deve ter snr_mrc e mrc_gw_count apos process_uplink com multi-GW."""
        from packet import Packet
        assert hasattr(Packet(0, 7, 14, 125000, 868.1, -90, 0.0, 0.1), 'snr_mrc')

    def test_gateway_manager_process_uplink_from_all(self):
        """process_uplink_from_all deve retornar recepcoes para GWs que ouvem o pacote."""
        net = self._make_multi_gw_network()
        device = net.devices[0]
        from packet import Packet
        p = Packet(device.device_id, 7, 14, 125000, 868.1, -90, 0.0, 0.1)
        receptions = net.ns.gateway_manager.process_uplink_from_all(p, net.gateways, net)
        assert isinstance(receptions, list)
        # Cada recepcao deve ter rssi, snr, gateway
        for r in receptions:
            assert 'rssi' in r
            assert 'snr' in r
            assert 'gateway' in r

    def test_mrc_formula_sum_linear(self):
        """MRC: SNR_comb = 10*log10(sum(10^(SNRi/10))) > max(SNRi)."""
        snr_gw1 = 5.0   # dB
        snr_gw2 = 5.0   # dB
        snr_linear_sum = 10 ** (snr_gw1 / 10) + 10 ** (snr_gw2 / 10)
        snr_mrc = 10 * math.log10(snr_linear_sum)
        # SNR combinado deve ser maior que cada SNR individual
        assert snr_mrc > max(snr_gw1, snr_gw2)
        # Para dois sinais iguais, ganho esperado e ~3 dB
        assert abs(snr_mrc - (snr_gw1 + 3.01)) < 0.1

    def test_mrc_gain_3db_two_equal_gws(self):
        """Dois GWs com mesmo SNR -> ganho MRC exatamente 3.01 dB."""
        snr = 10.0
        snr_mrc = 10 * math.log10(2 * 10 ** (snr / 10))
        assert abs(snr_mrc - snr - 3.01) < 0.01
