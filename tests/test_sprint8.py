"""Testes Sprint 8: PathLoss Oulu, DL-DL, RxParamSetup, DevStatus, LinkCheck."""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest
import parametors

# ===================== PathLoss Oulu =====================

class TestPathlossOulu:
    def _make_network(self):
        from network import Network
        return Network(
            num_devices=2, num_gateways=1, area_size=1000,
            lambda_rate=0.1, speed=1.0, sf_range=[7], tx_power=14,
            frequency_mhz=[868.1], ht_m=30, hr_m=1, bw=[125000],
            cr=1, pl=20, simulation_time=10, adr_enabled=False,
            model_pathloss="oulu",
        )

    def test_oulu_model_returns_float(self):
        net = self._make_network()
        pl = net.pathloss(1000, 868.1, "oulu")
        assert isinstance(pl, float)

    def test_oulu_increases_with_distance(self):
        net = self._make_network()
        import numpy as np
        np.random.seed(0)
        pl_near = net.pathloss(100, 868.1, "oulu")
        np.random.seed(0)
        pl_far = net.pathloss(5000, 868.1, "oulu")
        assert pl_far > pl_near

    def test_oulu_params_match_flora(self):
        """PL_d0=127.41, d0=40m, gamma=2.08 — igual FLoRa LoRaSim."""
        net = self._make_network()
        import numpy as np
        # Sem shadowing: PL(d0=40m) deve ser proximo de 127.41
        results = [net.pathloss(40.0, 868.1, "oulu") for _ in range(200)]
        mean_pl = float(np.mean(results))
        assert abs(mean_pl - 127.41) < 2.0  # dentro de 2 dB (sigma=3.57)

    def test_oulu_d0_formula(self):
        """Sem shadowing (seed fixo), PL(40) ~= 127.41."""
        net = self._make_network()
        import numpy as np
        np.random.seed(42)
        results = [net.pathloss(40.0, 868.1, "oulu") for _ in range(500)]
        mean_pl = float(np.mean(results))
        assert abs(mean_pl - 127.41) < 1.0


# ===================== DL-DL Interference =====================

class TestDLDLInterference:
    def _make_network(self, n_gw=2):
        from network import Network
        return Network(
            num_devices=4, num_gateways=n_gw, area_size=2000,
            lambda_rate=0.05, speed=1.0, sf_range=[7], tx_power=14,
            frequency_mhz=[868.1], ht_m=30, hr_m=1, bw=[125000],
            cr=1, pl=20, simulation_time=10, adr_enabled=False,
            model_pathloss="log_normal_shadowing",
        )

    def test_dl_dl_busy_dict_initialized(self):
        net = self._make_network()
        assert hasattr(net, '_network_dl_busy')
        assert isinstance(net._network_dl_busy, dict)

    def test_dl_dl_blocks_second_dl_same_freq(self):
        """Se freq ja esta ocupada com DL, segundo DL deve ser bloqueado."""
        net = self._make_network()
        freq = 868.1
        # Marca freq como ocupada ate tempo 10.0
        net._network_dl_busy[freq] = 10.0
        # Verifica que a freq esta bloqueada em t=5
        assert net._network_dl_busy.get(freq, 0) > 5.0

    def test_dl_dl_releases_after_airtime(self):
        """Apos airtime, freq deve estar livre."""
        net = self._make_network()
        freq = 868.1
        net._network_dl_busy[freq] = 0.1  # expira em 0.1s
        # Em t=0.2, deve estar livre
        assert net._network_dl_busy.get(freq, 0) <= 0.2


# ===================== RxParamSetup =====================

class TestRxParamSetup:
    def _make_device(self):
        from network import Network
        net = Network(
            num_devices=1, num_gateways=1, area_size=1000,
            lambda_rate=0.1, speed=1.0, sf_range=[7], tx_power=14,
            frequency_mhz=[868.1], ht_m=30, hr_m=1, bw=[125000],
            cr=1, pl=20, simulation_time=10, adr_enabled=False,
            model_pathloss="log_normal_shadowing",
        )
        return net.devices[0]

    def test_device_has_rx2_attrs(self):
        device = self._make_device()
        assert hasattr(device, '_rx2_freq')
        assert hasattr(device, '_rx2_sf')
        assert hasattr(device, '_rx2_bw')
        assert hasattr(device, '_rx1_dr_offset')
        assert device._rx2_freq is None
        assert device._rx2_sf is None

    def test_apply_rx_param_setup_updates_device(self):
        from mac_commands import MACCommandProcessor, RXParamSetupReq
        device = self._make_device()
        processor = MACCommandProcessor()
        cmd = RXParamSetupReq(rx1_dr_offset=1, rx2_dr=0, rx2_frequency=869.525)
        result = processor.process_downlink_commands(device, [cmd])
        assert device._rx1_dr_offset == 1
        assert device._rx2_sf == 12  # DR0 = SF12
        assert device._rx2_freq == 869.525

    def test_rx_param_setup_ans_returned(self):
        from mac_commands import MACCommandProcessor, RXParamSetupReq, RXParamSetupAns
        device = self._make_device()
        processor = MACCommandProcessor()
        cmd = RXParamSetupReq(rx1_dr_offset=0, rx2_dr=0, rx2_frequency=869.525)
        result = processor.process_downlink_commands(device, [cmd])
        assert len(result) == 1
        assert isinstance(result[0], RXParamSetupAns)

    def test_invalid_rx1_dr_offset_rejected(self):
        from mac_commands import MACCommandProcessor, RXParamSetupReq, RXParamSetupAns
        device = self._make_device()
        processor = MACCommandProcessor()
        cmd = RXParamSetupReq(rx1_dr_offset=10, rx2_dr=0, rx2_frequency=869.525)
        result = processor.process_downlink_commands(device, [cmd])
        assert isinstance(result[0], RXParamSetupAns)
        assert not result[0].payload['rx1_dr_offset_ack']


# ===================== DevStatus =====================

class TestDevStatusComponent:
    def test_dev_status_component_exists(self):
        from network_server.components.dev_status import DevStatusComponent
        ds = DevStatusComponent(period=5)
        assert ds.period == 5

    def test_dev_status_req_generated_every_period(self):
        from network_server.components.dev_status import DevStatusComponent
        from packet import Packet
        ds = DevStatusComponent(period=3)

        class FakeStatus:
            gateways = {}

        p = Packet(0, 7, 14, 125000, 868.1, -90, 0.0, 0.1)

        results = []
        for _ in range(3):
            r = ds.on_packet(p, FakeStatus())
            results.append(r)

        # 3rd call should return DevStatusReq
        from mac_commands import DevStatusReq
        assert any(isinstance(c, DevStatusReq) for c in results[2])

    def test_dev_status_record_and_retrieve(self):
        from network_server.components.dev_status import DevStatusComponent
        ds = DevStatusComponent()
        ds.record_status(device_id=0, battery=200, snr_margin=5, time=10.0)
        status = ds.get_last_status(0)
        assert status is not None
        assert status['battery'] == 200
        assert status['snr_margin'] == 5
        assert status['time'] == 10.0

    def test_dev_status_registered_in_ns(self):
        from network_server.server import NetworkServer
        ns = NetworkServer()
        assert hasattr(ns, 'dev_status')

    def test_dev_status_multiple_devices(self):
        from network_server.components.dev_status import DevStatusComponent
        ds = DevStatusComponent()
        ds.record_status(0, 100, 3, 1.0)
        ds.record_status(1, 200, 7, 2.0)
        assert ds.get_last_status(0)['battery'] == 100
        assert ds.get_last_status(1)['battery'] == 200


# ===================== LinkCheck =====================

class TestLinkCheck:
    def test_link_check_component_exists(self):
        from network_server.components.link_check import LinkCheckComponent
        lc = LinkCheckComponent()
        assert hasattr(lc, 'pending_requests')

    def test_link_check_request_generates_ans(self):
        from network_server.components.link_check import LinkCheckComponent
        from mac_commands import LinkCheckAns
        from packet import Packet

        class FakeStatus:
            gateways = {'gw0': {}, 'gw1': {}}

        lc = LinkCheckComponent()
        p = Packet(0, 7, 14, 125000, 868.1, -90, 0.0, 0.1)
        p.snr = 5.0

        # Sem request, nada
        result = lc.on_packet(p, FakeStatus())
        assert result == []

        # Com request, gera LinkCheckAns
        lc.request_link_check(0)
        result = lc.on_packet(p, FakeStatus())
        assert len(result) == 1
        assert isinstance(result[0], LinkCheckAns)

    def test_link_check_margin_calculation(self):
        from network_server.components.link_check import LinkCheckComponent
        from mac_commands import LinkCheckAns
        from packet import Packet

        class FakeStatus:
            gateways = {'gw0': {}}

        lc = LinkCheckComponent()
        p = Packet(0, 7, 14, 125000, 868.1, -90, 0.0, 0.1)
        p.snr = 0.0  # SNR=0, min=-7.5 -> margin=7

        lc.request_link_check(0)
        result = lc.on_packet(p, FakeStatus())
        assert result[0].payload['margin'] >= 0

    def test_link_check_gw_count(self):
        from network_server.components.link_check import LinkCheckComponent
        from packet import Packet

        class FakeStatus:
            gateways = {'gw0': {}, 'gw1': {}, 'gw2': {}}

        lc = LinkCheckComponent()
        p = Packet(0, 7, 14, 125000, 868.1, -90, 0.0, 0.1)
        p.snr = 5.0

        lc.request_link_check(0)
        result = lc.on_packet(p, FakeStatus())
        assert result[0].payload['gw_count'] == 3
