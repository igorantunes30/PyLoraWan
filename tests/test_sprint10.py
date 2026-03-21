"""Testes Sprint 10: OTAA completo, session keys, MIC, NewChannelReq."""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest
import struct


# ===================== Security Module =====================

class TestKeyDerivation:
    def _make_key(self):
        return bytes(range(16))  # 0x00..0x0F

    def test_derive_session_keys_returns_two_keys(self):
        from security import derive_session_keys
        app_key = self._make_key()
        nwk, app = derive_session_keys(app_key, app_nonce=0x1A2B3C, net_id=0x000001, dev_nonce=0x1234)
        assert len(nwk) == 16
        assert len(app) == 16

    def test_nwk_and_app_skey_are_different(self):
        from security import derive_session_keys
        app_key = self._make_key()
        nwk, app = derive_session_keys(app_key, 0x1A2B3C, 0x000001, 0x1234)
        assert nwk != app

    def test_same_inputs_produce_same_keys(self):
        from security import derive_session_keys
        app_key = self._make_key()
        nwk1, app1 = derive_session_keys(app_key, 0x1A2B3C, 0x000001, 0x1234)
        nwk2, app2 = derive_session_keys(app_key, 0x1A2B3C, 0x000001, 0x1234)
        assert nwk1 == nwk2
        assert app1 == app2

    def test_different_dev_nonce_different_keys(self):
        from security import derive_session_keys
        app_key = self._make_key()
        nwk1, _ = derive_session_keys(app_key, 0x1A2B3C, 0x000001, 0x0001)
        nwk2, _ = derive_session_keys(app_key, 0x1A2B3C, 0x000001, 0x0002)
        assert nwk1 != nwk2

    def test_generate_app_key_is_16_bytes(self):
        from security import generate_app_key
        key = generate_app_key()
        assert len(key) == 16

    def test_generate_dev_eui_deterministic(self):
        from security import generate_dev_eui
        eui1 = generate_dev_eui(42)
        eui2 = generate_dev_eui(42)
        assert eui1 == eui2
        assert len(eui1) == 8

    def test_generate_dev_eui_unique_per_device(self):
        from security import generate_dev_eui
        assert generate_dev_eui(0) != generate_dev_eui(1)


class TestMIC:
    def _make_key(self):
        return bytes(range(16))

    def test_compute_frame_mic_returns_4_bytes(self):
        from security import compute_frame_mic
        key = self._make_key()
        mic = compute_frame_mic(key, 0x12345678, 1, 0, bytes(20))
        assert len(mic) == 4

    def test_same_inputs_same_mic(self):
        from security import compute_frame_mic
        key = self._make_key()
        mic1 = compute_frame_mic(key, 0x12345678, 1, 0, bytes(20))
        mic2 = compute_frame_mic(key, 0x12345678, 1, 0, bytes(20))
        assert mic1 == mic2

    def test_different_frame_counter_different_mic(self):
        from security import compute_frame_mic
        key = self._make_key()
        mic1 = compute_frame_mic(key, 0x12345678, 1, 0, bytes(20))
        mic2 = compute_frame_mic(key, 0x12345678, 2, 0, bytes(20))
        assert mic1 != mic2

    def test_verify_mic_valid(self):
        from security import compute_frame_mic, verify_frame_mic
        key = self._make_key()
        payload = bytes(20)
        mic = compute_frame_mic(key, 0x12345678, 5, 0, payload)
        assert verify_frame_mic(key, 0x12345678, 5, 0, payload, mic)

    def test_verify_mic_invalid_wrong_counter(self):
        from security import compute_frame_mic, verify_frame_mic
        key = self._make_key()
        payload = bytes(20)
        mic = compute_frame_mic(key, 0x12345678, 5, 0, payload)
        assert not verify_frame_mic(key, 0x12345678, 6, 0, payload, mic)

    def test_compute_join_mic_returns_4_bytes(self):
        from security import compute_join_mic, generate_app_eui, generate_dev_eui
        key = self._make_key()
        mic = compute_join_mic(key, generate_app_eui(), generate_dev_eui(1), 0x0042)
        assert len(mic) == 4

    def test_join_mic_changes_with_dev_nonce(self):
        from security import compute_join_mic, generate_app_eui, generate_dev_eui
        key = self._make_key()
        app_eui = generate_app_eui()
        dev_eui = generate_dev_eui(1)
        mic1 = compute_join_mic(key, app_eui, dev_eui, 0x0001)
        mic2 = compute_join_mic(key, app_eui, dev_eui, 0x0002)
        assert mic1 != mic2


# ===================== OTAA Device Flow =====================

class TestOTAADevice:
    def _make_device(self):
        from network import Network
        net = Network(
            num_devices=1, num_gateways=1, area_size=1000,
            lambda_rate=0.1, speed=0.0, sf_range=[7],
            tx_power=14, frequency_mhz=[868.1],
            ht_m=30, hr_m=1, bw=[125000], cr=1, pl=20,
            simulation_time=10, adr_enabled=False,
            model_pathloss="log_normal_shadowing",
        )
        return net.devices[0], net

    def test_device_has_app_key(self):
        device, _ = self._make_device()
        assert device.app_key is not None
        assert len(device.app_key) == 16

    def test_device_has_dev_eui(self):
        device, _ = self._make_device()
        assert device.dev_eui is not None
        assert len(device.dev_eui) == 8

    def test_prepare_join_request_returns_dict(self):
        device, _ = self._make_device()
        req = device.prepare_join_request()
        assert 'dev_eui' in req
        assert 'dev_nonce' in req
        assert 'mic' in req
        assert len(req['mic']) == 4

    def test_dev_nonce_increments(self):
        device, _ = self._make_device()
        req1 = device.prepare_join_request()
        req2 = device.prepare_join_request()
        assert req2['dev_nonce'] == req1['dev_nonce'] + 1

    def test_process_join_accept_sets_session_keys(self):
        device, _ = self._make_device()
        device.prepare_join_request()
        result = device.process_join_accept(
            app_nonce=0x1A2B3C,
            net_id=0x000001,
            dev_addr=0xABCD1234,
        )
        assert result is True
        assert device.nwk_skey is not None
        assert len(device.nwk_skey) == 16
        assert device.app_skey is not None
        assert device.dev_addr == 0xABCD1234

    def test_process_join_accept_resets_frame_counter(self):
        device, _ = self._make_device()
        device.frame_counter_up = 100
        device.prepare_join_request()
        device.process_join_accept(0x1A2B3C, 0x000001, 0xABCD1234)
        assert device.frame_counter_up == 0


# ===================== OTAA Network Server =====================

class TestOTAAServer:
    def _make_ns(self):
        from network_server.server import NetworkServer
        return NetworkServer()

    def test_on_join_request_returns_params(self):
        ns = self._make_ns()
        from security import generate_app_key, generate_dev_eui, generate_app_eui
        result = ns.on_join_request(
            device_id=0,
            dev_eui=generate_dev_eui(0),
            app_eui=generate_app_eui(),
            dev_nonce=0x0001,
            app_key=generate_app_key(),
        )
        assert result is not None
        assert 'app_nonce' in result
        assert 'net_id' in result
        assert 'dev_addr' in result

    def test_on_join_request_registers_device(self):
        ns = self._make_ns()
        from security import generate_app_key, generate_dev_eui, generate_app_eui
        ns.on_join_request(
            device_id=5,
            dev_eui=generate_dev_eui(5),
            app_eui=generate_app_eui(),
            dev_nonce=0x0042,
            app_key=generate_app_key(),
        )
        status = ns.device_registry.get_status(5)
        assert status is not None
        assert status.joined is True
        assert status.nwk_skey is not None

    def test_on_join_request_stores_session_keys(self):
        ns = self._make_ns()
        from security import generate_app_key, generate_dev_eui, generate_app_eui
        ns.on_join_request(
            device_id=7,
            dev_eui=generate_dev_eui(7),
            app_eui=generate_app_eui(),
            dev_nonce=0x0003,
            app_key=generate_app_key(),
        )
        status = ns.device_registry.get_status(7)
        assert len(status.nwk_skey) == 16
        assert len(status.app_skey) == 16


# ===================== Packet MIC =====================

class TestPacketMIC:
    def test_packet_has_mic_attribute(self):
        from packet import Packet
        p = Packet(0, 7, 14, 125000, 868.1, -90, 0.0, 0.1)
        assert hasattr(p, 'mic')
        assert p.mic is None

    def test_packet_has_mic_valid_attribute(self):
        from packet import Packet
        p = Packet(0, 7, 14, 125000, 868.1, -90, 0.0, 0.1)
        assert hasattr(p, 'mic_valid')
        assert p.mic_valid is None


# ===================== NewChannelReq =====================

class TestNewChannelReq:
    def _make_device(self):
        from network import Network
        net = Network(
            num_devices=1, num_gateways=1, area_size=1000,
            lambda_rate=0.1, speed=0.0, sf_range=[7],
            tx_power=14, frequency_mhz=[868.1],
            ht_m=30, hr_m=1, bw=[125000], cr=1, pl=20,
            simulation_time=10, adr_enabled=False,
            model_pathloss="log_normal_shadowing",
        )
        return net.devices[0]

    def test_new_channel_adds_to_available_channels(self):
        from mac_commands import MACCommandProcessor, NewChannelReq
        device = self._make_device()
        processor = MACCommandProcessor()
        cmd = NewChannelReq(ch_index=3, frequency=867.1, min_dr=0, max_dr=5)
        result = processor.process_downlink_commands(device, [cmd])
        assert 867.1 in device._available_channels

    def test_new_channel_ans_returned(self):
        from mac_commands import MACCommandProcessor, NewChannelReq, NewChannelAns
        device = self._make_device()
        processor = MACCommandProcessor()
        cmd = NewChannelReq(ch_index=3, frequency=867.3, min_dr=0, max_dr=5)
        result = processor.process_downlink_commands(device, [cmd])
        assert len(result) == 1
        assert isinstance(result[0], NewChannelAns)

    def test_new_channel_invalid_dr_range_rejected(self):
        from mac_commands import MACCommandProcessor, NewChannelReq, NewChannelAns
        device = self._make_device()
        processor = MACCommandProcessor()
        cmd = NewChannelReq(ch_index=3, frequency=867.5, min_dr=5, max_dr=3)  # min > max
        result = processor.process_downlink_commands(device, [cmd])
        assert isinstance(result[0], NewChannelAns)
        assert not result[0].payload['dr_range_ok']

    def test_new_channel_component_exists(self):
        from network_server.components.new_channel import NewChannelComponent
        nc = NewChannelComponent()
        assert hasattr(nc, 'configured')

    def test_new_channel_component_sends_once_per_device(self):
        from network_server.components.new_channel import NewChannelComponent
        from packet import Packet
        nc = NewChannelComponent()

        class FakeStatus:
            gateways = {}

        p = Packet(0, 7, 14, 125000, 868.1, -90, 0.0, 0.1)
        r1 = nc.on_packet(p, FakeStatus())
        r2 = nc.on_packet(p, FakeStatus())

        # Primeira vez: envia comandos; segunda vez: nada
        assert len(r1) > 0
        assert len(r2) == 0

    def test_new_channel_registered_in_ns(self):
        from network_server.server import NetworkServer
        ns = NetworkServer()
        assert hasattr(ns, 'new_channel')
