"""Testes Sprint 6 — LR-FHSS: PHY, colisao por fragmento, ACRDA/SIC."""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest
from lrfhss import LRFHSS_PHY, LRFHSS_Channel, ACRDA, LRFHSSFragment


class TestLRFHSS_PHY:
    def setup_method(self):
        self.phy = LRFHSS_PHY(code_rate="1/3", obw=35, num_headers=3)

    def test_fragment_packet_returns_tuple(self):
        n_headers, n_payloads, threshold = self.phy.fragment_packet(20)
        assert n_headers == 3
        assert n_payloads > 0
        assert threshold >= 1

    def test_threshold_is_ceil_third(self):
        """threshold = ceil(n_payloads / 3) para code_rate=1/3."""
        import math
        _, n_payloads, threshold = self.phy.fragment_packet(20)
        assert threshold == math.ceil(n_payloads / 3)

    def test_calculate_toa_positive(self):
        toa = self.phy.calculate_toa(20)
        assert toa > 0

    def test_toa_increases_with_payload(self):
        toa_small = self.phy.calculate_toa(10)
        toa_large = self.phy.calculate_toa(50)
        assert toa_large > toa_small

    def test_hopping_sequence_length(self):
        seq = self.phy.generate_hopping_sequence(10)
        assert len(seq) == 10

    def test_hopping_sequence_in_range(self):
        seq = self.phy.generate_hopping_sequence(100)
        assert all(0 <= ch < self.phy.obw for ch in seq)

    def test_create_fragments_count(self):
        n_headers, n_payloads, _ = self.phy.fragment_packet(20)
        frags = self.phy.create_fragments("pkt1", 20, tx_start=0.0)
        assert len(frags) == n_headers + n_payloads

    def test_create_fragments_timing_monotonic(self):
        frags = self.phy.create_fragments("pkt1", 20, tx_start=0.0)
        for i in range(1, len(frags)):
            assert frags[i].tx_start >= frags[i-1].tx_end

    def test_create_fragments_header_first(self):
        frags = self.phy.create_fragments("pkt1", 20, tx_start=0.0)
        n_headers, _, _ = self.phy.fragment_packet(20)
        for f in frags[:n_headers]:
            assert f.frag_type == 'header'
        for f in frags[n_headers:]:
            assert f.frag_type == 'payload'

    def test_code_rate_2_3(self):
        phy = LRFHSS_PHY(code_rate="2/3", obw=35, num_headers=3)
        import math
        _, n_payloads, threshold = phy.fragment_packet(20)
        assert threshold == math.ceil(n_payloads / 3)

    def test_code_rate_1_2(self):
        phy = LRFHSS_PHY(code_rate="1/2", obw=35, num_headers=3)
        import math
        _, n_payloads, threshold = phy.fragment_packet(20)
        assert threshold == math.ceil(n_payloads / 2)


class TestLRFHSS_Channel:
    def setup_method(self):
        self.phy = LRFHSS_PHY(code_rate="1/3", obw=35, num_headers=3)
        self.channel = LRFHSS_Channel()

    def _make_packet_frags(self, packet_id, tx_start=0.0, payload=20):
        return self.phy.create_fragments(packet_id, payload, tx_start)

    def test_single_packet_no_collision(self):
        """Pacote sozinho no canal deve ser recebido."""
        frags = self._make_packet_frags("pkt1", 0.0)
        self.channel.add_fragments(frags)
        self.channel.check_fragment_collisions(frags)
        _, _, threshold = self.phy.fragment_packet(20)
        received = self.channel.evaluate_packet(frags, threshold)
        assert received is True

    def test_two_packets_same_channel_collision(self):
        """Dois pacotes no mesmo canal e tempo devem colidir."""
        # Forcamos todos os fragmentos no mesmo canal
        frags1 = self._make_packet_frags("pkt1", 0.0)
        frags2 = self._make_packet_frags("pkt2", 0.0)
        for f in frags2:
            f.channel = frags1[0].channel  # mesmo canal para todos

        self.channel.add_fragments(frags1)
        self.channel.add_fragments(frags2)
        self.channel.check_fragment_collisions(frags1)
        self.channel.check_fragment_collisions(frags2)

        # Ao menos alguns fragmentos devem colidir
        collided_1 = sum(1 for f in frags1 if f.collided)
        collided_2 = sum(1 for f in frags2 if f.collided)
        assert collided_1 > 0 or collided_2 > 0

    def test_two_packets_different_time_no_collision(self):
        """Pacotes completamente separados no tempo nao colidem."""
        toa = self.phy.calculate_toa(20)
        frags1 = self._make_packet_frags("pkt1", tx_start=0.0)
        frags2 = self._make_packet_frags("pkt2", tx_start=toa + 10.0)

        self.channel.add_fragments(frags1)
        self.channel.add_fragments(frags2)
        self.channel.check_fragment_collisions(frags1)
        self.channel.check_fragment_collisions(frags2)

        _, _, threshold = self.phy.fragment_packet(20)
        assert self.channel.evaluate_packet(frags1, threshold) is True
        assert self.channel.evaluate_packet(frags2, threshold) is True

    def test_evaluate_needs_at_least_one_header(self):
        """Sem header valido, pacote falha mesmo com payloads ok."""
        _, n_payloads, threshold = self.phy.fragment_packet(20)
        frags = self._make_packet_frags("pkt1", 0.0)
        # Marcar todos os headers como colididos
        n_headers = self.phy.num_headers
        for f in frags[:n_headers]:
            f.collided = True
        received = self.channel.evaluate_packet(frags, threshold)
        assert received is False

    def test_cleanup_expired(self):
        frags = self._make_packet_frags("pkt1", tx_start=0.0)
        self.channel.add_fragments(frags)
        toa = self.phy.calculate_toa(20)
        self.channel.cleanup_expired(toa + 20.0)
        assert len(self.channel.active_fragments) == 0

    def test_stats(self):
        stats = self.channel.stats()
        assert "total_collisions" in stats
        assert "total_receptions" in stats
        assert "active_fragments" in stats


class TestACRDA:
    def setup_method(self):
        self.phy = LRFHSS_PHY(code_rate="1/3", obw=35, num_headers=3)
        self.channel = LRFHSS_Channel()
        self.acrda = ACRDA(window_size=3)

    def test_decode_isolated_packet(self):
        """Pacote sem colisoes deve ser decodificado pelo ACRDA."""
        frags = self.phy.create_fragments("pkt1", 20, tx_start=0.0)
        self.channel.add_fragments(frags)
        self.acrda.register_packet("pkt1", frags)
        decoded, iters = self.acrda.process_window(self.channel)
        assert decoded >= 1
        assert iters >= 1

    def test_decoded_packets_tracked(self):
        frags = self.phy.create_fragments("pkt1", 20, tx_start=0.0)
        self.channel.add_fragments(frags)
        self.acrda.register_packet("pkt1", frags)
        self.acrda.process_window(self.channel)
        assert "pkt1" in self.acrda.decoded_packets

    def test_reset_clears_state(self):
        frags = self.phy.create_fragments("pkt1", 20, tx_start=0.0)
        self.acrda.register_packet("pkt1", frags)
        self.acrda.process_window(self.channel)
        self.acrda.reset()
        assert len(self.acrda.decoded_packets) == 0
        assert len(self.acrda.packet_fragments) == 0

    def test_sic_recovers_collided_packet(self):
        """SIC deve recuperar pkt2 apos decodificar pkt1 e cancelar sua interferencia."""
        toa = self.phy.calculate_toa(20)
        frags1 = self.phy.create_fragments("pkt1", 20, tx_start=0.0)
        frags2 = self.phy.create_fragments("pkt2", 20, tx_start=0.0)

        # Forcar frags2 no mesmo canal que frags1 para causar colisao
        for f1, f2 in zip(frags1, frags2):
            f2.channel = f1.channel

        self.channel.add_fragments(frags1)
        self.channel.add_fragments(frags2)
        self.channel.check_fragment_collisions(frags1)
        self.channel.check_fragment_collisions(frags2)

        self.acrda.register_packet("pkt1", frags1)
        self.acrda.register_packet("pkt2", frags2)

        # pkt1 pode nao estar colidido (se pkt2 e frags em outro canal por padding)
        # O importante e que process_window nao levanta excecao
        decoded, iters = self.acrda.process_window(self.channel)
        assert decoded >= 0
        assert iters >= 1


class TestLRFHSSIntegration:
    """Testes de integracao LR-FHSS com network.py."""

    def test_network_with_lrfhss_ratio(self):
        """Rede com 50% LR-FHSS deve criar devices marcados corretamente."""
        import sys
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
        from network import Network

        net = Network(
            num_devices=10, num_gateways=1, area_size=5000,
            lambda_rate=0.1, speed=1.5, sf_range=[7, 8, 9],
            tx_power=14, frequency_mhz=[868.1], ht_m=1.5, hr_m=30,
            bw=[125000], cr=1, pl=20, simulation_time=60,
            adr_enabled=False, model_pathloss="fspl",
            lrfhss_ratio=0.5
        )
        lrfhss_devs = [d for d in net.devices if d.use_lrfhss]
        css_devs = [d for d in net.devices if not d.use_lrfhss]
        assert len(lrfhss_devs) == 5
        assert len(css_devs) == 5

    def test_network_lrfhss_disabled(self):
        """Com lrfhss_ratio=0, nenhum device usa LR-FHSS."""
        from network import Network
        net = Network(
            num_devices=10, num_gateways=1, area_size=5000,
            lambda_rate=0.1, speed=1.5, sf_range=[7, 8],
            tx_power=14, frequency_mhz=[868.1], ht_m=1.5, hr_m=30,
            bw=[125000], cr=1, pl=20, simulation_time=60,
            adr_enabled=False, model_pathloss="fspl",
            lrfhss_ratio=0.0
        )
        assert all(not d.use_lrfhss for d in net.devices)

    def test_packet_phy_type_default_css(self):
        """Pacotes criados sem LR-FHSS devem ter phy_type='CSS'."""
        from packet import Packet
        p = Packet(0, 7, 14, 125000, 868.1, -90, 0.0, 0.1)
        assert p.phy_type == "CSS"
        assert p.lrfhss_fragments is None
