"""Testes de modelo de canal e colisoes."""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import math
import pytest
from channel import ChannelModel
from packet import Packet
import parametors


class MockNetwork:
    def calculate_noise_floor(self, bw):
        return -120  # dBm para 125kHz (aproximado)


class TestChannelModel:
    def setup_method(self):
        self.network = MockNetwork()
        self.channel = ChannelModel(self.network)

    def _make_packet(self, device_id, sf, freq, rssi, time, duration):
        p = Packet(device_id, sf, 14, 125000, freq, rssi, time, duration)
        return p

    def test_no_interference_success(self):
        """Pacote sem interferentes deve ser recebido."""
        p = self._make_packet(0, 7, 868.1, -90, 0, 0.05)
        self.channel.add_transmission(p, 0, 0.05)
        assert self.channel.evaluate_reception(p, None)
        assert not p.collided

    def test_snr_below_threshold_fails(self):
        """Pacote com SNR abaixo do minimo deve falhar."""
        p = self._make_packet(0, 7, 868.1, -130, 0, 0.05)
        self.channel.add_transmission(p, 0, 0.05)
        assert not self.channel.evaluate_reception(p, None)
        assert p.collided

    def test_different_frequencies_no_collision(self):
        """Pacotes em frequencias diferentes nao colidem."""
        p1 = self._make_packet(0, 7, 868.1, -90, 0, 0.1)
        p2 = self._make_packet(1, 7, 868.3, -90, 0, 0.1)
        self.channel.add_transmission(p1, 0, 0.1)
        self.channel.add_transmission(p2, 0, 0.1)
        assert self.channel.evaluate_reception(p1, None)
        assert not p1.collided

    def test_no_temporal_overlap_no_collision(self):
        """Pacotes sem sobreposicao temporal nao colidem."""
        p1 = self._make_packet(0, 7, 868.1, -90, 0, 0.05)
        p2 = self._make_packet(1, 7, 868.1, -90, 1.0, 0.05)
        self.channel.add_transmission(p1, 0, 0.05)
        self.channel.add_transmission(p2, 1.0, 1.05)
        assert self.channel.evaluate_reception(p1, None)

    def test_capture_effect_strong_survives(self):
        """Pacote muito mais forte que interferente deve sobreviver (capture effect)."""
        p1 = self._make_packet(0, 7, 868.1, -80, 0, 0.1)  # Forte
        p2 = self._make_packet(1, 7, 868.1, -110, 0, 0.1) # Fraco
        self.channel.add_transmission(p1, 0, 0.1)
        self.channel.add_transmission(p2, 0, 0.1)
        assert self.channel.evaluate_reception(p1, None)

    def test_cleanup(self):
        """Cleanup deve remover pacotes expirados."""
        p = self._make_packet(0, 7, 868.1, -90, 0, 0.05)
        self.channel.add_transmission(p, 0, 0.05)
        assert self.channel.get_on_air_count() == 1
        self.channel.cleanup_expired(1.0)
        assert self.channel.get_on_air_count() == 0

    def test_stats(self):
        """Stats deve retornar contadores validos."""
        stats = self.channel.stats()
        assert "total_collisions" in stats
        assert "total_receptions" in stats


class TestEnergyBasedInterference:
    """G14 — Testes do modelo de interferencia baseado em energia acumulada (ns-3)."""

    def setup_method(self):
        self.network = MockNetwork()
        self.channel = ChannelModel(self.network)

    def _make_packet(self, device_id, sf, freq, rssi, time, duration):
        p = Packet(device_id, sf, 14, 125000, freq, rssi, time, duration)
        return p

    def test_partial_overlap_advantage(self):
        """Com sobreposicao parcial, sinal tem vantagem de energia sobre interferente.

        Se p1 dura 1s e p2 sobrepos por apenas 0.1s (10%):
        energy_ratio_db = (RSSI_p1 - RSSI_p2) + 10*log10(1.0/0.1) = diff + 10dB
        Mesmo com RSSI igual, p1 sobrevive por ter mais energia total.
        """
        # p1: SF7, dura 1s, RSSI = -90 dBm
        p1 = self._make_packet(0, 7, 868.1, -90, 0.0, 1.0)
        # p2: SF7, sobreposicao de 0.1s no inicio de p1
        p2 = self._make_packet(1, 7, 868.1, -90, 0.0, 0.1)

        self.channel.add_transmission(p1, 0.0, 1.0)
        self.channel.add_transmission(p2, 0.0, 0.1)

        # energy_ratio = (-90 - -90) + 10*log10(1.0/0.1) = 0 + 10 = +10 dB
        # threshold co-SF (Semtech) = 1 dB → +10 dB > 1 dB → p1 sobrevive
        received = self.channel.evaluate_reception(p1, None)
        assert received, "p1 deve sobreviver: energia acumulada supera interferente parcial"

    def test_full_overlap_equal_rssi_collision(self):
        """Com sobreposicao total e RSSI igual, co-SF causa colisao.

        energy_ratio = (RSSI - RSSI) + 10*log10(1.0) = 0 dB
        threshold co-SF Semtech = 1 dB → 0 < 1 → colisao.
        """
        p1 = self._make_packet(0, 7, 868.1, -90, 0.0, 0.1)
        p2 = self._make_packet(1, 7, 868.1, -90, 0.0, 0.1)

        self.channel.add_transmission(p1, 0.0, 0.1)
        self.channel.add_transmission(p2, 0.0, 0.1)

        received = self.channel.evaluate_reception(p1, None)
        assert not received, "p1 deve colidir: energy_ratio 0 dB < threshold 1 dB"

    def test_stronger_signal_survives_full_overlap(self):
        """Sinal 15 dB mais forte sobrevive a interferente co-SF em sobreposicao total."""
        p1 = self._make_packet(0, 7, 868.1, -80, 0.0, 0.1)   # forte
        p2 = self._make_packet(1, 7, 868.1, -95, 0.0, 0.1)   # fraco (-15 dB)

        self.channel.add_transmission(p1, 0.0, 0.1)
        self.channel.add_transmission(p2, 0.0, 0.1)

        # energy_ratio = (-80 - -95) + 0 = 15 dB > 1 dB (threshold co-SF Semtech)
        received = self.channel.evaluate_reception(p1, None)
        assert received

    def test_energy_correction_formula(self):
        """Verifica que a correcao de energia e 10*log10(T_packet/T_overlap)."""
        T_packet = 1.0
        T_overlap = 0.25  # 25% do pacote
        expected_correction = 10 * math.log10(T_packet / T_overlap)
        assert abs(expected_correction - 6.02) < 0.1  # ~6 dB

    def test_goursaud_matrix_stricter_co_sf(self):
        """Matriz Goursaud tem threshold co-SF de 6 dB (mais restritivo que Semtech 1 dB)."""
        original = parametors.interference_model
        try:
            parametors.interference_model = "goursaud"
            channel = ChannelModel(self.network)

            # Sinal 4 dB mais forte: sobreviveria com Semtech (>1dB) mas nao com Goursaud (>6dB)
            p1 = self._make_packet(0, 7, 868.1, -86, 0.0, 0.1)
            p2 = self._make_packet(1, 7, 868.1, -90, 0.0, 0.1)

            channel.add_transmission(p1, 0.0, 0.1)
            channel.add_transmission(p2, 0.0, 0.1)

            received = channel.evaluate_reception(p1, None)
            # energy_ratio = 4 dB < 6 dB (Goursaud co-SF) → colisao
            assert not received, "Goursaud co-SF 6dB threshold deve causar colisao"
        finally:
            parametors.interference_model = original

    def test_semtech_matrix_less_strict_co_sf(self):
        """Mesma configuracao do teste Goursaud, mas com Semtech (threshold=1dB): sobrevive."""
        original = parametors.interference_model
        try:
            parametors.interference_model = "semtech"
            channel = ChannelModel(self.network)

            # Sinal 4 dB mais forte com Semtech threshold=1dB → deve sobreviver
            p1 = self._make_packet(0, 7, 868.1, -86, 0.0, 0.1)
            p2 = self._make_packet(1, 7, 868.1, -90, 0.0, 0.1)

            channel.add_transmission(p1, 0.0, 0.1)
            channel.add_transmission(p2, 0.0, 0.1)

            received = channel.evaluate_reception(p1, None)
            # energy_ratio = 4 dB > 1 dB (Semtech) → sobrevive
            assert received, "Semtech co-SF 1dB: sinal 4dB mais forte deve sobreviver"
        finally:
            parametors.interference_model = original

    def test_cross_sf_orthogonality(self):
        """SF12 vs SF7 interferente: alta ortogonalidade permite recepcao mesmo com RSSI igual."""
        # SF12 alvo vs SF7 interferente: threshold Semtech = -25 dB
        # energy_ratio = 0 dB >> -25 dB → SF12 sobrevive facilmente
        p1 = self._make_packet(0, 12, 868.1, -90, 0.0, 2.0)   # SF12
        p2 = self._make_packet(1, 7,  868.1, -90, 0.0, 0.056)  # SF7 (ToA tipico)

        self.channel.add_transmission(p1, 0.0, 2.0)
        self.channel.add_transmission(p2, 0.0, 0.056)

        received = self.channel.evaluate_reception(p1, None)
        # energy_ratio ≈ 0 + 10*log10(2.0/0.056) ≈ 0 + 15.5 dB > -25 dB
        assert received, "SF12 deve sobreviver a SF7 (alta ortogonalidade)"

    def test_interference_per_sf_on_packet(self):
        """Pacote deve ter atributo interference_per_sf (set pelo gateway.process_uplink)."""
        p = Packet(0, 7, 14, 125000, 868.1, -90, 0.0, 0.1)
        # Atributo deve existir no Packet
        assert hasattr(p, 'interference_per_sf')
        assert isinstance(p.interference_per_sf, dict)
