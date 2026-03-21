import math
import parametors
from parametors import interference_matrix, interference_matrix_goursaud, snr_min_per_sf


class ChannelModel:
    """Modelo de canal LoRa com deteccao de colisao baseada em SINR.

    Gerencia todas as transmissoes on-air e avalia recepcao usando
    a matriz de interferencia (capture effect) e limiar de SNR.
    Baseado nos modelos de ns-3 LoraInterferenceHelper e FLoRa LoRaReceiver.
    """

    def __init__(self, network):
        self.network = network
        self.on_air = []  # Lista de (packet, tx_start, tx_end)
        self.total_collisions = 0
        self.total_receptions = 0

    def add_transmission(self, packet, tx_start, tx_end):
        """Registra um pacote como estando on-air de tx_start a tx_end."""
        self.on_air.append((packet, tx_start, tx_end))

    def evaluate_reception(self, packet, gateway):
        """Avalia se um pacote e recebido com sucesso pelo gateway.

        G14 — Modelo baseado em energia acumulada (ns-3 LoraInterferenceHelper):
        1. SNR >= SNR minimo para o SF (sinal vs ruido termico)
        2. Capture effect com razao de energia: E_signal/E_interferer >= threshold
           E_signal = P_signal * T_packet
           E_interferer = P_interferer * T_overlap
           → energy_ratio_db = (RSSI_signal - RSSI_interferer) + 10*log10(T_packet/T_overlap)
        Suporta matrizes Semtech AN1200.18 e Goursaud (ns-3 default).
        """
        tx_start = packet.arrival_time
        tx_end = tx_start + packet.rectime
        packet_duration = tx_end - tx_start

        # 1. Verifica SNR (sinal vs ruido termico)
        noise_floor = self.network.calculate_noise_floor(packet.bw)
        # Sprint 9: MRC — usa SNR combinado se disponivel (melhor que SNR de um unico GW)
        if getattr(packet, 'snr_mrc', None) is not None:
            snr = packet.snr_mrc
        else:
            snr = packet.rssi - noise_floor if packet.rssi is not None else -999
        packet.snr = snr
        packet.noise_floor = noise_floor

        snr_required = snr_min_per_sf.get(packet.sf, -20.0)
        if snr < snr_required:
            packet.collided = True
            self.total_collisions += 1
            return False

        # G13 — Preamble lock: interferentes que chegam apos lock nao afetam sincronizacao
        Tsym = (2.0 ** packet.sf) / packet.bw
        preamble_lock_time = tx_start + 6 * Tsym

        # Seleciona matriz conforme configuracao (lida em runtime para testabilidade)
        matrix = (interference_matrix_goursaud
                  if parametors.interference_model == "goursaud"
                  else interference_matrix)

        # 2. Encontra interferentes (mesma frequencia, sobreposicao temporal)
        interferers = []
        for (other_pkt, other_start, other_end) in self.on_air:
            if other_pkt.packet_id == packet.packet_id:
                continue
            if other_pkt.freq != packet.freq:
                continue
            overlap_start = max(tx_start, other_start)
            overlap_end = min(tx_end, other_end)
            if overlap_end <= overlap_start:
                continue
            # G13: interferente chegou apos lock do preambulo — nao afeta
            if other_start >= preamble_lock_time:
                continue
            overlap_duration = overlap_end - overlap_start
            interferers.append((other_pkt, overlap_duration, other_start))

        # 3. Sem interferentes: recepcao bem sucedida
        if not interferers:
            packet.collided = False
            self.total_receptions += 1
            return True

        # 4. G14 — Avalia capture effect com razao de energia acumulada (ns-3)
        survived = True
        for interferer, overlap_duration, _ in interferers:
            sf_target_idx = 12 - packet.sf
            sf_inter_idx = 12 - interferer.sf
            threshold_db = matrix[sf_target_idx][sf_inter_idx]

            if packet.rssi is not None and interferer.rssi is not None:
                # Razao de energia: E_signal / E_interferer
                # E_signal = P_signal * T_packet  |  E_interferer = P_interferer * T_overlap
                # energy_ratio = (P_signal/P_interferer) * (T_packet/T_overlap)
                # em dB: rssi_diff + 10*log10(T_packet/T_overlap)
                energy_correction_db = 10.0 * math.log10(
                    packet_duration / max(overlap_duration, 1e-9))
                energy_ratio_db = (packet.rssi - interferer.rssi) + energy_correction_db

                if energy_ratio_db < threshold_db:
                    survived = False
                    break

        packet.collided = not survived
        if survived:
            self.total_receptions += 1
        else:
            self.total_collisions += 1

        return survived

    def cleanup_expired(self, current_time):
        """Remove transmissoes que ja terminaram."""
        self.on_air = [(p, s, e) for (p, s, e) in self.on_air if e > current_time]

    def get_on_air_count(self):
        """Retorna o numero de transmissoes ativas."""
        return len(self.on_air)

    def get_active_on_frequency(self, freq, current_time):
        """Retorna pacotes ativos em dada frequencia."""
        return [(p, s, e) for (p, s, e) in self.on_air
                if p.freq == freq and e > current_time]

    def stats(self):
        """Retorna estatisticas do canal."""
        return {
            "total_collisions": self.total_collisions,
            "total_receptions": self.total_receptions,
            "on_air_count": len(self.on_air),
        }
