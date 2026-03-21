"""LR-FHSS (Long Range Frequency Hopping Spread Spectrum).

Diferencial unico: nenhum dos 4 simuladores comparados combina LoRa CSS + LR-FHSS.
Implementa:
- LRFHSS_PHY: fragmentacao, hopping sequences, calculo ToA
- LRFHSS_Channel: avaliacao de colisao por fragmento
- ACRDA: Asynchronous Coded Random Access com SIC iterativo
"""

import math
import random


class LRFHSSFragment:
    """Representa um fragmento LR-FHSS (header ou payload)."""
    def __init__(self, frag_type, channel, duration, packet_id):
        self.frag_type = frag_type  # 'header' ou 'payload'
        self.channel = channel
        self.duration = duration
        self.packet_id = packet_id
        self.collided = False
        self.tx_start = 0
        self.tx_end = 0


class LRFHSS_PHY:
    """Camada fisica LR-FHSS.

    Parametros baseados no LR-FHSS-sim e especificacao Semtech.
    """

    # Code rates disponiveis
    CODE_RATES = {
        "1/3": {"payload_bytes_per_frag": 2, "threshold_div": 3},
        "2/3": {"payload_bytes_per_frag": 4, "threshold_div": 3},
        "1/2": {"payload_bytes_per_frag": 3, "threshold_div": 2},
    }

    def __init__(self, code_rate="1/3", obw=35, num_headers=3):
        self.code_rate = code_rate
        self.obw = obw  # Occupied Bandwidth (numero de canais de hopping)
        self.num_headers = num_headers
        self.header_duration = 0.233472   # segundos
        self.payload_duration = 0.1024    # segundos
        self.transceiver_wait = 0.006472  # segundos

        cr_params = self.CODE_RATES.get(code_rate, self.CODE_RATES["1/3"])
        self._bytes_per_frag = cr_params["payload_bytes_per_frag"]
        self._threshold_div = cr_params["threshold_div"]

    def fragment_packet(self, payload_size):
        """Fragmenta pacote em headers e payloads.

        Retorna (n_headers, n_payloads, threshold).
        threshold = minimo de payloads necessarios para decodificacao.
        """
        n_payloads = math.ceil((payload_size + 3) / self._bytes_per_frag)
        threshold = math.ceil(n_payloads / self._threshold_div)
        return self.num_headers, n_payloads, threshold

    def calculate_toa(self, payload_size):
        """Calcula Time-on-Air para pacote LR-FHSS."""
        h, p, _ = self.fragment_packet(payload_size)
        total_frags = h + p
        return (h * self.header_duration +
                p * self.payload_duration +
                total_frags * self.transceiver_wait)

    def generate_hopping_sequence(self, n_fragments):
        """Gera sequencia de canais aleatorios para hopping."""
        return [random.randint(0, self.obw - 1) for _ in range(n_fragments)]

    def create_fragments(self, packet_id, payload_size, tx_start):
        """Cria lista de fragmentos com canais e tempos atribuidos."""
        n_headers, n_payloads, _ = self.fragment_packet(payload_size)
        total_frags = n_headers + n_payloads
        hopping = self.generate_hopping_sequence(total_frags)

        fragments = []
        t = tx_start

        # Headers primeiro
        for i in range(n_headers):
            frag = LRFHSSFragment('header', hopping[i], self.header_duration, packet_id)
            frag.tx_start = t
            frag.tx_end = t + self.header_duration
            fragments.append(frag)
            t += self.header_duration + self.transceiver_wait

        # Payloads
        for i in range(n_payloads):
            idx = n_headers + i
            frag = LRFHSSFragment('payload', hopping[idx], self.payload_duration, packet_id)
            frag.tx_start = t
            frag.tx_end = t + self.payload_duration
            fragments.append(frag)
            t += self.payload_duration + self.transceiver_wait

        return fragments


class LRFHSS_Channel:
    """Modelo de canal LR-FHSS com colisao por fragmento."""

    def __init__(self):
        self.active_fragments = []  # (fragment, tx_start, tx_end)
        self.total_collisions = 0
        self.total_receptions = 0

    def add_fragments(self, fragments):
        """Registra fragmentos de um pacote como on-air."""
        for frag in fragments:
            self.active_fragments.append((frag, frag.tx_start, frag.tx_end))

    def check_fragment_collisions(self, fragments):
        """Verifica colisao de cada fragmento com outros on-air."""
        for frag in fragments:
            for (other, other_start, other_end) in self.active_fragments:
                if other.packet_id == frag.packet_id:
                    continue
                if other.channel != frag.channel:
                    continue
                # Sobreposicao temporal
                if frag.tx_end <= other_start or frag.tx_start >= other_end:
                    continue
                frag.collided = True
                break

    def evaluate_packet(self, fragments, threshold):
        """Avalia se pacote pode ser decodificado com decodificacao parcial.

        Precisa de pelo menos 1 header e 'threshold' payloads sem colisao.
        """
        h_success = sum(1 for f in fragments if f.frag_type == 'header' and not f.collided)
        p_success = sum(1 for f in fragments if f.frag_type == 'payload' and not f.collided)

        success = h_success >= 1 and p_success >= threshold
        if success:
            self.total_receptions += 1
        else:
            self.total_collisions += 1
        return success

    def cleanup_expired(self, current_time):
        """Remove fragmentos expirados."""
        self.active_fragments = [
            (f, s, e) for (f, s, e) in self.active_fragments if e > current_time
        ]

    def stats(self):
        return {
            "total_collisions": self.total_collisions,
            "total_receptions": self.total_receptions,
            "active_fragments": len(self.active_fragments),
        }


class ACRDA:
    """Asynchronous Coded Random Access com SIC (Successive Interference Cancellation).

    Cancelamento de interferencia iterativo dentro de janelas temporais.
    """

    def __init__(self, window_size=3):
        self.window_size = window_size
        self.decoded_packets = set()
        self.packet_fragments = {}  # packet_id -> list of fragments

    def register_packet(self, packet_id, fragments):
        """Registra fragmentos de um pacote para processamento SIC."""
        self.packet_fragments[packet_id] = fragments

    def process_window(self, channel):
        """Executa SIC iterativo dentro da janela.

        1. Encontra pacotes decodificaveis
        2. Remove interferencia desses pacotes
        3. Repete ate nao haver novas decodificacoes
        """
        new_recovery = True
        iterations = 0

        while new_recovery:
            new_recovery = False
            iterations += 1

            for packet_id, fragments in list(self.packet_fragments.items()):
                if packet_id in self.decoded_packets:
                    continue

                # Conta fragmentos nao colididos
                h_ok = sum(1 for f in fragments if f.frag_type == 'header' and not f.collided)
                p_ok = sum(1 for f in fragments if f.frag_type == 'payload' and not f.collided)

                # Calcula threshold
                n_payloads = sum(1 for f in fragments if f.frag_type == 'payload')
                threshold = max(1, n_payloads // 3)

                if h_ok >= 1 and p_ok >= threshold:
                    self.decoded_packets.add(packet_id)
                    self._cancel_interference(packet_id, channel)
                    new_recovery = True

        return len(self.decoded_packets), iterations

    def _cancel_interference(self, packet_id, channel):
        """Remove interferencia do pacote decodificado em outros fragmentos."""
        decoded_frags = self.packet_fragments.get(packet_id, [])

        for dec_frag in decoded_frags:
            for other_id, other_frags in self.packet_fragments.items():
                if other_id == packet_id or other_id in self.decoded_packets:
                    continue
                for other_frag in other_frags:
                    if not other_frag.collided:
                        continue
                    if other_frag.channel != dec_frag.channel:
                        continue
                    # Verifica sobreposicao temporal
                    if (dec_frag.tx_end <= other_frag.tx_start or
                            dec_frag.tx_start >= other_frag.tx_end):
                        continue
                    # Cancela a interferencia (simplificado)
                    other_frag.collided = False

    def reset(self):
        """Reseta estado do ACRDA para nova janela."""
        self.decoded_packets.clear()
        self.packet_fragments.clear()
