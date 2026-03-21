"""ADR Component - Adaptive Data Rate com multiplas politicas.

Politicas: average, maximum, minimum, percentile, ewma.
Inclui ADR Backoff (ADR_ACK_LIMIT / ADR_ACK_DELAY).
"""

import numpy as np
from collections import deque
from parametors import snr_min_per_sf, ADR_HISTORY_SIZE, ADR_SNR_MARGIN_DB, ADR_STEP_DB


class ADRComponent:
    """Componente ADR com multiplas politicas e backoff.

    Baseado em ns-3 AdrComponent com extensoes.
    """

    ADR_ACK_LIMIT = 64   # Frames sem DL antes de pedir ADR ACK
    ADR_ACK_DELAY = 32   # Frames adicionais antes de backoff

    def __init__(self, method="average", margin_db=None, history_size=None):
        self.method = method
        self.margin_db = margin_db if margin_db is not None else ADR_SNR_MARGIN_DB
        self.history_size = history_size if history_size is not None else ADR_HISTORY_SIZE
        self.device_histories = {}  # device_id -> deque of SNR values

    def on_packet(self, packet, device_status):
        """Processa pacote e gera LinkAdrReq se necessario."""
        device_id = packet.device_id
        snr = packet.snr

        if snr is None:
            return []

        # Inicializa historico
        if device_id not in self.device_histories:
            self.device_histories[device_id] = deque(maxlen=self.history_size)
        self.device_histories[device_id].append(snr)

        # Aguarda dados suficientes
        if len(self.device_histories[device_id]) < self.history_size:
            return []

        # Computa metrica SNR
        snr_metric = self.compute_snr_metric(self.device_histories[device_id])

        # SNR minimo requerido
        required_snr = snr_min_per_sf.get(packet.sf, -20.0)

        # Margem
        margin = snr_metric - required_snr - self.margin_db

        # Converte em passos
        n_steps = int(margin / ADR_STEP_DB)

        new_sf = packet.sf
        new_tx_power = packet.tx_power

        # Passo 1: Reduzir SF se margem positiva
        while n_steps > 0 and new_sf > 7:
            new_sf -= 1
            n_steps -= 1

        # Passo 2: Reduzir TX power
        while n_steps > 0 and new_tx_power > 2:
            new_tx_power -= 2
            n_steps -= 1

        # Passo 3: Se margem negativa, aumentar TX power
        while n_steps < 0 and new_tx_power < 14:
            new_tx_power += 2
            n_steps += 1

        # Passo 4: Se ainda negativa, aumentar SF
        while n_steps < 0 and new_sf < 12:
            new_sf += 1
            n_steps += 1

        # Gera MAC command se houve mudanca
        if new_sf != packet.sf or new_tx_power != packet.tx_power:
            from mac_commands import LinkAdrReq
            dr = 12 - new_sf  # Converte SF para DR
            return [LinkAdrReq(data_rate=dr, tx_power=new_tx_power, ch_mask=0xFFFF)]

        return []

    def compute_snr_metric(self, snr_history):
        """Computa metrica SNR baseada na politica selecionada."""
        history = list(snr_history)
        if not history:
            return -20

        if self.method == "average":
            return float(np.mean(history))
        elif self.method == "maximum":
            return float(np.max(history))
        elif self.method == "minimum":
            return float(np.min(history))
        elif self.method == "percentile":
            return float(np.percentile(history, 90))
        elif self.method == "ewma":
            alpha = 0.3
            result = history[0]
            for snr in history[1:]:
                result = alpha * snr + (1 - alpha) * result
            return float(result)
        else:
            return float(np.mean(history))

    def check_backoff(self, device_status):
        """Verifica e aplica ADR backoff.

        Retorna (new_tx_power, new_sf) se backoff necessario, None caso contrario.
        """
        if device_status.adr_ack_cnt < self.ADR_ACK_LIMIT:
            return None

        excess = device_status.adr_ack_cnt - self.ADR_ACK_LIMIT
        if excess < self.ADR_ACK_DELAY:
            return None  # Ainda dentro do delay

        # Backoff: aumenta TX power, depois SF
        last_packet = device_status.last_packet
        if last_packet is None:
            return None

        new_tx_power = min(last_packet.tx_power + 2, 14)
        new_sf = last_packet.sf
        if new_tx_power >= 14 and new_sf < 12:
            new_sf += 1

        return new_tx_power, new_sf
