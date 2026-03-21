"""Gateway Manager - gerenciamento de gateways no Network Server.

Implementa:
- Multi-gateway diversity reception
- Selecao de melhor gateway para downlink
- Tracking de duty cycle por gateway
"""

import numpy as np


class GatewayStatus:
    """Status de um gateway no NS."""
    def __init__(self, gateway):
        self.gateway = gateway
        self.last_tx_time = {"rx1": 0, "rx2": 0}
        self.total_dl_sent = 0
        self.duty_cycle_usage = 0


class GatewayManager:
    """Gerencia gateways e seleciona melhor para DL."""

    def __init__(self, gw_rx1_dc=0.01, gw_rx2_dc=0.10):
        self.gateways = {}  # gw_id -> GatewayStatus
        self.rx1_dc = gw_rx1_dc
        self.rx2_dc = gw_rx2_dc

    def register(self, gateway):
        """Registra gateway no manager."""
        self.gateways[gateway.gw_id] = GatewayStatus(gateway)

    def process_uplink_from_all(self, packet, gateways, network):
        """Processa uplink de todos os GWs em alcance.

        Retorna lista de recepcoes com RSSI/SNR por GW.
        """
        from parametors import ed_antenna_gain, gw_antenna_gain

        receptions = []
        for gw in gateways:
            device = next((d for d in network.devices if d.device_id == packet.device_id), None)
            if device is None:
                continue

            distance = max(np.sqrt((device.x - gw.x) ** 2 + (device.y - gw.y) ** 2 +
                                   (network.ht_m - network.hr_m) ** 2), 1.0)
            path_loss = network.pathloss(distance, device.freq, model_pathloss=network.model_pathloss)
            rssi = device.tx_power + ed_antenna_gain + gw_antenna_gain - path_loss
            noise_floor = network.calculate_noise_floor(device.bw)
            snr = rssi - noise_floor
            sensitivity = device.set_sensibilidade(device.sf, device.bw)

            if rssi >= sensitivity:
                receptions.append({
                    "gateway": gw,
                    "rssi": rssi,
                    "snr": snr,
                    "distance": distance,
                })

        return receptions

    def select_best_for_downlink(self, device_id, device_registry):
        """Seleciona melhor gateway para enviar downlink a um device."""
        status = device_registry.get_status(device_id)
        if status is None or not status.gateways:
            return None

        best_gw_id = None
        best_snr = -999

        for gw_id, gw_info in status.gateways.items():
            snr = gw_info.get("snr") or -999
            if snr > best_snr:
                gw_status = self.gateways.get(gw_id)
                if gw_status is not None:
                    best_gw_id = gw_id
                    best_snr = snr

        if best_gw_id is None:
            return None
        return self.gateways[best_gw_id].gateway

    def can_send_downlink(self, gw_id, current_time, airtime, window="rx1"):
        """Verifica se gateway pode enviar DL respeitando duty cycle."""
        gw_status = self.gateways.get(gw_id)
        if gw_status is None:
            return False

        dc_limit = self.rx1_dc if window == "rx1" else self.rx2_dc
        off_time = airtime / dc_limit - airtime
        last_tx = gw_status.last_tx_time.get(window, 0)

        return current_time >= last_tx + off_time

    def record_downlink(self, gw_id, current_time, airtime, window="rx1"):
        """Registra envio de downlink para tracking de duty cycle."""
        gw_status = self.gateways.get(gw_id)
        if gw_status is not None:
            gw_status.last_tx_time[window] = current_time + airtime
            gw_status.total_dl_sent += 1
