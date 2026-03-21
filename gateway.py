import random
import numpy as np
from parametors import gw_antenna_gain, ed_antenna_gain

class ReceptionPath:
    """Representa um demodulador SX1301 (1 dos 8 reception paths)."""
    def __init__(self):
        self.busy = False
        self.packet = None
        self.tx_end = 0

    def is_free(self, current_time):
        return not self.busy or self.tx_end <= current_time

    def assign(self, packet, tx_end):
        self.busy = True
        self.packet = packet
        self.tx_end = tx_end

    def release(self, current_time):
        if self.tx_end <= current_time:
            self.busy = False
            self.packet = None


class Gateway:
    def __init__(self, gw_id, x, y, network, max_capacity=100, num_reception_paths=8):
        self.gw_id = gw_id
        self.x = x
        self.y = y
        self.network = network
        self.received_packets = []
        self.max_capacity = max_capacity
        self.assigned_frequencies = set()

        # SX1301: 8 reception paths (demoduladores)
        self.num_reception_paths = num_reception_paths
        self.reception_paths = [ReceptionPath() for _ in range(num_reception_paths)]
        self.saturation_events = 0

        # Duty cycle tracking para downlink
        self.last_dl_time = {"rx1": 0, "rx2": 0}
        self.total_dl_sent = 0

        # G8: rastreia por quanto tempo o GW esta em TX de DL por frequencia
        self.dl_busy_until = {}  # freq -> tx_end_time

    def try_allocate_path(self, packet, current_time):
        """Tenta alocar um reception path livre para o pacote.

        Retorna True se path alocado, False se gateway saturado.
        """
        tx_end = current_time + packet.rectime
        for path in self.reception_paths:
            if path.is_free(current_time):
                path.assign(packet, tx_end)
                return True
        self.saturation_events += 1
        return False

    def release_paths(self, current_time):
        """Libera reception paths cujos pacotes ja terminaram."""
        for path in self.reception_paths:
            path.release(current_time)

    def active_paths_count(self, current_time):
        """Retorna numero de paths atualmente em uso."""
        return sum(1 for p in self.reception_paths if not p.is_free(current_time))

    def process_uplink(self, packet):
        """Processa um pacote uplink recebido de um dispositivo."""
        if not packet or not hasattr(packet, 'device_id'):
            print(f"[LOG GATEWAY] Pacote invalido recebido no Gateway {self.gw_id}. Ignorando...")
            return

        device = next((d for d in self.network.devices if d.device_id == packet.device_id), None)
        if not device:
            print(f"[LOG GATEWAY] Dispositivo {packet.device_id} nao encontrado na rede.")
            return

        # Verifica reception paths (SX1301)
        current_time = packet.arrival_time

        # G8: DL-UL interference — GW nao pode receber UL enquanto transmite DL na mesma freq
        if self.dl_busy_until.get(packet.freq, 0) > current_time:
            packet.collided = True
            return

        if not self.try_allocate_path(packet, current_time):
            print(f"[LOG GATEWAY] Gateway {self.gw_id} saturado! Todos os {self.num_reception_paths} paths ocupados.")
            packet.collided = True
            return

        distance = max(np.sqrt((device.x - self.x) ** 2 + (device.y - self.y) ** 2 +
                               (self.network.ht_m - self.network.hr_m) ** 2), 1.0)

        # G2: passa coordenadas do device para correlated_shadowing
        path_loss = self.network.pathloss(distance, device.freq,
                                          model_pathloss=self.network.model_pathloss,
                                          device_x=device.x, device_y=device.y)
        # G3: adiciona perda de penetracao predial para dispositivos indoor
        path_loss += self.network.get_building_penetration(device)
        # BUG-08 FIX: inclui ganho de antena no link budget
        packet.rssi = device.tx_power + ed_antenna_gain + gw_antenna_gain - path_loss

        # G14 — Interferencia baseada em energia acumulada por SF (ns-3 approach)
        interference_power_linear = 0
        interference_per_sf = {}  # sf -> energia linear acumulada (para analise por SF)

        for (other_pkt, tx_start, tx_end) in self.network.channel.on_air:
            if other_pkt.packet_id == packet.packet_id:
                continue
            if other_pkt.freq != packet.freq:
                continue
            # Verifica sobreposicao temporal
            if tx_end <= current_time or tx_start >= current_time + packet.rectime:
                continue

            # Calcula tempo de sobreposicao (energia acumulada, nao apenas ratio)
            overlap_start = max(current_time, tx_start)
            overlap_end = min(current_time + packet.rectime, tx_end)
            overlap_duration = overlap_end - overlap_start
            overlap_ratio = overlap_duration / packet.rectime if packet.rectime > 0 else 1.0

            # Calcula RSSI do interferente neste gateway
            other_device = next((d for d in self.network.devices if d.device_id == other_pkt.device_id), None)
            if not other_device:
                continue

            other_distance = max(np.sqrt((other_device.x - self.x) ** 2 + (other_device.y - self.y) ** 2 +
                                         (self.network.ht_m - self.network.hr_m) ** 2), 1.0)
            other_path_loss = self.network.pathloss(other_distance, other_pkt.freq,
                                                    model_pathloss=self.network.model_pathloss,
                                                    device_x=other_device.x, device_y=other_device.y)
            other_path_loss += self.network.get_building_penetration(other_device)
            other_rssi_dbm = other_pkt.tx_power + ed_antenna_gain + gw_antenna_gain - other_path_loss
            other_rssi_linear = 10 ** (other_rssi_dbm / 10)

            # Interferencia proporcional ao overlap (= E_interferer / T_packet)
            interference_power_linear += other_rssi_linear * overlap_ratio

            # Rastreia por SF para analise detalhada (G14)
            sf_key = other_pkt.sf
            interference_per_sf[sf_key] = (interference_per_sf.get(sf_key, 0)
                                           + other_rssi_linear * overlap_ratio)

        packet.interference_per_sf = interference_per_sf

        # Calculo do noise floor
        packet.noise_floor = self.network.calculate_noise_floor(device.bw)
        noise_linear = 10 ** (packet.noise_floor / 10)
        signal_linear = 10 ** (packet.rssi / 10)

        # SINR = S / (I + N) - BUG-02 FIX
        sinr_linear = signal_linear / (interference_power_linear + noise_linear)
        packet.sinr = 10 * np.log10(sinr_linear)

        # SIR e SNR para retrocompatibilidade
        if interference_power_linear > 0:
            packet.sir = packet.rssi - 10 * np.log10(interference_power_linear)
        else:
            packet.sir = float('inf')

        packet.snr = packet.rssi - packet.noise_floor
        device.snr = packet.snr

        print(f"[LOG GATEWAY] Pacote do Dispositivo {packet.device_id} recebido pelo Gateway {self.gw_id} | "
              f"RSSI: {packet.rssi:.2f} dBm | SNR: {packet.snr:.2f} dB | SINR: {packet.sinr:.2f} dB | "
              f"Paths: {self.active_paths_count(current_time)}/{self.num_reception_paths}")

        if len(self.received_packets) >= self.max_capacity:
            print(f"[LOG GATEWAY] Gateway {self.gw_id} sobrecarregado! Tentando balancear carga...")
            self.balance_load(packet, device)
            return

        self.received_packets.append(packet)

    def balance_load(self, packet, device):
        """Redireciona pacotes para outro gateway menos congestionado."""
        closest_gateway, _, _ = self.network.find_best_gateway(device, avoid_congestion=True)

        if closest_gateway:
            closest_gateway.received_packets.append(packet)
            print(f"[LOG GATEWAY] Pacote do dispositivo {packet.device_id} encaminhado de Gateway {self.gw_id} -> {closest_gateway.gw_id}")
        else:
            print(f"[LOG GATEWAY] Nenhum gateway disponivel para balanceamento. Pacote processado localmente.")
            self.received_packets.append(packet)

    def relay_packet(self, packet):
        """Retransmissao de pacote para outro gateway."""
        alternative_gateway = self.network.find_best_gateway(avoid_congestion=True)
        if alternative_gateway:
            alternative_gateway.received_packets.append(packet)

    def dynamic_frequency_assignment(self):
        """Ajusta frequencias dos gateways para minimizar interferencias."""
        if not self.network.frequency_mhz:
            return

        self.assigned_frequencies.clear()
        available_frequencies = set(self.network.frequency_mhz)
        used_frequencies = {packet.freq for packet in self.received_packets}

        free_frequencies = available_frequencies - used_frequencies
        if free_frequencies:
            new_freq = random.choice(list(free_frequencies))
            self.assigned_frequencies.add(new_freq)

    def clear_old_packets(self):
        """Remove pacotes antigos para liberar espaco."""
        if len(self.received_packets) > self.max_capacity:
            self.received_packets = self.received_packets[-self.max_capacity:]
