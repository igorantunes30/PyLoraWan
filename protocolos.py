import random
import numpy as np
from collections import deque
from parametors import snr_min_per_sf, ADR_HISTORY_SIZE, ADR_SNR_MARGIN_DB, ADR_STEP_DB

class LoRaWANMAC:
    def __init__(self, network=None):
        """
        Inicializa o gerenciamento MAC para a rede LoRaWAN.

        Parametros:
        - network: Referencia a rede para acessar dispositivos e configuracoes.
        """
        self.device_sessions = {}  # Sessoes ativas (OTAA)
        self.downlink_queue = {}  # Buffer de pacotes downlink
        self.network = network  # Referencia a rede principal

    def join_request(self, device):
        """
        Simula uma solicitacao OTAA (Over-the-Air Activation) para ingressar na rede.
        """
        if device.device_id not in self.device_sessions:
            self.device_sessions[device.device_id] = {
                "dev_addr": f"00FF{device.device_id:04X}",
                "session_key": f"SK-{device.device_id:04X}",
                "joined": True
            }
            print(f"[LOG LORAMAC] Dispositivo {device.device_id} entrou na rede LoRaWAN.")

    def process_mac_commands(self, device):
        """
        Aplica o ADR (Adaptive Data Rate) baseado no algoritmo ns-3 AdrComponent.
        Ajusta SF e potencia de transmissao com base no SNR medio.
        """
        if device.device_id not in self.device_sessions:
            return

        if not device.adr_enabled:
            return

        # Constantes importadas de parametors.py

        # Inicializa historicos como deque se necessario (BUG-07)
        if not isinstance(device.snr_history, deque):
            device.snr_history = deque(device.snr_history, maxlen=ADR_HISTORY_SIZE)
        if not isinstance(device.rssi_history, deque):
            device.rssi_history = deque(device.rssi_history, maxlen=ADR_HISTORY_SIZE)

        # Atualiza historicos de SNR e RSSI
        if device.snr is not None:
            device.snr_history.append(device.snr)

        if device.rssi is not None:
            device.rssi_history.append(device.rssi)

        # Aguarda dados suficientes
        if len(device.snr_history) < ADR_HISTORY_SIZE:
            return

        # Calcula SNR medio (ns-3 usa mean, NAO max)
        avg_snr = np.mean(device.snr_history)

        # SNR minimo requerido para o SF atual
        required_snr = snr_min_per_sf.get(device.sf, -20.0)

        # Margem disponivel
        margin = avg_snr - required_snr - ADR_SNR_MARGIN_DB

        # Converte margem em passos (1 step = ADR_STEP_DB dB)
        n_steps = int(margin / ADR_STEP_DB)

        new_sf = device.sf
        new_tx_power = device.tx_power

        # Passo 1: Reduzir SF (aumentar DR) se margem positiva
        while n_steps > 0 and new_sf > 7:
            new_sf -= 1
            n_steps -= 1

        # Passo 2: Reduzir potencia TX se SF ja no minimo
        while n_steps > 0 and new_tx_power > 2:
            new_tx_power -= 2  # 2 dB por step (padrao ns-3)
            n_steps -= 1

        # Passo 3: Se margem negativa, aumentar potencia TX
        while n_steps < 0 and new_tx_power < 14:
            new_tx_power += 2
            n_steps += 1
            
        # BUG-03: Se ainda houver margem negativa e TX power no maximo, aumentar SF
        while n_steps < 0 and new_sf < 12:
            new_sf += 1
            n_steps += 1

        # Aplica mudancas se houve alteracao
        if new_tx_power != device.tx_power or new_sf != device.sf:
            print(f"[LOG LORAMAC] ADR Ajustado | Device {device.device_id} -> SF: {new_sf}, Tx Power: {new_tx_power} dBm "
                  f"(avg_snr={avg_snr:.1f}, required={required_snr:.1f}, margin={margin:.1f})")
            device.tx_power = new_tx_power
            device.sf = new_sf
            device.airtime = device.calculate_airtime()

    def handle_ack(self, packet):
        """
        Processa pacotes com flag de ACK, verificando se devem ser retransmitidos.
        """
        if packet.received:
            print(f"[LOG LORAMAC] ACK recebido para o pacote do Dispositivo {packet.device_id}")
            return True
        else:
            print(f"[LOG LORAMAC] Nenhum ACK para Dispositivo {packet.device_id}, retransmitindo...")
            return False

    def manage_downlink(self, device):
        """
        Gerencia o envio de pacotes downlink para um dispositivo.
        """
        if device.device_id in self.downlink_queue:
            downlink_packet = self.downlink_queue.pop(device.device_id)
            print(f"[LOG LORAMAC] Pacote downlink enviado para Dispositivo {device.device_id} via Gateway.")
            self.network.packet_tracker.add_packet(downlink_packet)
            return downlink_packet
        return None

    def send_mac_commands(self):
        """
        Aplica ADR para otimizar a comunicacao dos dispositivos na rede.
        """
        if not self.network.adr_enabled:
            return

        for device in self.network.devices:
            if not device.adr_enabled:
                continue
            self.process_mac_commands(device)

    def check_ack(self, device_id, time):
        """
        Verifica se um ACK foi recebido para um dispositivo.
        Baseado na recepcao real do pacote pelo gateway (nao aleatorio).
        """
        device_packets = self.network.packet_tracker.packet_history.get(device_id, [])
        if not device_packets:
            return False

        last_packet = device_packets[-1]

        # ACK so e enviado para pacotes confirmed que nao colidiram
        if not last_packet.confirmed:
            return False
        if last_packet.collided:
            return False

        # Verifica se esta dentro da janela RX (TX_end + 1s a TX_end + 2.5s)
        tx_end = last_packet.arrival_time + last_packet.rectime
        if time < tx_end + 1.0 or time > tx_end + 2.5:
            return False

        return True
