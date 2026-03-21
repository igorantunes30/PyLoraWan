"""Device Registry - equivalente ao NetworkStatus do ns-3.

Mantem registro de todos os devices e seu estado no Network Server.
"""


class DeviceStatus:
    """Status de um device no Network Server."""
    def __init__(self, device_id):
        self.device_id = device_id
        self.dev_addr = None
        self.joined = False
        self.last_packet = None
        self.last_seen_time = 0
        self.gateways = {}  # gw_id -> {rssi, snr, time}
        self.frame_counter_up = 0
        self.frame_counter_down = 0
        self.pending_mac_commands = []
        self.needs_ack = False
        self.adr_ack_cnt = 0  # Contador para ADR backoff

        # OTAA session keys (Sprint 10)
        self.nwk_skey = None    # 16 bytes
        self.app_skey = None    # 16 bytes
        self.dev_addr = 0       # int 32-bit
        self.app_nonce = 0      # int 3-byte


class DeviceRegistry:
    """Registro e status de todos os devices conhecidos pelo NS."""

    def __init__(self):
        self.devices = {}  # device_id -> DeviceStatus

    def register(self, device_id, dev_addr=None):
        """Registra um novo device."""
        status = DeviceStatus(device_id)
        status.dev_addr = dev_addr or f"00FF{device_id:04X}"
        status.joined = True
        self.devices[device_id] = status
        return status

    def update(self, packet, gateway):
        """Atualiza status do device com info do ultimo pacote recebido."""
        device_id = packet.device_id
        status = self.devices.get(device_id)
        if status is None:
            status = self.register(device_id)

        status.last_packet = packet
        status.last_seen_time = packet.arrival_time
        status.gateways[gateway.gw_id] = {
            "rssi": packet.rssi,
            "snr": packet.snr,
            "sinr": packet.sinr,
            "time": packet.arrival_time,
        }

        return status

    def validate_frame_counter(self, device_id, frame_counter):
        """Valida frame counter (deve ser crescente)."""
        status = self.devices.get(device_id)
        if status is None:
            return True

        if frame_counter <= status.frame_counter_up:
            return False  # Replay ou duplicata

        status.frame_counter_up = frame_counter
        return True

    def needs_reply(self, device_id):
        """Verifica se device precisa de resposta (ACK ou MAC command)."""
        status = self.devices.get(device_id)
        if status is None:
            return False
        return status.needs_ack or len(status.pending_mac_commands) > 0

    def get_best_gateway(self, device_id):
        """Retorna o gateway com melhor SNR para dado device."""
        status = self.devices.get(device_id)
        if status is None or not status.gateways:
            return None

        best_gw_id = max(status.gateways, key=lambda g: status.gateways[g].get("snr", -999))
        return best_gw_id, status.gateways[best_gw_id]

    def get_gw_count(self, device_id):
        """Retorna numero de GWs que receberam o ultimo pacote."""
        status = self.devices.get(device_id)
        if status is None:
            return 0
        return len(status.gateways)

    def get_status(self, device_id):
        """Retorna status do device."""
        return self.devices.get(device_id)

    def get_all_devices(self):
        """Retorna lista de todos os device_ids registrados."""
        return list(self.devices.keys())
