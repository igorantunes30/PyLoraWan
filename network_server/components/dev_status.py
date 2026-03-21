"""DevStatus Component - solicita e registra status dos devices.

Gera DevStatusReq a cada N uplinks por device.
Armazena historico de bateria e SNR margin reportados.
"""


class DevStatusComponent:
    """Solicita DevStatusReq periodicamente e armazena respostas.

    Compativel com ns-3 DeviceStatusRequestComponent.
    """

    PERIOD = 20  # Solicita a cada 20 uplinks por device

    def __init__(self, period=None):
        self.period = period if period is not None else self.PERIOD
        self.uplink_counts = {}   # device_id -> int
        self.status_log = {}      # device_id -> list of {battery, margin, time}

    def on_packet(self, packet, device_status):
        """Gera DevStatusReq a cada PERIOD uplinks."""
        device_id = packet.device_id
        self.uplink_counts[device_id] = self.uplink_counts.get(device_id, 0) + 1

        if self.uplink_counts[device_id] % self.period == 0:
            from mac_commands import DevStatusReq
            return [DevStatusReq()]
        return []

    def record_status(self, device_id, battery, snr_margin, time):
        """Registra DevStatusAns recebido do device."""
        if device_id not in self.status_log:
            self.status_log[device_id] = []
        self.status_log[device_id].append({
            "battery": battery,
            "snr_margin": snr_margin,
            "time": time,
        })

    def get_last_status(self, device_id):
        """Retorna ultimo status reportado pelo device, ou None."""
        log = self.status_log.get(device_id, [])
        return log[-1] if log else None
