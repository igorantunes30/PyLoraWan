"""Downlink Scheduler - escalonador de pacotes downlink.

Implementa fila de prioridades para downlink com suporte a:
- ACK (prioridade mais alta)
- MAC Commands
- Application Data (prioridade mais baixa)
"""

import heapq


class DownlinkPacket:
    """Pacote downlink."""
    def __init__(self, device_id, payload=None, packet_type="ack"):
        self.device_id = device_id
        self.payload = payload or b''
        self.packet_type = packet_type  # "ack", "mac_command", "app_data"


class DownlinkScheduler:
    """Escalonador de downlink com fila de prioridades."""

    PRIORITY_ACK = 0
    PRIORITY_MAC_COMMAND = 1
    PRIORITY_APP_DATA = 2

    def __init__(self):
        self.queue = []
        self._counter = 0
        self.total_scheduled = 0
        self.total_sent = 0
        self.total_dropped = 0

    def schedule(self, device_id, packet, priority=None):
        """Agenda pacote downlink com prioridade.

        Parametros:
        - device_id: ID do device destino
        - packet: DownlinkPacket
        - priority: 0=ACK, 1=MAC, 2=APP (auto-detecta se None)
        """
        if priority is None:
            if packet.packet_type == "ack":
                priority = self.PRIORITY_ACK
            elif packet.packet_type == "mac_command":
                priority = self.PRIORITY_MAC_COMMAND
            else:
                priority = self.PRIORITY_APP_DATA

        self._counter += 1
        heapq.heappush(self.queue, (priority, self._counter, device_id, packet))
        self.total_scheduled += 1

    def get_next(self, device_id=None):
        """Retorna proximo pacote a enviar.

        Se device_id fornecido, retorna proximo para aquele device.
        """
        if device_id is None:
            if self.queue:
                _, _, dev_id, packet = heapq.heappop(self.queue)
                self.total_sent += 1
                return dev_id, packet
            return None, None

        # Busca pacote para device especifico
        for i, (prio, cnt, dev_id, packet) in enumerate(self.queue):
            if dev_id == device_id:
                self.queue.pop(i)
                heapq.heapify(self.queue)
                self.total_sent += 1
                return dev_id, packet
        return None, None

    def has_pending(self, device_id=None):
        """Verifica se ha pacotes pendentes."""
        if device_id is None:
            return len(self.queue) > 0
        return any(dev_id == device_id for (_, _, dev_id, _) in self.queue)

    def pending_count(self):
        """Retorna numero de pacotes pendentes."""
        return len(self.queue)

    def stats(self):
        return {
            "total_scheduled": self.total_scheduled,
            "total_sent": self.total_sent,
            "total_dropped": self.total_dropped,
            "pending": len(self.queue),
        }
