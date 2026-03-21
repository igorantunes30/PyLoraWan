"""Link Check Component - verificacao de link.

Processa LinkCheckReq e gera LinkCheckAns com margem e contagem de GWs.
"""

from parametors import snr_min_per_sf


class LinkCheckComponent:
    """Componente de verificacao de link."""

    def __init__(self):
        self.pending_requests = {}  # device_id -> True

    def on_packet(self, packet, device_status):
        """Processa pacote e verifica se ha LinkCheckReq pendente."""
        device_id = packet.device_id

        # Verifica se device tem LinkCheckReq pendente
        if device_id not in self.pending_requests:
            return []

        del self.pending_requests[device_id]

        # Calcula margem
        snr = packet.snr if packet.snr is not None else -20
        snr_required = snr_min_per_sf.get(packet.sf, -20.0)
        margin = max(0, int(snr - snr_required))

        # Conta GWs que receberam
        gw_count = len(device_status.gateways) if device_status else 1

        from mac_commands import LinkCheckAns
        return [LinkCheckAns(margin_db=margin, gw_count=gw_count)]

    def request_link_check(self, device_id):
        """Registra que device solicitou link check."""
        self.pending_requests[device_id] = True
