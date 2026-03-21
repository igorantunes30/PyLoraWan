"""Duty Cycle Component - gerenciamento de duty cycle por sub-banda.

Implementa controle de duty cycle conforme parametros regionais.
"""


class DutyCycleComponent:
    """Gerencia duty cycle por sub-banda para devices e gateways."""

    def __init__(self, region=None):
        self.region = region
        self.sub_band_usage = {}  # (f_low, f_high) -> last_tx_end_time

    def on_packet(self, packet, device_status):
        """Registra uso de sub-banda ao receber pacote."""
        # Component passivo - nao gera MAC commands automaticamente
        return []

    def can_transmit(self, frequency, current_time, airtime):
        """Verifica se transmissao e permitida na sub-banda.

        Parametros:
        - frequency: frequencia em MHz
        - current_time: tempo atual (s)
        - airtime: duracao da transmissao (s)
        """
        if self.region is None or self.region.duty_cycle is None:
            return True

        sub_band = self._get_sub_band(frequency)
        if sub_band is None:
            return True

        dc_limit = self.region.duty_cycle.get(sub_band, 0.01)
        last_tx_end = self.sub_band_usage.get(sub_band, 0)
        required_off_time = airtime / dc_limit - airtime

        return current_time >= last_tx_end + required_off_time

    def record_transmission(self, frequency, current_time, airtime):
        """Registra transmissao para tracking de duty cycle."""
        if self.region is None or self.region.duty_cycle is None:
            return

        sub_band = self._get_sub_band(frequency)
        if sub_band is not None:
            self.sub_band_usage[sub_band] = current_time + airtime

    def _get_sub_band(self, frequency):
        """Encontra sub-banda para dada frequencia."""
        if self.region is None or self.region.duty_cycle is None:
            return None
        for sub_band in self.region.duty_cycle:
            f_low, f_high = sub_band
            if f_low <= frequency <= f_high:
                return sub_band
        return None

    def get_next_available_time(self, frequency, current_time, airtime):
        """Retorna proximo tempo disponivel para transmissao na sub-banda."""
        if self.can_transmit(frequency, current_time, airtime):
            return current_time

        sub_band = self._get_sub_band(frequency)
        if sub_band is None:
            return current_time

        dc_limit = self.region.duty_cycle.get(sub_band, 0.01)
        last_tx_end = self.sub_band_usage.get(sub_band, 0)
        required_off_time = airtime / dc_limit - airtime

        return last_tx_end + required_off_time
