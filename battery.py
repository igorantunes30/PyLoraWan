"""Modelo de Bateria e Energy Harvesting.

Implementa:
- BatteryModel: capacidade, deplecao, SoC, estimativa de lifetime
- EnergyHarvester: modelos solar e constante
"""

import numpy as np


class BatteryModel:
    """Modelo de bateria com capacidade finita e deplecao.

    Compativel com ns-3 BasicEnergySource.
    """

    def __init__(self, capacity_mah=2400, voltage=3.3):
        self.capacity_mah = capacity_mah
        self.voltage = voltage
        self.capacity_mj = capacity_mah * voltage * 3.6  # mAh -> mJ
        self.remaining_mj = self.capacity_mj
        self.depleted = False
        self.harvester = None
        self._consumption_log = []

    def consume(self, energy_mj):
        """Consome energia da bateria.

        Retorna True se energia foi consumida com sucesso, False se bateria esgotada.
        """
        if self.depleted:
            return False

        self.remaining_mj -= energy_mj
        self._consumption_log.append(energy_mj)

        if self.remaining_mj <= 0:
            self.remaining_mj = 0
            self.depleted = True
            return False
        return True

    def harvest(self, power_mw, duration_s):
        """Adiciona energia colhida a bateria."""
        energy_mj = power_mw * duration_s
        self.remaining_mj = min(self.remaining_mj + energy_mj, self.capacity_mj)
        if self.remaining_mj > 0:
            self.depleted = False

    def soc_percent(self):
        """Retorna State of Charge em percentual."""
        return (self.remaining_mj / self.capacity_mj) * 100

    def remaining_mah(self):
        """Retorna capacidade restante em mAh."""
        return self.remaining_mj / (self.voltage * 3.6)

    def estimate_lifetime_days(self, avg_consumption_mj_per_hour):
        """Estima tempo de vida restante em dias."""
        if avg_consumption_mj_per_hour <= 0:
            return float('inf')
        hours = self.remaining_mj / avg_consumption_mj_per_hour
        return hours / 24

    def stats(self):
        """Retorna estatisticas da bateria."""
        total_consumed = self.capacity_mj - self.remaining_mj
        return {
            "capacity_mah": self.capacity_mah,
            "remaining_mah": self.remaining_mah(),
            "soc_percent": self.soc_percent(),
            "total_consumed_mj": total_consumed,
            "depleted": self.depleted,
        }


class EnergyHarvester:
    """Modelo de energy harvesting (solar, constante).

    Diferencial: nenhum dos 4 simuladores comparados possui energy harvesting.
    """

    def __init__(self, model="solar", peak_power_mw=100):
        self.model = model
        self.peak_power_mw = peak_power_mw
        self.total_harvested_mj = 0

    def get_power(self, time_of_day_hours):
        """Retorna potencia instantanea em mW para dado horario.

        Parametros:
        - time_of_day_hours: hora do dia (0-24)
        """
        if self.model == "solar":
            hour = time_of_day_hours % 24
            if 6 <= hour <= 18:
                return self.peak_power_mw * np.sin(np.pi * (hour - 6) / 12)
            return 0
        elif self.model == "constant":
            return self.peak_power_mw
        return 0

    def harvest_energy(self, battery, sim_time_s, duration_s, day_length_s=86400):
        """Colhe energia e carrega bateria.

        Parametros:
        - battery: BatteryModel a ser carregado
        - sim_time_s: tempo atual da simulacao em segundos
        - duration_s: duracao do periodo de harvesting
        - day_length_s: duracao do dia em segundos (default 86400)
        """
        time_of_day = (sim_time_s % day_length_s) / 3600.0
        power = self.get_power(time_of_day)
        energy = power * duration_s
        self.total_harvested_mj += energy
        battery.harvest(power, duration_s)
        return energy
