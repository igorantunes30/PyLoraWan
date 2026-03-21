from enum import Enum, auto
from parametors import (tx_current_table, rx_current_ma, rx_delay_current_ma,
                        sleep_current_ma, receive_delay1, receive_delay2, trx1, trx2)


class RadioState(Enum):
    """Estados do radio LoRa (baseado em ns-3 LoraRadioEnergyModel)."""
    SLEEP = auto()      # 0.0015 mA (SX1272 datasheet)
    STANDBY = auto()    # 1.4 mA
    TX = auto()         # 28-44 mA (depende da potencia)
    RX = auto()         # 11.2 mA


# Correntes por estado (mA) - valores SX1272 compativeis com ns-3
STATE_CURRENT_MA = {
    RadioState.SLEEP: sleep_current_ma,      # 0.0015 mA
    RadioState.STANDBY: 1.4,                 # 1.4 mA
    RadioState.RX: rx_current_ma,            # 11.2 mA (SX1272)
    RadioState.TX: 38.0,                     # Default, sobrescrito por tx_power
}


class EnergyModel:
    """Modelo de consumo de energia com maquina de estados.

    Rastreia transicoes de estado do radio e calcula energia consumida
    com base na corrente de cada estado e na duracao.
    Compativel com ns-3 LoraRadioEnergyModel.
    """

    def __init__(self, voltage=3.3):
        self.voltage = voltage
        self.energy_consumed = 0.0      # Energia total consumida (mJ)
        self.current_state = RadioState.SLEEP
        self.state_entry_time = 0.0
        self._current_tx_power = 14     # Potencia TX atual (dBm)

        # Historico de estados para debug/analise
        self.state_durations = {state: 0.0 for state in RadioState}

        # Breakdown detalhado de energia por estado
        self.energy_breakdown = {state: 0.0 for state in RadioState}

        # Battery reference (optional)
        self._battery = None

        # Sprint 5: tracking de tempo real de simulacao para contabilizar sleep entre TXs
        self._sim_time_last_tx_end = 0.0

    def set_battery(self, battery):
        """Associa modelo de bateria."""
        self._battery = battery

    def transition(self, new_state, time, tx_power=None):
        """Registra energia do estado anterior e transiciona para novo estado.

        Retorna: Energia consumida no estado anterior (mJ)
        """
        duration = time - self.state_entry_time
        if duration < 0:
            duration = 0

        # Calcula corrente do estado anterior
        if self.current_state == RadioState.TX:
            current_ma = tx_current_table.get(self._current_tx_power,
                                               tx_current_table.get(14, 38))
        else:
            current_ma = STATE_CURRENT_MA[self.current_state]

        # Energia = I * V * t (mA * V * s = mJ)
        energy = current_ma * self.voltage * duration
        self.energy_consumed += energy
        self.energy_breakdown[self.current_state] += energy

        # Registra duracao
        self.state_durations[self.current_state] += duration

        # Consome da bateria se disponivel
        if self._battery is not None and energy > 0:
            self._battery.consume(energy)

        # Transiciona
        self.current_state = new_state
        self.state_entry_time = time
        if tx_power is not None:
            self._current_tx_power = tx_power

        return energy

    def update_energy(self, device, airtime, sim_time=None):
        """Calcula energia de um ciclo TX completo.

        Ciclo: TX -> RX_DELAY1 -> RX1 -> RX_DELAY2 -> RX2 -> SLEEP

        sim_time: tempo real da simulacao no inicio do TX (Sprint 5).
        Se fornecido, contabiliza energia de sleep desde o ultimo ciclo TX.
        """
        if airtime <= 0:
            return

        # Sprint 5: contabiliza energia de SLEEP entre ciclos TX
        if sim_time is not None and sim_time > self._sim_time_last_tx_end:
            sleep_duration = sim_time - self._sim_time_last_tx_end
            sleep_energy = STATE_CURRENT_MA[RadioState.SLEEP] * self.voltage * sleep_duration
            self.energy_consumed += sleep_energy
            self.energy_breakdown[RadioState.SLEEP] += sleep_energy
            self.state_durations[RadioState.SLEEP] += sleep_duration
            if self._battery is not None and sleep_energy > 0:
                self._battery.consume(sleep_energy)

        t = self.state_entry_time

        # TX
        self.transition(RadioState.TX, t, tx_power=device.tx_power)
        t += airtime

        # RX_DELAY1 (standby ate RX1)
        self.transition(RadioState.STANDBY, t)
        t += receive_delay1

        # RX1 (escutando)
        self.transition(RadioState.RX, t)
        t += trx1

        # RX_DELAY2 (standby entre RX1 e RX2)
        self.transition(RadioState.STANDBY, t)
        rx_delay2_duration = max(0, receive_delay2 - receive_delay1 - trx1)
        t += rx_delay2_duration

        # RX2 (escutando)
        self.transition(RadioState.RX, t)
        t += trx2

        # SLEEP (ate proximo ciclo)
        self.transition(RadioState.SLEEP, t)

        # Sprint 5: registra fim do ciclo TX para proximo calculo de sleep
        if sim_time is not None:
            self._sim_time_last_tx_end = sim_time + airtime + receive_delay1 + trx1 + rx_delay2_duration + trx2

    def get_total_consumption_mj(self):
        """Retorna consumo total em mJ."""
        return self.energy_consumed

    def get_total_consumption_j(self):
        """Retorna consumo total em Joules."""
        return self.energy_consumed / 1000.0

    def get_energy_breakdown(self):
        """Retorna breakdown de energia por estado (mJ)."""
        return {s.name: round(e, 3) for s, e in self.energy_breakdown.items()}

    def get_avg_current_ua(self, total_time_s):
        """Retorna corrente media em uA."""
        if total_time_s <= 0:
            return 0
        # E = I * V * t -> I = E / (V * t)
        # mJ / (V * s) = mA -> *1000 = uA
        return (self.energy_consumed / (self.voltage * total_time_s)) * 1000

    def log_energy(self):
        """Exibe o consumo total de energia."""
        print(f"[LOG ENERGY] Consumo total: {self.energy_consumed:.2f} mJ "
              f"({self.energy_consumed/1000:.4f} J)")

    def reset_energy(self):
        """Reseta o consumo de energia para zero."""
        self.energy_consumed = 0
        self.state_durations = {state: 0.0 for state in RadioState}
        self.energy_breakdown = {state: 0.0 for state in RadioState}

    def stats(self):
        """Retorna estatisticas detalhadas de consumo por estado."""
        return {
            "total_energy_mj": round(self.energy_consumed, 3),
            "current_state": self.current_state.name,
            "state_durations_s": {s.name: round(d, 3) for s, d in self.state_durations.items()},
            "energy_breakdown_mj": self.get_energy_breakdown(),
        }
