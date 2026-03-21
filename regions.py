"""Parametros Regionais LoRaWAN.

Implementa as especificacoes regionais conforme LoRaWAN Regional Parameters v1.0.3.
Regioes suportadas: EU868, US915, AU915, AS923.
"""


class RegionalParameters:
    """Classe base para parametros regionais LoRaWAN."""
    name = "UNKNOWN"
    frequency_range = (0, 0)
    default_channels = []
    additional_channels = []
    rx2_frequency = 0
    rx2_dr = 0
    max_tx_power_dbm = 0
    duty_cycle = None
    max_dwell_time_ms = None
    dr_table = {}
    max_payload_bytes = {}
    join_accept_delay1 = 5
    join_accept_delay2 = 6
    receive_delay1 = 1
    receive_delay2 = 2

    def get_channels(self, include_additional=True):
        """Retorna lista de canais disponiveis."""
        channels = list(self.default_channels)
        if include_additional:
            channels.extend(self.additional_channels)
        return channels

    def sf_to_dr(self, sf, bw):
        """Converte SF/BW para Data Rate index."""
        for dr, params in self.dr_table.items():
            if params["sf"] == sf and params["bw"] == bw:
                return dr
        return 0

    def dr_to_sf_bw(self, dr):
        """Converte Data Rate index para (SF, BW)."""
        params = self.dr_table.get(dr, {"sf": 12, "bw": 125000})
        return params["sf"], params["bw"]

    def get_max_payload(self, sf, bw):
        """Retorna payload maximo em bytes para dado SF/BW."""
        dr = self.sf_to_dr(sf, bw)
        return self.max_payload_bytes.get(dr, 51)

    def get_duty_cycle_limit(self, frequency):
        """Retorna limite de duty cycle para dada frequencia."""
        if self.duty_cycle is None:
            return None
        for (f_low, f_high), dc in self.duty_cycle.items():
            if f_low <= frequency <= f_high:
                return dc
        return 0.01

    def check_dwell_time(self, airtime_s):
        """Verifica se transmissao respeita dwell time."""
        if self.max_dwell_time_ms is None:
            return True
        return (airtime_s * 1000) <= self.max_dwell_time_ms


class EU868(RegionalParameters):
    """Parametros regionais EU868 (Europa 863-870 MHz)."""
    name = "EU868"
    frequency_range = (863, 870)
    default_channels = [868.1, 868.3, 868.5]
    additional_channels = [867.1, 867.3, 867.5, 867.7, 867.9]
    rx2_frequency = 869.525
    rx2_dr = 0
    max_tx_power_dbm = 16
    duty_cycle = {
        (863.0, 868.6): 0.01,    # G/G1 sub-band: canais 868.1/868.3/868.5 + 867.x (G15)
        (868.7, 869.2): 0.001,   # G2 sub-band
        (869.4, 869.65): 0.10,   # G3 sub-band (RX2 channel)
        (869.7, 870.0): 0.01,    # G4 sub-band
    }
    max_dwell_time_ms = None
    dr_table = {
        0: {"sf": 12, "bw": 125000},
        1: {"sf": 11, "bw": 125000},
        2: {"sf": 10, "bw": 125000},
        3: {"sf": 9,  "bw": 125000},
        4: {"sf": 8,  "bw": 125000},
        5: {"sf": 7,  "bw": 125000},
        6: {"sf": 7,  "bw": 250000},
    }
    max_payload_bytes = {0: 51, 1: 51, 2: 51, 3: 115, 4: 222, 5: 222, 6: 222}
    receive_delay1 = 1
    receive_delay2 = 2


class US915(RegionalParameters):
    """Parametros regionais US915 (Americas 902-928 MHz)."""
    name = "US915"
    frequency_range = (902, 928)
    default_channels = [902.3 + 0.2 * i for i in range(64)]
    additional_channels = [903.0 + 1.6 * i for i in range(8)]
    rx2_frequency = 923.3
    rx2_dr = 8
    max_tx_power_dbm = 30
    duty_cycle = None
    max_dwell_time_ms = 400
    dr_table = {
        0: {"sf": 10, "bw": 125000},
        1: {"sf": 9,  "bw": 125000},
        2: {"sf": 8,  "bw": 125000},
        3: {"sf": 7,  "bw": 125000},
        4: {"sf": 8,  "bw": 500000},
        8: {"sf": 12, "bw": 500000},
        9: {"sf": 11, "bw": 500000},
        10: {"sf": 10, "bw": 500000},
        11: {"sf": 9,  "bw": 500000},
        12: {"sf": 8,  "bw": 500000},
        13: {"sf": 7,  "bw": 500000},
    }
    max_payload_bytes = {
        0: 11, 1: 53, 2: 125, 3: 242, 4: 242,
        8: 53, 9: 129, 10: 242, 11: 242, 12: 242, 13: 242,
    }
    downlink_channels = [923.3 + 0.6 * i for i in range(8)]
    receive_delay1 = 1
    receive_delay2 = 2


class AU915(RegionalParameters):
    """Parametros regionais AU915 (Australia 915-928 MHz)."""
    name = "AU915"
    frequency_range = (915, 928)
    default_channels = [915.2 + 0.2 * i for i in range(64)]
    additional_channels = [915.9 + 1.6 * i for i in range(8)]
    rx2_frequency = 923.3
    rx2_dr = 8
    max_tx_power_dbm = 30
    duty_cycle = None
    max_dwell_time_ms = 400
    dr_table = {
        0: {"sf": 12, "bw": 125000},
        1: {"sf": 11, "bw": 125000},
        2: {"sf": 10, "bw": 125000},
        3: {"sf": 9,  "bw": 125000},
        4: {"sf": 8,  "bw": 125000},
        5: {"sf": 7,  "bw": 125000},
        6: {"sf": 8,  "bw": 500000},
        8: {"sf": 12, "bw": 500000},
        9: {"sf": 11, "bw": 500000},
        10: {"sf": 10, "bw": 500000},
        11: {"sf": 9,  "bw": 500000},
        12: {"sf": 8,  "bw": 500000},
        13: {"sf": 7,  "bw": 500000},
    }
    max_payload_bytes = {
        0: 51, 1: 51, 2: 51, 3: 115, 4: 222, 5: 222, 6: 222,
        8: 53, 9: 129, 10: 242, 11: 242, 12: 242, 13: 242,
    }
    downlink_channels = [923.3 + 0.6 * i for i in range(8)]
    receive_delay1 = 1
    receive_delay2 = 2


class AS923(RegionalParameters):
    """Parametros regionais AS923 (Asia 923 MHz)."""
    name = "AS923"
    frequency_range = (923, 925)
    default_channels = [923.2, 923.4]
    additional_channels = [923.6, 923.8, 924.0, 924.2, 924.4, 924.6]
    rx2_frequency = 923.2
    rx2_dr = 2
    max_tx_power_dbm = 16
    duty_cycle = {
        (923.0, 925.0): 0.01,
    }
    max_dwell_time_ms = 400
    dr_table = {
        0: {"sf": 12, "bw": 125000},
        1: {"sf": 11, "bw": 125000},
        2: {"sf": 10, "bw": 125000},
        3: {"sf": 9,  "bw": 125000},
        4: {"sf": 8,  "bw": 125000},
        5: {"sf": 7,  "bw": 125000},
        6: {"sf": 7,  "bw": 250000},
    }
    max_payload_bytes = {0: 51, 1: 51, 2: 51, 3: 115, 4: 222, 5: 222, 6: 222}
    receive_delay1 = 1
    receive_delay2 = 2


REGIONS = {
    "EU868": EU868(),
    "US915": US915(),
    "AU915": AU915(),
    "AS923": AS923(),
}


def get_region(name="EU868"):
    """Retorna instancia de parametros regionais pelo nome."""
    region = REGIONS.get(name.upper())
    if region is None:
        raise ValueError(f"Regiao desconhecida: {name}. Disponiveis: {list(REGIONS.keys())}")
    return region
