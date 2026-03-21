import random
import math
import numpy as np
from enum import Enum
from packet import Packet
from energymodel import EnergyModel
from parametors import (rd1, rd2, trx1, trx2, receive_delay1, receive_delay2,
                        ed_dc_limit_percent, snr_min_per_sf, sensitivity_table)

class RadioState(Enum):
    IDLE = 0
    TX = 1
    WAIT_RX1 = 2
    RX1 = 3
    WAIT_RX2 = 4
    RX2 = 5
    SLEEP = 6

class EndDevice:
    def __init__(self, device_id, x, y, lambda_rate, speed, sf_range, tx_power, bw, cr, pl, frequency_mhz, network, mobility_enabled=True, model="random_walk"):
        self.device_id = device_id
        self.x = x
        self.y = y
        self.lambda_rate = lambda_rate
        self.speed = speed
        self.sf = random.choice(sf_range)
        self.tx_power = tx_power
        self.cr = cr
        self.pl = pl
        self.bw = random.choice(bw)
        self.rssi = None
        self.freq = random.choice(frequency_mhz)
        self.snr = None
        self.coverage_status = True
        self.retransmission_attempts = 0
        self.max_retransmissions = 10
        self.lorawan_class = random.choice(['A', 'B', 'C'])
        self.traffic_type = random.choice(["periodic", "sporadic", "critical"])
        self.network = network
        self.last_transmissions = []
        self.current_gateway = None
        self.snr_history = []
        self.rssi_history = []
        self.adr_enabled = True
        self.airtime = self.calculate_airtime()
        self.mobility_enabled = mobility_enabled
        self.model = model
        self.payload_size = self.calculate_payload_size()
        self.dc_release_time = 0
        self.frame_counter_up = 0
        self.confirmed_ratio = 0.3
        self.energy_model = EnergyModel(voltage=3.3)

        # Sprint 3 - MAC & PHY Enhancements
        self.radio_state = RadioState.IDLE
        self.is_indoor = (random.random() < 0.3)
        self.pending_mac_commands = []

        # Activation mode
        self.activation_mode = "OTAA"  # "OTAA" ou "ABP"
        self.dev_addr = None
        self.session_keys = None

        # OTAA credentials e session keys
        self.app_key = None      # 16 bytes — gerado pelo Network em initialize_devices
        self.dev_eui = None      # 8 bytes — deterministico baseado em device_id
        self.app_eui = None      # 8 bytes — AppEUI do servidor
        self.nwk_skey = None     # 16 bytes — derivado no JoinAccept
        self.app_skey = None     # 16 bytes — derivado no JoinAccept
        self.dev_nonce = 0       # 2 bytes — incrementado a cada tentativa de join
        self.app_nonce = 0       # 3 bytes — recebido no JoinAccept

        # Frame counter validation
        self._last_fc_down = 0

        # Battery model (optional)
        self.battery = None

        # Class B
        self.beacon_period = 128
        self.ping_period = 32
        self.ping_offset = 0
        self.is_synchronized = False
        self.last_beacon_time = 0

        # Class C
        self._class_c_idle_rx = False

        # RX params
        self._rx1_delay = receive_delay1
        self._rx2_delay = receive_delay2
        self._max_duty_cycle = None

        # RxParamSetupReq (MAC 0x05) — parametros RX configurados pelo NS
        self._rx1_dr_offset = 0   # Offset DR RX1 relativo ao UL DR
        self._rx2_freq = None      # None = usa default regional
        self._rx2_sf = None        # None = usa default regional
        self._rx2_bw = None        # None = usa default regional

        # ADR backoff
        self.adr_ack_cnt = 0
        self._adr_changes = 0

        # Available channels for frequency hopping
        self._available_channels = list(frequency_mhz) if isinstance(frequency_mhz, list) else [frequency_mhz]

        # Sprint 6: LR-FHSS
        self.use_lrfhss = False  # Definido pelo Network em initialize_devices

    def set_sensibilidade(self, sf, bw):
        """Retorna a sensibilidade (dBm) para o par SF/BW usando tabela de parametors.py."""
        return sensitivity_table.get((sf, bw), -999)

    def calculate_airtime(self):
        Tsym = (2.0 ** self.sf) / self.bw
        Tpream = (8 + 4.25) * Tsym
        DE = 1 if Tsym > 0.016 else 0
        H, CRC = 0, 1
        numerator = 8.0 * self.pl - 4.0 * self.sf + 28 + 16 * CRC - 20 * H
        denominator = 4.0 * (self.sf - 2 * DE)
        payloadSymbNB = 8 + max(math.ceil(numerator / denominator) * (self.cr + 4), 0)
        return Tpream + payloadSymbNB * Tsym

    def calculate_payload_size(self):
        return self.pl + 13 + 2 + 4

    def validate_payload_size(self, region=None):
        """Valida payload contra limite do DR/regiao. Retorna payload efetivo."""
        if region is None:
            return self.pl
        max_pl = region.get_max_payload(self.sf, self.bw)
        return min(self.pl, max_pl)

    def check_dwell_time(self, region=None):
        """Verifica se airtime respeita dwell time da regiao."""
        if region is None:
            return True
        return region.check_dwell_time(self.airtime)

    def select_channel(self):
        """Seleciona canal para proxima transmissao (round-robin ou random)."""
        if self._available_channels:
            self.freq = random.choice(self._available_channels)
        return self.freq

    def can_receive_downlink(self, current_time):
        if self.lorawan_class == 'A':
            return self.radio_state in [RadioState.RX1, RadioState.RX2]
        elif self.lorawan_class == 'B':
            if not self.is_synchronized:
                return False
            time_since_beacon = current_time - self.last_beacon_time
            ping_phase = time_since_beacon % self.ping_period
            return ping_phase < 0.030  # 30ms ping slot window
        elif self.lorawan_class == 'C':
            return self.radio_state != RadioState.TX
        return False

    def process_beacon(self, beacon_time):
        """Processa beacon recebido (Class B)."""
        self.is_synchronized = True
        self.last_beacon_time = beacon_time
        self.ping_offset = random.randint(0, self.ping_period - 1)

    def get_idle_current(self):
        """Retorna corrente em idle baseado na classe."""
        from parametors import rx_current_ma, sleep_current_ma
        if self.lorawan_class == 'C':
            return rx_current_ma  # Class C mantem RX continuamente
        return sleep_current_ma

    def validate_frame_counter_down(self, fc_down):
        """Valida frame counter de downlink (deve ser crescente)."""
        if fc_down <= self._last_fc_down:
            return False
        self._last_fc_down = fc_down
        return True

    def setup_abp(self, dev_addr, nwk_skey, app_skey):
        """Configura device em modo ABP (sem join)."""
        self.activation_mode = "ABP"
        self.dev_addr = dev_addr
        self.session_keys = {"nwk_skey": nwk_skey, "app_skey": app_skey}

    # Mapeamento FSM (enddevice.RadioState) -> EnergyModel (energymodel.RadioState)
    # Usado por transition_state() para rastrear consumo junto com o estado MAC.
    _ENERGY_STATE_MAP = {
        "IDLE":     "STANDBY",
        "TX":       "TX",
        "WAIT_RX1": "STANDBY",
        "RX1":      "RX",
        "WAIT_RX2": "STANDBY",
        "RX2":      "RX",
        "SLEEP":    "SLEEP",
    }

    def transition_state(self, new_state, time):
        """Transiciona FSM do radio e registra energia no modelo de energia.

        Mapeia RadioState do MAC (7 estados) para RadioState do EnergyModel (4 estados).
        Nao deve ser combinado com update_energy() no mesmo ciclo.
        """
        from energymodel import RadioState as EnergyState
        old_state = self.radio_state
        energy_state_name = self._ENERGY_STATE_MAP.get(new_state.name, "SLEEP")
        self.energy_model.transition(EnergyState[energy_state_name], time)
        self.radio_state = new_state
        return old_state

    def update_coverage_status(self):
        # G10: find_best_gateway ja usa sensibilidade GW (SX1301) internamente
        best_gateway, _, _ = self.network.find_best_gateway(self)
        self.coverage_status = best_gateway is not None

    def move(self, area_size, mobility_enabled, model):
        if not mobility_enabled: return 0, (self.x, self.y)
        time_interval = np.random.exponential(1 / self.lambda_rate)
        angle = random.uniform(0, 2 * np.pi)
        dx, dy = self.speed * np.cos(angle), self.speed * np.sin(angle)
        self.x = max(0, min(area_size, self.x + dx))
        self.y = max(0, min(area_size, self.y + dy))
        return time_interval, (self.x, self.y)

    def get_device_report(self):
        battery_soc = None
        if self.battery is not None:
            battery_soc = self.battery.soc_percent()
        return {
            "Device ID": self.device_id, "SF": self.sf, "Freq (MHz)": self.freq,
            "SNR": self.snr, "RSSI": self.rssi, "Classe LoRaWAN": self.lorawan_class,
            "Energy Consumed (mJ)": self.energy_model.energy_consumed, "Indoor": self.is_indoor,
            "Battery SoC (%)": battery_soc,
            "Activation": self.activation_mode,
        }

    def prepare_join_request(self):
        """Prepara parametros do JoinRequest (OTAA).

        Incrementa DevNonce, computa MIC, retorna dict com parametros.
        Conforme LoRaWAN 1.0.x sec 6.2.
        """
        from security import generate_dev_eui, generate_app_eui, compute_join_mic
        if self.dev_eui is None:
            self.dev_eui = generate_dev_eui(self.device_id)
        if self.app_eui is None:
            self.app_eui = generate_app_eui()
        if self.app_key is None:
            self.app_key = bytes(16)  # fallback

        self.dev_nonce = (self.dev_nonce + 1) & 0xFFFF
        mic = compute_join_mic(self.app_key, self.app_eui, self.dev_eui, self.dev_nonce)
        return {
            'dev_eui': self.dev_eui,
            'app_eui': self.app_eui,
            'dev_nonce': self.dev_nonce,
            'mic': mic,
        }

    def process_join_accept(self, app_nonce, net_id, dev_addr):
        """Processa JoinAccept: deriva session keys, configura DevAddr.

        Conforme LoRaWAN 1.0.x sec 6.2.5.
        Returns True se bem sucedido.
        """
        from security import derive_session_keys
        if self.app_key is None:
            return False

        self.app_nonce = app_nonce
        self.dev_addr = dev_addr  # int 32-bit
        self.nwk_skey, self.app_skey = derive_session_keys(
            self.app_key, app_nonce, net_id, self.dev_nonce)

        # Reset frame counters no join (LoRaWAN spec 4.3.1.5)
        self.frame_counter_up = 0
        self._last_fc_down = 0
        return True
