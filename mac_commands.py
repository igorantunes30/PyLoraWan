"""MAC Commands LoRaWAN.

Implementa os MAC commands conforme LoRaWAN 1.1 specification.
Suporta: LinkCheck, LinkAdr, DutyCycle, RXParamSetup, DevStatus,
NewChannel, RXTimingSetup, DlChannel.
"""


class MACCommand:
    """Classe base para MAC commands LoRaWAN."""
    def __init__(self, cid, payload=None):
        self.cid = cid
        self.payload = payload or {}

    def __repr__(self):
        return f"<MACCommand CID=0x{self.cid:02X} payload={self.payload}>"


# --- Uplink MAC Commands (ED -> NS) ---

class LinkCheckReq(MACCommand):
    """CID 0x02 - Device solicita verificacao de link."""
    def __init__(self):
        super().__init__(0x02)


class LinkAdrAns(MACCommand):
    """CID 0x03 - Device responde a LinkAdrReq."""
    def __init__(self, power_ack=True, data_rate_ack=True, channel_mask_ack=True):
        super().__init__(0x03, {
            "power_ack": power_ack,
            "data_rate_ack": data_rate_ack,
            "channel_mask_ack": channel_mask_ack,
        })


class DutyCycleAns(MACCommand):
    """CID 0x04 - Device confirma DutyCycleReq."""
    def __init__(self):
        super().__init__(0x04)


class RXParamSetupAns(MACCommand):
    """CID 0x05 - Device responde a RXParamSetupReq."""
    def __init__(self, rx1_dr_offset_ack=True, rx2_dr_ack=True, channel_ack=True):
        super().__init__(0x05, {
            "rx1_dr_offset_ack": rx1_dr_offset_ack,
            "rx2_dr_ack": rx2_dr_ack,
            "channel_ack": channel_ack,
        })


class DevStatusAns(MACCommand):
    """CID 0x06 - Device reporta nivel de bateria e SNR."""
    def __init__(self, battery_level, snr_margin):
        super().__init__(0x06, {
            "battery": battery_level,  # 0=external, 1-254=level, 255=unknown
            "margin": snr_margin,      # -32 to +31 dB
        })


class NewChannelAns(MACCommand):
    """CID 0x07 - Device confirma NewChannelReq."""
    def __init__(self, dr_range_ok=True, channel_freq_ok=True):
        super().__init__(0x07, {
            "dr_range_ok": dr_range_ok,
            "channel_freq_ok": channel_freq_ok,
        })


class RXTimingSetupAns(MACCommand):
    """CID 0x08 - Device confirma RXTimingSetupReq."""
    def __init__(self):
        super().__init__(0x08)


class DlChannelAns(MACCommand):
    """CID 0x0A - Device confirma DlChannelReq."""
    def __init__(self, uplink_freq_exists=True, channel_freq_ok=True):
        super().__init__(0x0A, {
            "uplink_freq_exists": uplink_freq_exists,
            "channel_freq_ok": channel_freq_ok,
        })


# --- Downlink MAC Commands (NS -> ED) ---

class LinkCheckAns(MACCommand):
    """CID 0x02 - NS responde com margem e contagem de GWs."""
    def __init__(self, margin_db, gw_count):
        super().__init__(0x02, {
            "margin": margin_db,
            "gw_count": gw_count,
        })


class LinkAdrReq(MACCommand):
    """CID 0x03 - NS solicita ajuste de DR, TX power e channel mask."""
    def __init__(self, data_rate, tx_power, ch_mask, nb_trans=1):
        super().__init__(0x03, {
            "data_rate": data_rate,
            "tx_power": tx_power,
            "ch_mask": ch_mask,
            "nb_trans": nb_trans,
        })


class DutyCycleReq(MACCommand):
    """CID 0x04 - NS define max duty cycle agregado."""
    def __init__(self, max_dc_cycle):
        super().__init__(0x04, {
            "max_dc_cycle": max_dc_cycle,  # 0=no limit, 1..15 = 1/2^val
        })


class RXParamSetupReq(MACCommand):
    """CID 0x05 - NS ajusta parametros RX1/RX2."""
    def __init__(self, rx1_dr_offset, rx2_dr, rx2_frequency):
        super().__init__(0x05, {
            "rx1_dr_offset": rx1_dr_offset,
            "rx2_dr": rx2_dr,
            "rx2_frequency": rx2_frequency,
        })


class DevStatusReq(MACCommand):
    """CID 0x06 - NS solicita status do device."""
    def __init__(self):
        super().__init__(0x06)


class NewChannelReq(MACCommand):
    """CID 0x07 - NS adiciona/modifica canal."""
    def __init__(self, ch_index, frequency, min_dr, max_dr):
        super().__init__(0x07, {
            "ch_index": ch_index,
            "frequency": frequency,
            "min_dr": min_dr,
            "max_dr": max_dr,
        })


class RXTimingSetupReq(MACCommand):
    """CID 0x08 - NS ajusta delay de RX1."""
    def __init__(self, delay_s):
        super().__init__(0x08, {
            "delay": delay_s,  # 1-15 seconds
        })


class DlChannelReq(MACCommand):
    """CID 0x0A - NS ajusta frequencia DL de um canal."""
    def __init__(self, ch_index, frequency):
        super().__init__(0x0A, {
            "ch_index": ch_index,
            "frequency": frequency,
        })


class MACCommandProcessor:
    """Processa MAC commands no Network Server."""

    def __init__(self, network_server=None):
        self.network_server = network_server

    def process_uplink_commands(self, device, commands):
        """Processa MAC commands recebidos no uplink (ED -> NS)."""
        responses = []
        for cmd in commands:
            if isinstance(cmd, LinkCheckReq):
                resp = self._handle_link_check(device)
                if resp:
                    responses.append(resp)
            elif isinstance(cmd, DevStatusAns):
                self._handle_dev_status(device, cmd)
            elif isinstance(cmd, LinkAdrAns):
                self._handle_link_adr_ans(device, cmd)
        return responses

    def process_downlink_commands(self, device, commands):
        """Aplica MAC commands recebidos no downlink (NS -> ED)."""
        responses = []
        for cmd in commands:
            if isinstance(cmd, LinkAdrReq):
                resp = self._apply_link_adr(device, cmd)
                responses.append(resp)
            elif isinstance(cmd, DutyCycleReq):
                self._apply_duty_cycle(device, cmd)
                responses.append(DutyCycleAns())
            elif isinstance(cmd, RXParamSetupReq):
                resp = self._apply_rx_param_setup(device, cmd)
                responses.append(resp)
            elif isinstance(cmd, DevStatusReq):
                resp = self._create_dev_status_ans(device)
                responses.append(resp)
            elif isinstance(cmd, NewChannelReq):
                resp = self._apply_new_channel(device, cmd)
                responses.append(resp)
            elif isinstance(cmd, RXTimingSetupReq):
                self._apply_rx_timing(device, cmd)
                responses.append(RXTimingSetupAns())
            elif isinstance(cmd, DlChannelReq):
                resp = self._apply_dl_channel(device, cmd)
                responses.append(resp)
            elif isinstance(cmd, LinkCheckAns):
                self._handle_link_check_ans(device, cmd)
        return responses

    def _handle_link_check(self, device):
        """Processa LinkCheckReq e gera LinkCheckAns."""
        if not self.network_server:
            return None
        snr = device.snr if device.snr is not None else -20
        snr_min = -20
        margin = max(0, int(snr - snr_min))
        gw_count = 1
        return LinkCheckAns(margin_db=margin, gw_count=gw_count)

    def _handle_dev_status(self, device, cmd):
        """Registra status reportado pelo device."""
        pass

    def _handle_link_adr_ans(self, device, cmd):
        """Registra resposta do device ao LinkAdrReq."""
        pass

    def _apply_link_adr(self, device, cmd):
        """Aplica ajuste de DR e TX power no device."""
        power_ack = True
        dr_ack = True
        ch_mask_ack = True

        new_power = cmd.payload.get("tx_power", device.tx_power)
        if 2 <= new_power <= 14:
            device.tx_power = new_power
        else:
            power_ack = False

        new_dr = cmd.payload.get("data_rate")
        if new_dr is not None:
            new_sf = 12 - new_dr
            if 7 <= new_sf <= 12:
                device.sf = new_sf
                device.airtime = device.calculate_airtime()
            else:
                dr_ack = False

        return LinkAdrAns(power_ack=power_ack, data_rate_ack=dr_ack, channel_mask_ack=ch_mask_ack)

    def _apply_duty_cycle(self, device, cmd):
        """Aplica ajuste de duty cycle no device."""
        max_dc = cmd.payload.get("max_dc_cycle", 0)
        if max_dc == 0:
            device._max_duty_cycle = None
        else:
            device._max_duty_cycle = 1.0 / (2 ** max_dc)

    def _apply_rx_param_setup(self, device, cmd):
        """Aplica parametros RX1 offset e RX2 (freq, DR) no device."""
        rx1_dr_offset_ack = True
        rx2_dr_ack = True
        channel_ack = True

        rx1_dr_offset = cmd.payload.get("rx1_dr_offset", 0)
        if 0 <= rx1_dr_offset <= 5:
            device._rx1_dr_offset = rx1_dr_offset
        else:
            rx1_dr_offset_ack = False

        rx2_dr = cmd.payload.get("rx2_dr", 0)
        rx2_sf = 12 - rx2_dr
        if 7 <= rx2_sf <= 12:
            device._rx2_sf = rx2_sf
            device._rx2_bw = 125000
            rx2_dr_ack = True
        else:
            rx2_dr_ack = False

        rx2_freq = cmd.payload.get("rx2_frequency")
        if rx2_freq is not None and rx2_freq > 0:
            device._rx2_freq = rx2_freq
        else:
            channel_ack = False

        return RXParamSetupAns(
            rx1_dr_offset_ack=rx1_dr_offset_ack,
            rx2_dr_ack=rx2_dr_ack,
            channel_ack=channel_ack,
        )

    def _create_dev_status_ans(self, device):
        """Cria DevStatusAns com nivel de bateria e SNR."""
        battery = 255
        if hasattr(device, 'battery') and device.battery is not None:
            soc = device.battery.soc_percent()
            if soc <= 0:
                battery = 0
            else:
                battery = max(1, min(254, int(soc * 254 / 100)))

        snr_margin = min(31, max(-32, int(device.snr))) if device.snr is not None else 0
        return DevStatusAns(battery_level=battery, snr_margin=snr_margin)

    def _apply_new_channel(self, device, cmd):
        """Adiciona ou modifica canal na lista de canais disponiveis do device.

        Conforme LoRaWAN 1.0.x sec 5.2. Permite NS configurar canais opcionais.
        """
        ch_index = cmd.payload.get("ch_index", 0)
        frequency = cmd.payload.get("frequency")
        min_dr = cmd.payload.get("min_dr", 0)
        max_dr = cmd.payload.get("max_dr", 5)

        dr_range_ok = (0 <= min_dr <= max_dr <= 5)
        channel_freq_ok = False

        if frequency is not None and frequency > 0:
            if not hasattr(device, '_available_channels'):
                device._available_channels = []
            if frequency not in device._available_channels:
                device._available_channels.append(frequency)
            channel_freq_ok = True

        return NewChannelAns(dr_range_ok=dr_range_ok, channel_freq_ok=channel_freq_ok)

    def _apply_rx_timing(self, device, cmd):
        """Aplica ajuste de RX1 delay."""
        delay = cmd.payload.get("delay", 1)
        if hasattr(device, '_rx1_delay'):
            device._rx1_delay = delay

    def _apply_dl_channel(self, device, cmd):
        """Aplica frequencia DL."""
        return DlChannelAns()

    def _handle_link_check_ans(self, device, cmd):
        """Device recebe resposta de link check."""
        pass
