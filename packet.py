import uuid

class Packet:
    def __init__(self, device_id, sf,tx_power, bw, freq, rssi, arrival_time, rectime, packet_type="uplink"):
        """
        Inicializa um pacote LoRaWAN.

        Parâmetros:
        - device_id: ID do dispositivo que transmite o pacote
        - sf: Spreading Factor do dispositivo
        - bw: Largura de banda usada na transmissão
        - freq: Frequência da transmissão
        - rssi: Intensidade do sinal recebido (RSSI)
        - arrival_time: Tempo de chegada do pacote
        - rectime: Tempo de recepção do pacote
        - packet_type: Tipo do pacote (uplink, downlink, critical)
        """
        self.packet_id = uuid.uuid4()
        self.device_id = device_id
        self.sf = sf
        self.tx_power = tx_power
        self.bw = bw
        self.freq = freq
        self.rssi = rssi
        self.arrival_time = arrival_time
        self.rectime = rectime
        self.tx_start = arrival_time
        self.tx_end = arrival_time + rectime
        self.collided = False
        self.received = True
        self.packet_type = packet_type
        self.sir = None
        self.snr = None
        self.snr_mrc = None
        self.mrc_gw_count = 1
        self.sinr = None
        self.noise_floor = None
        self.confirmed = False
        self.ack_received = False
        self.mic = None          # 4 bytes — MIC do frame (computado pelo device)
        self.mic_valid = None    # bool — resultado da verificacao no NS (None=nao verificado)
        self.is_retransmission = False
        self.retry_count = 0
        self.gateway_id = None
        self.frame_counter = 0
        # Sprint 6: LR-FHSS
        self.phy_type = "CSS"           # "CSS" ou "LR-FHSS"
        self.lrfhss_fragments = None    # Lista de LRFHSSFragment quando phy_type="LR-FHSS"
        # G14: interferencia por SF (energy-based)
        self.interference_per_sf = {}   # sf -> potencia linear acumulada (weighted by overlap_ratio)

    def __repr__(self):
            """Representação amigável do pacote para debug e logs."""
            sinr_str = f"{self.sinr:.2f}" if self.sinr is not None else "N/A"
            snr_str = f"{self.snr:.2f}" if self.snr is not None else "N/A"
            nf_str = f"{self.noise_floor:.2f}" if self.noise_floor is not None else "N/A"
            rssi_str = f"{self.rssi:.2f}" if self.rssi is not None else "N/A"
            return (f"<Packet ID={self.packet_id} | Device={self.device_id} | SF={self.sf} | TP: {self.tx_power} "
                    f"RSSI={rssi_str} dBm | SINR={sinr_str} dB | SNR={snr_str} dB | "
                    f"NF={nf_str} dBm | Collided={self.collided} | Type={self.packet_type}>")
