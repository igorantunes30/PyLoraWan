import csv

class PacketTracker:
    def __init__(self):
        self.packets = []  # 📦 Lista de pacotes originais transmitidos na rede
        self.retransmitted_packets = []  # ♻️ Pacotes retransmitidos
        self.packet_history = {}  # 📜 Histórico de pacotes por dispositivo
        self.total_retransmissions = 0  # 🔄 Contador global de retransmissões
        self.unique_packet_count = 0  # 🔢 Contador de pacotes únicos para rastreamento

    def add_packet(self, packet, is_retransmission=False):
        """
        Adiciona um pacote ao rastreador de pacotes e registra no histórico do dispositivo.
        - Se for retransmissão (`is_retransmission=True`), adiciona à lista de retransmissões.
        """
        if is_retransmission:
            self.retransmitted_packets.append(packet)
            self.total_retransmissions += 1
            print(f"♻️ [LOG] Retransmissão registrada: Dispositivo {packet.device_id} | Total retransmissões: {self.total_retransmissions}")
        else:
            self.packets.append(packet)
            self.unique_packet_count += 1
            print(f"📦 [LOG] Pacote original armazenado: Dispositivo {packet.device_id} | Total pacotes únicos: {self.unique_packet_count}")


        if packet.device_id not in self.packet_history:
            self.packet_history[packet.device_id] = []
        self.packet_history[packet.device_id].append(packet)

    def get_stats(self):
        """Retorna estatísticas gerais sobre a transmissão de pacotes na rede."""
        total_packets = len(self.packets) + len(self.retransmitted_packets)

        if total_packets == 0:
            return {
                "Total Pacotes": 0,
                "Colisões": 0,
                "Retransmissões": 0,
                "Entregues com Sucesso": 0,
                "Taxa de Entrega (PDR)": 0.00
            }


        collided_packets = sum(1 for p in self.packets + self.retransmitted_packets if getattr(p, "collided", False))
        successful_packets = total_packets - collided_packets

        pdr = round((successful_packets / total_packets) * 100, 2) if total_packets > 0 else 0.00


        return {
            "Total Pacotes": total_packets,
            "Colisões": collided_packets,
            "Retransmissões": self.total_retransmissions,
            "Entregues com Sucesso": successful_packets,
            "Taxa de Entrega (PDR)": pdr
        }

    def get_device_stats(self, device_id):
        """
        Retorna estatísticas de pacotes específicos para um dispositivo.
        """
        packets = self.packet_history.get(device_id, [])
        total_packets = len(packets)

        if total_packets == 0:
            return {
                "Device ID": device_id,
                "Total Pacotes": 0,
                "Colisões": 0,
                "Retransmissões": 0,
                "Entregues com Sucesso": 0,
                "Taxa de Entrega (PDR)": "0.00%"
            }

        collided_packets = sum(1 for p in packets if getattr(p, "collided", False))
        retransmissions = sum(1 for p in packets if p in self.retransmitted_packets)
        successful_packets = total_packets - collided_packets

        pdr = round((successful_packets / total_packets) * 100, 2)

        return {
            "Device ID": device_id,
            "Total Pacotes": total_packets,
            "Colisões": collided_packets,
            "Retransmissões": retransmissions,
            "Entregues com Sucesso": successful_packets,
            "Taxa de Entrega (PDR)": f"{pdr:.2f}%"
        }

    def export_device_log(self, network, filename="device_log.csv"):
        """
        Exporta um log detalhado baseado nos pacotes rastreados.

        Parametros:
        - network: Referencia ao objeto Network para acessar devices e posicoes.
        - filename: Nome do arquivo CSV de saida.
        """
        try:
            all_packets = self.packets + self.retransmitted_packets
            if not all_packets:
                print("[LOG PACKETTRACKER] Nenhum pacote registrado. Log nao gerado.")
                return

            with open(filename, mode="w", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(["Tempo", "Device ID", "SF", "TX Power", "RSSI", "SNR",
                                 "Freq", "BW", "Collided", "Confirmed", "Frame Counter"])

                for packet in sorted(all_packets, key=lambda p: p.arrival_time):
                    rssi_str = f"{packet.rssi:.2f}" if packet.rssi is not None else "N/A"
                    snr_str = f"{packet.snr:.2f}" if packet.snr is not None else "N/A"
                    writer.writerow([
                        f"{packet.arrival_time:.3f}",
                        packet.device_id,
                        packet.sf,
                        packet.tx_power,
                        rssi_str,
                        snr_str,
                        packet.freq,
                        packet.bw,
                        packet.collided,
                        packet.confirmed,
                        packet.frame_counter
                    ])

            print(f"[LOG PACKETTRACKER] Log exportado para {filename} com {len(all_packets)} registros.")

        except Exception as e:
            print(f"[LOG PACKETTRACKER] Erro ao exportar log: {e}")


    def clear_packets(self):
        """
        Limpa o histórico de pacotes, incluindo retransmissões.
        """
        self.packets.clear()
        self.retransmitted_packets.clear()
        self.packet_history.clear()
        self.total_retransmissions = 0  #  Zera contador de retransmissões
        self.unique_packet_count = 0  #  Zera contador de pacotes únicos
        print("🧹 Histórico de pacotes e retransmissões limpo.")
