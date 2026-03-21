import numpy as np

class NetworkThroughput:
    def __init__(self, network):
        """
        Inicializa o cálculo da vazão da rede.

        Parâmetros:
        - network: Referência ao objeto da classe Network para acessar dispositivos e parâmetros da rede.
        """
        self.network = network

    def calculate_sf_proportions(self):
        """
        Calcula a proporção de dispositivos que utilizam cada Spreading Factor (SF).

        Retorna:
        - Um dicionário {SF: proporção} indicando a porcentagem de dispositivos usando cada SF.
        """
        total_devices = len(self.network.devices)
        sf_counts = {sf: 0 for sf in range(7, 13)}

        for device in self.network.devices:
            sf_counts[device.sf] += 1

        # 🔹 Evita divisão por zero
        sf_proportions = {sf: (sf_counts[sf] / total_devices) if total_devices > 0 else 0 for sf in sf_counts}

        print(f"📊 [LOG NETWORKTHROUGHPUT] Proporção de dispositivos para SF: {sf_proportions}")
        return sf_proportions

    def calculate_avg_airtime(self):
        """
        Calcula o tempo médio de transmissão no ar (ToA) para cada SF.

        Retorna:
        - Um dicionário {SF: ToA médio} indicando a média de airtime por SF.
        """
        avg_airtime = {}
        for sf in range(7, 13):  # SF7 a SF12
            devices_with_sf = [device.calculate_airtime() for device in self.network.devices if device.sf == sf]
            avg_airtime[sf] = np.mean(devices_with_sf) if devices_with_sf else 0

        print(f"📊 [LOG NETWORKTHROUGHPUT] Tempo médio de transmissão (ToA) por SF: {avg_airtime}")
        return avg_airtime

    def calculate_covered_devices(self):
        """
        Conta o número total de dispositivos dentro da cobertura.

        Retorna:
        - Número total de dispositivos cobertos.
        """
        covered_devices = sum(1 for device in self.network.devices if device.coverage_status)

        return covered_devices

    def calculate_traffic_load(self):
        """
        Calcula a carga de tráfego G para cada SF.

        Retorna:
        - Um dicionário {SF: G} com a carga de tráfego por Spreading Factor.
        """
        G = {}
        sf_proportions = self.calculate_sf_proportions()
        avg_airtime = self.calculate_avg_airtime()

        for sf in range(7, 13):  # SF7 a SF12
            Nc = sum(1 for device in self.network.devices if device.sf == sf)  # Número de nós com este SF
            Ps = sf_proportions[sf]  # Proporção de nós com este SF
            toa = avg_airtime[sf]  # Tempo médio de transmissão
            G[sf] = 46 * Nc * Ps * toa  # Carga de tráfego para este SF

        print(f"📊 [LOG NETWORKTHROUGHPUT] Carga de tráfego G por SF: {G}")
        return G

    def calculate_total_throughput(self):
        """
        Calcula a vazão total T da rede.

        Retorna:
        - Vazão total da rede em bits por segundo.
        """
        sf_proportions = self.calculate_sf_proportions()  # Ps[sf]
        avg_airtime = self.calculate_avg_airtime()  # ToA médio por SF
        G = self.calculate_traffic_load()  # Carga de tráfego para cada SF
        Nc = self.calculate_covered_devices()  # Número total de nós cobertos


        if Nc == 0:
            print("⚠️ [LOG NETWORKTHROUGHPUT] Nenhum dispositivo dentro da cobertura. Vazão total = 0.")
            return 0


        T = sum(
            46 * sf_proportions[sf] * Nc * np.exp(-2 * G[sf] / 10000)
            for sf in G if G[sf] > 0
        )

        print(f"📡 [LOG NETWORKTHROUGHPUT] Vazão total da rede: {T:.2f} bits/s")
        return T
