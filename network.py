import numpy as np
import random
import csv
import logging
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from collections import Counter
from enddevice import EndDevice, RadioState
from gateway import Gateway
from packet import Packet
from packettracker import PacketTracker
from protocolos import LoRaWANMAC
from throughput import NetworkThroughput
from event_scheduler import EventScheduler, EventType
from channel import ChannelModel
from regions import get_region, EU868
from deployment import deploy_grid, deploy_circular, deploy_hexagonal, deploy_random_uniform, deploy_annular, deploy_clustered, deploy_from_file
from battery import BatteryModel, EnergyHarvester
from network_server import NetworkServer
from parametors import *
from parametors import (random_seed, interference_matrix, interference_matrix_goursaud,
                        interference_model, receive_delay1,
                        receive_delay2, trx1, trx2, ed_dc_limit_percent,
                        gw_antenna_gain, ed_antenna_gain, gw_sensitivity_table,
                        lrfhss_ratio, lrfhss_code_rate, lrfhss_obw,
                        lrfhss_num_headers, lrfhss_acrda_window)
from lrfhss import LRFHSS_PHY, LRFHSS_Channel, ACRDA
from security import generate_app_key, generate_dev_eui, generate_app_eui

# --- Logging Estruturado ---
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(levelname)s: %(message)s")
phy_logger = logging.getLogger("pylorawan.phy")
mac_logger = logging.getLogger("pylorawan.mac")
adr_logger = logging.getLogger("pylorawan.adr")
energy_logger = logging.getLogger("pylorawan.energy")
network_logger = logging.getLogger("pylorawan.network")


class CorrelatedShadowingMap:
    """Implementa shadowing espacialmente correlacionado usando interpolacao bilinear.

    Aproxima o modelo ns-3 CorrelatedShadowingPropagationLossModel com
    correlacao exponencial (d_corr=110m). Cada ponto do grid recebe um
    valor de shadowing aleatorio; posicoes intermediarias sao interpoladas.
    """
    def __init__(self, area_size, grid_step=50, sigma=3.57):
        self.area_size = area_size
        self.grid_step = grid_step
        self.grid_points = int(area_size / grid_step) + 1
        self.grid = np.random.normal(0, sigma, (self.grid_points, self.grid_points))

    def get_shadowing(self, x, y):
        # G2: clamp para garantir coordenadas dentro da area
        x = max(0.0, min(float(self.area_size), float(x)))
        y = max(0.0, min(float(self.area_size), float(y)))

        gx, gy = x / self.grid_step, y / self.grid_step
        x0, y0 = int(gx), int(gy)
        x1, y1 = min(x0 + 1, self.grid_points - 1), min(y0 + 1, self.grid_points - 1)
        dx, dy = gx - x0, gy - y0

        v00 = self.grid[x0, y0]
        v10 = self.grid[x1, y0]
        v01 = self.grid[x0, y1]
        v11 = self.grid[x1, y1]
        return (1-dx)*(1-dy)*v00 + dx*(1-dy)*v10 + (1-dx)*dy*v01 + dx*dy*v11


class Network:
    def __init__(self, num_devices, num_gateways, area_size, lambda_rate, speed,
                 sf_range, tx_power, frequency_mhz, ht_m, hr_m, bw, cr, pl,
                 simulation_time, adr_enabled, model_pathloss,
                 region_name="EU868", deployment_type="grid",
                 deployment_radius=None, deployment_file=None,
                 battery_capacity_mah=None,
                 energy_harvesting=None, adr_method="average",
                 indoor_ratio=0.3, correlated_shadowing=False,
                 server_side_adr=True, lrfhss_ratio=lrfhss_ratio):
        """Inicializa a rede LoRaWAN com todos os componentes."""
        # Parametros da rede
        self.num_devices = num_devices
        self.num_gateways = num_gateways
        self.area_size = area_size
        self.lambda_rate = lambda_rate
        self.speed = speed
        self.sf_range = sf_range
        self.tx_power = tx_power
        self.frequency_mhz = frequency_mhz
        self.ht_m = ht_m
        self.hr_m = hr_m
        self.bw = bw
        self.cr = cr
        self.pl = pl
        self.simulation_time = simulation_time
        self.adr_enabled = adr_enabled
        self.model_pathloss = model_pathloss
        self.mobility_enabled = mobility_enabled
        self.indoor_ratio = indoor_ratio

        # Regional parameters
        self.region = get_region(region_name)

        # Deployment
        self.deployment_type = deployment_type
        self.deployment_radius = deployment_radius or (area_size / 2)
        self.deployment_file = deployment_file

        # Battery & Energy Harvesting
        self.battery_capacity_mah = battery_capacity_mah
        self.energy_harvesting_config = energy_harvesting

        # ADR method
        self.adr_method = adr_method

        # G5: server-side ADR via NS (LinkAdrReq) — desativa device-side ADR redundante
        self.server_side_adr = server_side_adr

        # Correlated shadowing
        self.correlated_shadowing = correlated_shadowing
        self.shadowing_map = None
        if correlated_shadowing:
            self.shadowing_map = CorrelatedShadowingMap(area_size)

        # Sprint 6: LR-FHSS
        self.lrfhss_ratio = lrfhss_ratio
        self.lrfhss_phy = LRFHSS_PHY(
            code_rate=lrfhss_code_rate,
            obw=lrfhss_obw,
            num_headers=lrfhss_num_headers
        )
        self.lrfhss_channel = LRFHSS_Channel()
        self.acrda = ACRDA(window_size=lrfhss_acrda_window)

        # Building penetration
        self._building_penetration_cache = {}

        # DL-DL interference: rastreia downlinks ativos por frequencia (network-wide)
        self._network_dl_busy = {}  # freq -> dl_end_time

        # Seed para reproducibilidade
        if random_seed is not None:
            random.seed(random_seed)
            np.random.seed(random_seed)

        # Componentes da simulacao
        self.devices = []
        self.gateways = []
        self.positions = {"devices": {}, "gateways": {}}
        self.energy_log = []
        self.current_time = 0.0

        # Subsistemas
        self.lorawan_mac = LoRaWANMAC(network=self)
        self.packet_tracker = PacketTracker()
        self.throughput_calculator = NetworkThroughput(self)
        self.scheduler = EventScheduler()
        self.channel = ChannelModel(self)

        # Network Server
        self.ns = NetworkServer(region=self.region, adr_method=adr_method)

        # Inicializa dispositivos e gateways
        self.initialize_devices()
        self.initialize_gateways()

        network_logger.info(f"Rede inicializada: {num_devices} devices, {num_gateways} GWs, "
                            f"regiao={region_name}, deploy={deployment_type}, "
                            f"pathloss={model_pathloss}")

    def calculate_noise_floor(self, bw):
        """Calcula noise floor em dBm baseado na largura de banda."""
        noise_floor = 10 * np.log10(k * temperatura * bw) + 30 + noise_figure
        return noise_floor

    def initialize_devices(self):
        """Inicializa os dispositivos usando a estrategia de deployment selecionada."""
        # Gera posicoes
        if self.deployment_type == "grid":
            positions = deploy_grid(self.num_devices, self.area_size)
        elif self.deployment_type == "circular":
            center = self.area_size / 2
            positions = deploy_circular(self.num_devices, self.deployment_radius, center, center)
        elif self.deployment_type == "annular":
            center = self.area_size / 2
            positions = deploy_annular(self.num_devices, self.deployment_radius * 0.2,
                                       self.deployment_radius, center, center)
        elif self.deployment_type == "hexagonal":
            positions = deploy_hexagonal(self.num_devices, self.area_size / max(1, int(np.sqrt(self.num_devices))))
        elif self.deployment_type == "random_uniform":
            positions = deploy_random_uniform(self.num_devices, self.area_size)
        elif self.deployment_type == "clustered":
            n_clusters = max(1, self.num_devices // 10)
            positions = deploy_clustered(self.num_devices, n_clusters, self.area_size)
        elif self.deployment_type == "from_file":
            if self.deployment_file is None:
                raise ValueError("deployment_type='from_file' requer deployment_file=<caminho>")
            positions = deploy_from_file(self.deployment_file)
            if len(positions) < self.num_devices:
                raise ValueError(
                    f"Arquivo '{self.deployment_file}' tem {len(positions)} posicoes, "
                    f"mas num_devices={self.num_devices}"
                )
            positions = positions[:self.num_devices]
        else:
            positions = deploy_grid(self.num_devices, self.area_size)

        # Sprint 6: seleciona devices que usarao LR-FHSS
        n_lrfhss = int(self.num_devices * self.lrfhss_ratio)
        lrfhss_ids = set(random.sample(range(self.num_devices), n_lrfhss)) if n_lrfhss > 0 else set()

        for i in range(self.num_devices):
            x, y = positions[i]
            device = EndDevice(
                device_id=i,
                x=x, y=y,
                lambda_rate=self.lambda_rate,
                speed=self.speed,
                sf_range=self.sf_range,
                tx_power=self.tx_power,
                bw=self.bw,
                cr=self.cr,
                pl=self.pl,
                frequency_mhz=self.frequency_mhz,
                network=self
            )
            device.adr_enabled = self.adr_enabled
            device.is_indoor = (random.random() < self.indoor_ratio)
            device.use_lrfhss = (i in lrfhss_ids)

            # Sprint 10: OTAA credentials
            device.app_key = generate_app_key()
            device.dev_eui = generate_dev_eui(i)
            device.app_eui = generate_app_eui()

            # Battery model
            if self.battery_capacity_mah is not None:
                device.battery = BatteryModel(
                    capacity_mah=self.battery_capacity_mah,
                    voltage=voltage
                )
                device.energy_model.set_battery(device.battery)

                # Energy harvesting
                if self.energy_harvesting_config:
                    harvester = EnergyHarvester(
                        model=self.energy_harvesting_config.get("model", "solar"),
                        peak_power_mw=self.energy_harvesting_config.get("peak_power_mw", 100)
                    )
                    device.battery.harvester = harvester

            # Building penetration
            if device.is_indoor:
                self._building_penetration_cache[i] = np.random.lognormal(
                    mean=np.log(20), sigma=0.5)

            self.devices.append(device)
            self.positions["devices"][i] = (x, y)

    def initialize_gateways(self):
        """Inicializa os gateways."""
        rows = int(np.ceil(np.sqrt(self.num_gateways)))
        cols = int(np.ceil(self.num_gateways / rows))
        x_grid = np.linspace(0, self.area_size, cols, endpoint=False) + (self.area_size / (2 * cols))
        y_grid = np.linspace(0, self.area_size, rows, endpoint=False) + (self.area_size / (2 * rows))

        for i in range(self.num_gateways):
            r = i // cols
            c = i % cols
            x, y = x_grid[c], y_grid[r]
            gateway = Gateway(
                gw_id=i,
                x=x, y=y,
                network=self
            )
            self.gateways.append(gateway)
            self.positions["gateways"][i] = (x, y)
            self.ns.register_gateway(gateway)

    def toggle_adr(self):
        """Ativa/desativa ADR para toda a rede."""
        self.adr_enabled = adr_enabled
        for device in self.devices:
            device.adr_enabled = self.adr_enabled
        network_logger.info(f"ADR {'ATIVADO' if self.adr_enabled else 'DESATIVADO'} para toda a rede.")

    def set_mobility(self, mobility_enabled):
        """Ativa ou desativa a mobilidade dos dispositivos."""
        for device in self.devices:
            device.mobility_enabled = mobility_enabled
        status = "ativada" if mobility_enabled else "desativada"
        network_logger.info(f"Mobilidade {status}.")

    def get_propagation_delay(self, distance):
        """Retorna o atraso de propagacao baseado na distancia (s).
        Nota: Para distancias LoRa tipicas (5km), delay ~16.7us << ToA.
        Mantido para rigor, mas nao afeta resultados significativamente."""
        return distance / 3e8

    def export_device_log(self, device_data, filename="device_log.csv"):
        """Exporta dados detalhados dos dispositivos para CSV."""
        try:
            with open(filename, mode="w", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(["Tempo", "Device ID", "SF", "RSSI", "SNR",
                                 "Posicao (X, Y)", "Distancia do Gateway"])
                for data in device_data:
                    writer.writerow([data["time"], data["device_id"], data["sf"],
                                     data["rssi"], data["snr"], data["position"],
                                     data["distance"]])
            network_logger.info(f"Log exportado para {filename}")
        except Exception as e:
            network_logger.error(f"Erro ao exportar log: {e}")

    def assign_sf_by_distance(self, margin_db=10):
        """Atribui SF minimo que satisfaz link budget por device."""
        for device in self.devices:
            _, distance, _ = self.find_best_gateway(device)
            if distance is None:
                device.sf = 12
                continue
            path_loss = self.pathloss(distance, device.freq, self.model_pathloss,
                                      device_x=device.x, device_y=device.y)
            path_loss += self.get_building_penetration(device)
            rssi = device.tx_power + ed_antenna_gain + gw_antenna_gain - path_loss

            for sf in range(7, 13):
                sensitivity = gw_sensitivity_table.get((sf, device.bw),
                                                       device.set_sensibilidade(sf, device.bw))
                if rssi > sensitivity + margin_db:
                    device.sf = sf
                    break
            else:
                device.sf = 12
            device.airtime = device.calculate_airtime()

    def simulate_transmissions(self):
        """Simula a transmissao de pacotes usando simulacao de eventos discretos."""
        network_logger.info("Iniciando Simulacao de Transmissoes (Event-Driven)...")
        self.energy_log = []

        # Sprint 10: OTAA — JoinRequest/Accept com derivacao de session keys
        for device in self.devices:
            join_req = device.prepare_join_request()
            join_result = self.ns.on_join_request(
                device.device_id,
                join_req['dev_eui'],
                join_req['app_eui'],
                join_req['dev_nonce'],
                device.app_key,
            )
            if join_result:
                device.process_join_accept(
                    join_result['app_nonce'],
                    join_result['net_id'],
                    join_result['dev_addr'],
                )
            else:
                # Fallback: join simples (ABP)
                self.ns.on_join(device.device_id)

        # Schedule initial DEVICE_SEND for each device (Poisson-distributed)
        for device in self.devices:
            initial_delay = random.uniform(0, 1.0 / max(device.lambda_rate, 0.001))
            self.scheduler.schedule(initial_delay, EventType.DEVICE_SEND,
                                    self._on_device_send, device)

        # Schedule periodic mobility updates
        if self.mobility_enabled:
            self.scheduler.schedule(1.0, EventType.MOBILITY_UPDATE,
                                    self._on_mobility_update)

        # Schedule beacons for Class B (every 128s)
        self.scheduler.schedule(128.0, EventType.BEACON, self._on_beacon)

        # Schedule energy harvesting updates (every 60s)
        if self.energy_harvesting_config:
            self.scheduler.schedule(60.0, EventType.ENERGY_HARVEST,
                                    self._on_energy_harvest)

        # Run the discrete event simulation
        self.scheduler.run(until=self.simulation_time)
        self.current_time = self.simulation_time

        # Final energy sample
        total_energy = sum(d.energy_model.energy_consumed for d in self.devices) / 1000
        self.energy_log.append(total_energy)

        # Sprint 6: ACRDA — SIC iterativo pos-simulacao para LR-FHSS
        if self.lrfhss_ratio > 0:
            decoded_sic, sic_iters = self.acrda.process_window(self.lrfhss_channel)
            network_logger.info(f"ACRDA SIC: {decoded_sic} pkts decodificados em {sic_iters} iteracoes")

        # Statistics
        total_transmissions = self.packet_tracker.unique_packet_count
        total_retransmissions = self.packet_tracker.total_retransmissions
        self.print_statistics(total_transmissions, total_retransmissions)
        network_logger.info("Simulacao de Transmissoes Concluida.")
        network_logger.info(f"Scheduler: {self.scheduler.stats()}")
        network_logger.info(f"Channel: {self.channel.stats()}")
        network_logger.info(f"NS: {self.ns.stats()}")
        if self.lrfhss_ratio > 0:
            network_logger.info(f"LR-FHSS Channel: {self.lrfhss_channel.stats()}")

    # ==================== Event Handlers ====================

    def _on_device_send(self, device):
        """Handler: device tenta transmitir um pacote."""
        time = self.scheduler.now

        # Check battery depletion
        if device.battery is not None and device.battery.depleted:
            return  # Device morto

        # Duty cycle check
        if time < device.dc_release_time:
            delay = device.dc_release_time - time + 0.001
            self.scheduler.schedule(delay, EventType.DEVICE_SEND,
                                    self._on_device_send, device)
            return

        # Select channel for this transmission
        device.select_channel()

        # LinkCheck: 10% das transmissoes incluem LinkCheckReq para verificar cobertura
        if random.random() < 0.10:
            self.ns.link_check.request_link_check(device.device_id)

        # Validate payload size for region
        effective_pl = device.validate_payload_size(self.region)
        if effective_pl < device.pl:
            phy_logger.debug(f"Device {device.device_id}: payload truncado {device.pl}->{effective_pl}B (DR limit)")

        # Check dwell time
        if not device.check_dwell_time(self.region):
            phy_logger.debug(f"Device {device.device_id}: airtime excede dwell time, pulando")
            next_delay = np.random.exponential(1.0 / max(device.lambda_rate, 0.001))
            self.scheduler.schedule(next_delay, EventType.DEVICE_SEND,
                                    self._on_device_send, device)
            return

        # Check coverage
        best_gateway, _, best_rssi = self.find_best_gateway(device)
        if best_gateway is None or not device.coverage_status:
            next_delay = np.random.exponential(1.0 / max(device.lambda_rate, 0.001))
            self.scheduler.schedule(next_delay, EventType.DEVICE_SEND,
                                    self._on_device_send, device)
            return

        device.current_gateway = best_gateway
        device.rssi = best_rssi

        # G4: FSM — device entra em TX
        device.radio_state = RadioState.TX

        # Sprint 6: calcular airtime conforme PHY do device
        if device.use_lrfhss:
            airtime = self.lrfhss_phy.calculate_toa(device.pl)
        else:
            airtime = device.calculate_airtime()

        # Calculate airtime and update energy (Sprint 5: sim_time para sleep tracking)
        device.energy_model.update_energy(device=device, airtime=airtime, sim_time=time)

        # G7: duty cycle por sub-banda regional (Sprint 5)
        dc_limit = self.region.get_duty_cycle_limit(device.freq) or (ed_dc_limit_percent / 100.0)
        device.dc_release_time = time + airtime / dc_limit

        # Create packet
        device.frame_counter_up += 1

        # Sprint 10: computa MIC do frame (OTAA — NwkSKey derivado no join)
        _frame_mic = None
        if device.nwk_skey is not None and isinstance(device.dev_addr, int):
            from security import compute_frame_mic
            _payload_sim = bytes(device.pl)  # payload simulado (zeros)
            _frame_mic = compute_frame_mic(
                device.nwk_skey,
                device.dev_addr,
                device.frame_counter_up,
                0,  # uplink
                _payload_sim,
            )

        packet = Packet(device.device_id, device.sf, device.tx_power, device.bw,
                        device.freq, device.rssi, time, airtime)
        packet.frame_counter = device.frame_counter_up
        packet.mic = _frame_mic
        packet.confirmed = (random.random() < device.confirmed_ratio)
        device.last_transmissions.append(packet)

        if device.use_lrfhss:
            # Sprint 6: LR-FHSS path — fragmentacao + hopping
            packet.phy_type = "LR-FHSS"
            fragments = self.lrfhss_phy.create_fragments(str(packet.packet_id), device.pl, time)
            packet.lrfhss_fragments = fragments
            self.lrfhss_channel.add_fragments(fragments)
            self.acrda.register_packet(str(packet.packet_id), fragments)
            # LR-FHSS nao usa gateway de recepcao tradicional (sem paths SX1301)
            # mas ainda registra no gateway para fins de estatistica e duty cycle DL
            best_gateway.process_uplink(packet)
        else:
            # LoRa CSS path normal
            best_gateway.process_uplink(packet)

        # Sprint 9: MRC — combina SNR de todos os GWs que recebem o pacote
        if not device.use_lrfhss and len(self.gateways) > 1:
            mrc_receptions = self.ns.gateway_manager.process_uplink_from_all(
                packet, self.gateways, self)
            if len(mrc_receptions) > 1:
                snr_linear_sum = sum(10 ** (r['snr'] / 10.0) for r in mrc_receptions)
                packet.snr_mrc = 10.0 * np.log10(snr_linear_sum)
                packet.mrc_gw_count = len(mrc_receptions)
                # Registra todos os GWs que receberam no device registry
                dev_status = self.ns.device_registry.get_status(device.device_id)
                if dev_status is not None:
                    for r in mrc_receptions:
                        gw_id = r['gateway'].gw_id
                        if gw_id != best_gateway.gw_id:  # ja registrado pelo process_uplink
                            dev_status.gateways[gw_id] = {
                                'rssi': r['rssi'],
                                'snr': r['snr'],
                                'sinr': None,
                                'time': time,
                            }

        # Register in channel model for interference tracking (CSS only)
        if not device.use_lrfhss:
            self.channel.add_transmission(packet, time, time + airtime)

        # Track packet
        self.packet_tracker.add_packet(packet)

        # Network Server processing — aplica MAC commands retornados (G6)
        mac_commands = self.ns.on_uplink_received(packet, best_gateway)
        if mac_commands:
            self._apply_mac_commands(device, mac_commands)

        # G5: ADR exclusivamente server-side (NS gera LinkAdrReq via ADRComponent).
        # Device-side ADR so corre se server_side_adr=False (modo legado).
        if self.adr_enabled and not self.server_side_adr:
            self.lorawan_mac.process_mac_commands(device)

        # ADR backoff counter
        device.adr_ack_cnt += 1

        # Schedule TX_END event
        self.scheduler.schedule(airtime, EventType.PACKET_TX_END,
                                self._on_tx_end, device, packet)

        # Schedule next regular transmission (Poisson inter-arrival)
        next_delay = np.random.exponential(1.0 / max(device.lambda_rate, 0.001))
        self.scheduler.schedule(next_delay, EventType.DEVICE_SEND,
                                self._on_device_send, device)

    def _on_tx_end(self, device, packet):
        """Handler: transmissao terminou - avalia recepcao via canal."""
        time = self.scheduler.now

        # G4: FSM — TX terminou, aguardando janela RX1
        device.radio_state = RadioState.WAIT_RX1

        if device.use_lrfhss and packet.lrfhss_fragments:
            # Sprint 6: avaliacao LR-FHSS — colisao por fragmento + decodificacao parcial
            self.lrfhss_channel.check_fragment_collisions(packet.lrfhss_fragments)
            _, n_payloads, threshold = self.lrfhss_phy.fragment_packet(device.pl)
            received = self.lrfhss_channel.evaluate_packet(packet.lrfhss_fragments, threshold)
            packet.received = received
            if not received:
                packet.collided = True
            self.lrfhss_channel.cleanup_expired(time - 10.0)
            phy_logger.debug(f"LR-FHSS Device {device.device_id}: "
                             f"frags={len(packet.lrfhss_fragments)}, threshold={threshold}, "
                             f"received={received}")
        else:
            # Evaluate reception using channel model (SINR + interference matrix)
            if device.current_gateway:
                received = self.channel.evaluate_reception(packet, device.current_gateway)
                packet.received = received
                if not received:
                    packet.collided = True

        # Release gateway reception paths
        if device.current_gateway:
            device.current_gateway.release_paths(time)

        # Clean up expired transmissions
        if not device.use_lrfhss:
            self.channel.cleanup_expired(time - 10.0)

        # Schedule RX1 window
        self.scheduler.schedule(receive_delay1, EventType.RX1_WINDOW_OPEN,
                                self._on_rx1_open, device, packet)

    def _on_rx1_open(self, device, packet):
        """Handler: janela RX1 aberta — tenta entregar DL via NS scheduler."""
        device.radio_state = RadioState.RX1
        time = self.scheduler.now

        # Sprint 4: entrega DL (ACK ou MAC command) via NS scheduler + GW duty cycle (G5/G8)
        if self.ns.dl_scheduler.has_pending(device.device_id):
            best_gw = self.ns.gateway_manager.select_best_for_downlink(
                device.device_id, self.ns.device_registry)
            if best_gw is not None:
                dl_airtime = self._calc_dl_airtime(packet.sf, packet.bw)
                if self.ns.gateway_manager.can_send_downlink(
                        best_gw.gw_id, time, dl_airtime, "rx1"):
                    dl_packet = self.ns.get_downlink(device.device_id)
                    if dl_packet:
                        # DL-DL interference: verifica se outro GW ja transmite DL na mesma freq
                        if self._network_dl_busy.get(packet.freq, 0) > time:
                            # Colisao DL-DL: devolve pacote e vai para RX2
                            self.ns.dl_scheduler.schedule(device.device_id, dl_packet)
                            mac_logger.debug(f"DL-DL interference on {packet.freq} MHz: blocked for device {device.device_id}")
                        else:
                            self.ns.gateway_manager.record_downlink(
                                best_gw.gw_id, time, dl_airtime, "rx1")
                            # G8: bloqueia UL na mesma freq enquanto GW transmite DL
                            best_gw.dl_busy_until[packet.freq] = time + dl_airtime
                            # DL-DL: marca freq como busy network-wide
                            self._network_dl_busy[packet.freq] = time + dl_airtime
                            if dl_packet.packet_type == "ack" and not packet.collided:
                                packet.ack_received = True
                            if dl_packet.payload and hasattr(dl_packet.payload, 'cid'):
                                self._apply_mac_commands(device, [dl_packet.payload])
                            device.retransmission_attempts = 0
                            device.adr_ack_cnt = 0
                            device.radio_state = RadioState.IDLE
                            return

        # Sem DL entregue em RX1 — agenda RX2
        delay_to_rx2 = max(0, receive_delay2 - receive_delay1 - trx1)
        device.radio_state = RadioState.WAIT_RX2
        self.scheduler.schedule(trx1 + delay_to_rx2, EventType.RX2_WINDOW_OPEN,
                                self._on_rx2_open, device, packet)

    def _on_rx2_open(self, device, packet):
        """Handler: janela RX2 aberta — fallback DL em 869.525 MHz @ SF12."""
        device.radio_state = RadioState.RX2
        time = self.scheduler.now

        # G15: RX2 com parametros regionais ou configurados por RxParamSetupReq (MAC 0x05)
        rx2_freq = getattr(device, '_rx2_freq', None) or self.region.rx2_frequency
        if getattr(device, '_rx2_sf', None) is not None:
            rx2_sf = device._rx2_sf
            rx2_bw = getattr(device, '_rx2_bw', None) or self.region.dr_to_sf_bw(self.region.rx2_dr)[1]
        else:
            rx2_sf, rx2_bw = self.region.dr_to_sf_bw(self.region.rx2_dr)

        # Tenta entregar DL pendente via RX2 (G5/G8)
        if self.ns.dl_scheduler.has_pending(device.device_id):
            best_gw = self.ns.gateway_manager.select_best_for_downlink(
                device.device_id, self.ns.device_registry)
            if best_gw is not None:
                dl_airtime = self._calc_dl_airtime(rx2_sf, rx2_bw)
                if self.ns.gateway_manager.can_send_downlink(
                        best_gw.gw_id, time, dl_airtime, "rx2"):
                    dl_packet = self.ns.get_downlink(device.device_id)
                    if dl_packet:
                        # DL-DL interference: verifica se outro GW ja transmite DL na mesma freq
                        if self._network_dl_busy.get(rx2_freq, 0) > time:
                            # Colisao DL-DL: devolve pacote
                            self.ns.dl_scheduler.schedule(device.device_id, dl_packet)
                            mac_logger.debug(f"DL-DL interference on {rx2_freq} MHz (RX2): blocked for device {device.device_id}")
                        else:
                            self.ns.gateway_manager.record_downlink(
                                best_gw.gw_id, time, dl_airtime, "rx2")
                            # G8: bloqueia freq RX2 durante DL
                            best_gw.dl_busy_until[rx2_freq] = time + dl_airtime
                            # DL-DL: marca freq como busy network-wide
                            self._network_dl_busy[rx2_freq] = time + dl_airtime
                            if dl_packet.packet_type == "ack" and not packet.collided:
                                packet.ack_received = True
                            if dl_packet.payload and hasattr(dl_packet.payload, 'cid'):
                                self._apply_mac_commands(device, [dl_packet.payload])
                            device.retransmission_attempts = 0
                            device.adr_ack_cnt = 0
                            device.radio_state = RadioState.SLEEP
                            device.radio_state = RadioState.IDLE
                            return

        # G4: FSM — ciclo completo sem DL entregue
        device.radio_state = RadioState.SLEEP
        device.radio_state = RadioState.IDLE

        # Retransmite se o pacote colidiu
        if packet.collided and device.retransmission_attempts < device.max_retransmissions:
            backoff = random.uniform(1, 10) * (2 ** device.retransmission_attempts)
            device.retransmission_attempts += 1
            self.scheduler.schedule(backoff, EventType.DEVICE_SEND,
                                    self._on_retransmit, device)

    def _on_retransmit(self, device):
        """Handler: retransmissao de pacote."""
        time = self.scheduler.now

        # Check battery
        if device.battery is not None and device.battery.depleted:
            return

        # Duty cycle check
        if time < device.dc_release_time:
            delay = device.dc_release_time - time + 0.001
            self.scheduler.schedule(delay, EventType.DEVICE_SEND,
                                    self._on_retransmit, device)
            return

        # Select new channel
        device.select_channel()

        best_gateway, _, best_rssi = self.find_best_gateway(device)
        if best_gateway is None:
            return

        device.current_gateway = best_gateway
        device.rssi = best_rssi

        airtime = device.calculate_airtime()
        device.energy_model.update_energy(device=device, airtime=airtime, sim_time=time)
        # G7: duty cycle por sub-banda regional
        dc_limit = self.region.get_duty_cycle_limit(device.freq) or (ed_dc_limit_percent / 100.0)
        device.dc_release_time = time + airtime / dc_limit

        device.frame_counter_up += 1
        packet = Packet(device.device_id, device.sf, device.tx_power, device.bw,
                        device.freq, device.rssi, time, airtime)
        packet.frame_counter = device.frame_counter_up
        packet.confirmed = True
        packet.is_retransmission = True
        device.last_transmissions.append(packet)

        best_gateway.process_uplink(packet)
        self.channel.add_transmission(packet, time, time + airtime)
        self.packet_tracker.add_packet(packet, is_retransmission=True)
        self.ns.on_uplink_received(packet, best_gateway)

        self.scheduler.schedule(airtime, EventType.PACKET_TX_END,
                                self._on_tx_end, device, packet)

    def _on_mobility_update(self):
        """Handler: atualiza posicoes de todos os dispositivos."""
        for device in self.devices:
            if device.mobility_enabled:
                _, new_position = device.move(self.area_size, device.mobility_enabled, device.model)
                self.positions["devices"][device.device_id] = new_position
                device.update_coverage_status()

        # Sample energy consumption
        total_energy = sum(d.energy_model.energy_consumed for d in self.devices) / 1000
        self.energy_log.append(total_energy)

        self.scheduler.schedule(1.0, EventType.MOBILITY_UPDATE,
                                self._on_mobility_update)

    def _on_beacon(self):
        """Handler: transmite beacon para devices Class B."""
        time = self.scheduler.now
        for device in self.devices:
            if device.lorawan_class == 'B':
                device.process_beacon(time)
        # Schedule next beacon
        self.scheduler.schedule(128.0, EventType.BEACON, self._on_beacon)

    def _on_energy_harvest(self):
        """Handler: atualiza energy harvesting para devices com harvester."""
        time = self.scheduler.now
        for device in self.devices:
            if device.battery is not None and device.battery.harvester is not None:
                device.battery.harvester.harvest_energy(device.battery, time, 60.0)
        self.scheduler.schedule(60.0, EventType.ENERGY_HARVEST,
                                self._on_energy_harvest)

    def _calc_dl_airtime(self, sf, bw):
        """Calcula airtime de um pacote downlink tipico (13B payload, CR=4/5)."""
        import math
        pl = 13  # MHDR + MACPayload minimo (ACK + MAC commands)
        Tsym = (2.0 ** sf) / bw
        Tpream = (8 + 4.25) * Tsym
        DE = 1 if Tsym > 0.016 else 0
        H, CRC, cr = 0, 1, 1
        numerator = 8.0 * pl - 4.0 * sf + 28 + 16 * CRC - 20 * H
        denominator = 4.0 * (sf - 2 * DE)
        payloadSymbNB = 8 + max(math.ceil(numerator / denominator) * (cr + 4), 0)
        return Tpream + payloadSymbNB * Tsym

    def _apply_mac_commands(self, device, mac_commands):
        """Aplica MAC commands do Network Server ao device (G6 Sprint 3).

        Entrega LinkAdrReq, DutyCycleReq, DevStatusReq e outros na janela de downlink.
        """
        from mac_commands import MACCommandProcessor
        processor = MACCommandProcessor(network_server=self.ns)
        responses = processor.process_downlink_commands(device, mac_commands)
        if responses:
            mac_logger.debug(f"Device {device.device_id}: {len(mac_commands)} MAC cmds aplicados, "
                             f"{len(responses)} respostas geradas")
        # Processa respostas geradas pelo device (uplink MAC commands)
        if responses:
            device.pending_mac_commands.extend(responses)
            mac_logger.debug(f"Device {device.device_id}: {len(responses)} respostas MAC enfileiradas para proximo UL")
            # DevStatusAns: registra imediatamente no NS
            from mac_commands import DevStatusAns
            for resp in responses:
                if isinstance(resp, DevStatusAns):
                    self.ns.dev_status.record_status(
                        device.device_id,
                        resp.payload.get("battery", 255),
                        resp.payload.get("margin", 0),
                        self.scheduler.now,
                    )

    def detect_collisions_and_interference(self):
        """Detecta colisoes usando sobreposicao temporal + SINR com matriz de interferencia."""
        packets = sorted(self.packet_tracker.packets, key=lambda p: p.arrival_time)

        for i, p1 in enumerate(packets):
            for j in range(i + 1, len(packets)):
                p2 = packets[j]

                if p2.arrival_time >= p1.arrival_time + p1.rectime:
                    break

                if p1.device_id == p2.device_id:
                    continue
                if p1.freq != p2.freq:
                    continue

                overlap_start = max(p1.arrival_time, p2.arrival_time)
                overlap_end = min(p1.arrival_time + p1.rectime, p2.arrival_time + p2.rectime)
                if overlap_end <= overlap_start:
                    continue

                sf1_idx = 12 - p1.sf
                sf2_idx = 12 - p2.sf

                if p1.rssi is not None and p2.rssi is not None:
                    threshold_1 = interference_matrix[sf1_idx][sf2_idx]
                    rssi_diff_1 = p1.rssi - p2.rssi
                    if rssi_diff_1 < threshold_1:
                        p1.collided = True

                    threshold_2 = interference_matrix[sf2_idx][sf1_idx]
                    rssi_diff_2 = p2.rssi - p1.rssi
                    if rssi_diff_2 < threshold_2:
                        p2.collided = True

    def find_best_gateway(self, device=None, avoid_congestion=False):
        """Encontra o melhor gateway disponivel para um dispositivo."""
        available_gateways = []

        for gateway in self.gateways:
            if device:
                distance = max(np.sqrt((device.x - gateway.x) ** 2 + (device.y - gateway.y) ** 2 +
                                      (self.ht_m - self.hr_m) ** 2), 1.0)

                path_loss = self.pathloss(distance, device.freq, model_pathloss=self.model_pathloss,
                                          device_x=device.x, device_y=device.y)
                # G3: adiciona perda de penetracao predial para dispositivos indoor
                path_loss += self.get_building_penetration(device)
                rssi = device.tx_power + ed_antenna_gain + gw_antenna_gain - path_loss

                # G10: usa sensibilidade do GW (SX1301), mais sensivel que o ED (SX1272)
                gw_sensitivity = gw_sensitivity_table.get(
                    (device.sf, device.bw), device.set_sensibilidade(device.sf, device.bw))

                if rssi > gw_sensitivity:
                    available_gateways.append((gateway, distance, rssi))
            else:
                available_gateways.append((gateway, None, None))

        if not available_gateways:
            if device:
                device.coverage_status = False
            return None, None, None

        if avoid_congestion:
            other_gateways = [g for g in available_gateways
                              if device is None or device.current_gateway is None
                              or g[0].gw_id != device.current_gateway.gw_id]
            if not other_gateways:
                return None, None, None
            return min(other_gateways, key=lambda x: len(x[0].received_packets))

        if device:
            best = max(available_gateways, key=lambda x: x[2])
            device.coverage_status = True
            return best

        return None, None, None

    def get_building_penetration(self, device):
        """Retorna perda de penetracao predial para dispositivo indoor (G3 Sprint 2)."""
        if device.is_indoor:
            return self._building_penetration_cache.get(device.device_id, 20.0)
        return 0.0

    def pathloss(self, distance, frequency_mhz, model_pathloss, fading_type=None,
                 device_x=None, device_y=None):
        """Calcula a perda de percurso usando diferentes modelos de propagacao.

        Modelos suportados:
        1. okumura_hata
        2. log_distance
        3. fspl (Free-Space Path Loss)
        4. cost_hata
        5. log_normal_shadowing (padrao FLoRa/ns-3)
        6. correlated_shadowing (como ns-3 CorrelatedShadowingPropagationLossModel)
        7. building_penetration (como ns-3, adiciona perda indoor)
        8. fading (Path Loss + Fading: Rayleigh/Rician/Nakagami)
        9. oulu (FLoRa/LoRaSim, d0=40m, gamma=2.08)

        device_x, device_y: posicao do device para correlated_shadowing (G2 Sprint 2)
        """
        if distance <= 0:
            return float('inf')

        # Modelos classicos esperam km
        distance_km = distance / 1000.0

        if model_pathloss == "okumura_hata":
            correction_factor = (1.1 * np.log10(frequency_mhz) - 0.7) * self.hr_m - (1.56 * np.log10(frequency_mhz) - 0.8)
            path_loss = (
                69.55 + 26.16 * np.log10(frequency_mhz)
                - 13.82 * np.log10(self.ht_m)
                - correction_factor
                + (44.9 - 6.55 * np.log10(self.ht_m)) * np.log10(distance_km)
            )

        elif model_pathloss == "log_distance":
            path_loss_exponent = 3.5
            path_loss = 20 * np.log10(frequency_mhz) + 10 * path_loss_exponent * np.log10(distance_km) - 28

        elif model_pathloss == "fspl":
            path_loss = 20 * np.log10(distance_km) + 20 * np.log10(frequency_mhz) + 32.45

        elif model_pathloss == "cost_hata":
            correction_factor = (1.1 * np.log10(frequency_mhz) - 0.7) * self.hr_m - (1.56 * np.log10(frequency_mhz) - 0.8)
            path_loss = (
                46.3 + 33.9 * np.log10(frequency_mhz)
                - 13.82 * np.log10(self.ht_m)
                - correction_factor
                + (44.9 - 6.55 * np.log10(self.ht_m)) * np.log10(distance_km)
            )

        elif model_pathloss == "log_normal_shadowing":
            d0 = 1.0
            PL_d0 = 7.7
            gamma = 3.76
            sigma = 3.57
            X_sigma = np.random.normal(0, sigma)
            path_loss = PL_d0 + 10 * gamma * np.log10(distance / d0) + X_sigma

        elif model_pathloss == "correlated_shadowing":
            d0 = 1.0
            PL_d0 = 7.7
            gamma = 3.76
            base_loss = PL_d0 + 10 * gamma * np.log10(distance / d0)
            if self.shadowing_map is not None and device_x is not None and device_y is not None:
                # G2: usa posicao real do device para shadowing espacialmente correlacionado
                shadowing = self.shadowing_map.get_shadowing(device_x, device_y)
                path_loss = base_loss + shadowing
            elif self.shadowing_map is not None:
                # Fallback: shadowing independente se coordenadas nao fornecidas
                sigma = 3.57
                path_loss = base_loss + np.random.normal(0, sigma)
            else:
                sigma = 3.57
                path_loss = base_loss + np.random.normal(0, sigma)

        elif model_pathloss == "building_penetration":
            # Modelo base + perda de penetracao indoor
            d0 = 1.0
            PL_d0 = 7.7
            gamma = 3.76
            sigma = 3.57
            X_sigma = np.random.normal(0, sigma)
            path_loss = PL_d0 + 10 * gamma * np.log10(distance / d0) + X_sigma
            # Perda de penetracao (ITU-R P.2109)
            bpl = np.random.lognormal(mean=np.log(20), sigma=0.5)
            path_loss += bpl

        elif model_pathloss == "oulu":
            # Modelo Oulu — LoRaSim/FLoRa (Bor et al. 2016)
            # PL(d) = 127.41 + 10*2.08*log10(d/40) + N(0, 3.57)
            # Compativel com FLoRa (d0=40m, gamma=2.08, PL_d0=127.41, sigma=3.57)
            d0 = 40.0
            PL_d0 = 127.41
            gamma = 2.08
            sigma = 3.57
            X_sigma = np.random.normal(0, sigma)
            path_loss = PL_d0 + 10 * gamma * np.log10(max(distance / d0, 1e-10)) + X_sigma

        elif model_pathloss == "fading":
            base_loss = self.pathloss(distance, frequency_mhz, "cost_hata")

            if fading_type == "rayleigh":
                fading_db = 20 * np.log10(max(np.random.rayleigh(scale=1), 1e-10))
            elif fading_type == "rician":
                K = 3
                sample = max(np.sqrt(np.random.noncentral_chi2(2, 2 * K) / (2 * (1 + K))), 1e-10)
                fading_db = 20 * np.log10(sample)
            elif fading_type == "nakagami":
                m = 3
                sample = max(np.sqrt(np.random.gamma(shape=m, scale=1.0 / m)), 1e-10)
                fading_db = 20 * np.log10(sample)
            else:
                fading_db = 0
            path_loss = base_loss - fading_db

        else:
            raise ValueError(f"Modelo de propagacao desconhecido: {model_pathloss}")

        return path_loss

    def print_statistics(self, total_transmissions=0, total_retransmissions=0):
        """Exibe estatisticas finais da simulacao."""
        stats = self.packet_tracker.get_stats()

        total_transmissions = stats.get("Total Pacotes", total_transmissions)
        total_retransmissions = stats.get("Retransmissoes", total_retransmissions)
        total_colisoes = stats.get("Colisoes", 0)
        pacotes_sucesso = stats.get("Entregues com Sucesso", 0)
        pdr = stats.get("Taxa de Entrega (PDR)", 0.0)

        retransmission_rate = (total_retransmissions / total_transmissions * 100) if total_transmissions > 0 else 0
        covered_devices = self.throughput_calculator.calculate_covered_devices()
        network_throughput = self.throughput_calculator.calculate_total_throughput()

        # Battery stats
        battery_stats = ""
        if self.battery_capacity_mah is not None:
            alive = sum(1 for d in self.devices if d.battery is None or not d.battery.depleted)
            avg_soc = np.mean([d.battery.soc_percent() for d in self.devices if d.battery is not None])
            battery_stats = f"\n{'Devices Vivos:':<30} {alive}/{self.num_devices}\n{'SoC Medio:':<30} {avg_soc:.1f}%"

        # Gateway saturation
        total_sat = sum(gw.saturation_events for gw in self.gateways)

        # Sprint 6: metricas separadas por PHY
        css_pkts = [p for p in self.packet_tracker.packets if p.phy_type == "CSS"]
        lrfhss_pkts = [p for p in self.packet_tracker.packets if p.phy_type == "LR-FHSS"]

        print("\n" + "=" * 50)
        print(f"{'Regiao:':<30} {self.region.name}")
        print(f"{'Dispositivos Cobertos:':<30} {covered_devices}")
        print(f"{'Total de Pacotes:':<30} {total_transmissions}")
        print(f"{'Retransmissoes:':<30} {total_retransmissions} ({retransmission_rate:.2f}%)")
        print(f"{'Colisoes:':<30} {total_colisoes}")
        print(f"{'Pacotes Entregues:':<30} {pacotes_sucesso}")
        print(f"{'PDR (Packet Delivery Ratio):':<30} {pdr:.2f}%")
        print(f"{'Vazao Total de Dados:':<30} {network_throughput:.2f} kbps")
        print(f"{'GW Saturation Events:':<30} {total_sat}")
        if battery_stats:
            print(battery_stats)

        if self.lrfhss_ratio > 0 and (css_pkts or lrfhss_pkts):
            print("-" * 50)
            if css_pkts:
                css_ok = sum(1 for p in css_pkts if not p.collided)
                css_pdr = 100.0 * css_ok / len(css_pkts)
                print(f"{'[CSS] Pacotes:':<30} {len(css_pkts)} | PDR: {css_pdr:.1f}%")
            if lrfhss_pkts:
                lrfhss_ok = sum(1 for p in lrfhss_pkts if not p.collided)
                lrfhss_pdr = 100.0 * lrfhss_ok / len(lrfhss_pkts)
                fhss_stats = self.lrfhss_channel.stats()
                print(f"{'[LR-FHSS] Pacotes:':<30} {len(lrfhss_pkts)} | PDR: {lrfhss_pdr:.1f}%")
                print(f"{'[LR-FHSS] Colisoes frag:':<30} {fhss_stats['total_collisions']}")

        # Sprint 9: MRC diversity stats
        css_packets = [p for p in self.packet_tracker.packets if p.phy_type == "CSS"]
        mrc_packets = [p for p in css_packets if getattr(p, 'mrc_gw_count', 1) > 1]
        if mrc_packets:
            avg_mrc_gws = np.mean([p.mrc_gw_count for p in mrc_packets])
            mrc_gain = np.mean([p.snr_mrc - p.snr for p in mrc_packets if p.snr is not None and p.snr_mrc is not None])
            print(f"{'MRC Diversity (pkts):':<30} {len(mrc_packets)}/{len(css_packets)} ({len(mrc_packets)/max(len(css_packets),1)*100:.1f}%)")
            print(f"{'MRC Avg GW Count:':<30} {avg_mrc_gws:.1f}")
            if not np.isnan(mrc_gain):
                print(f"{'MRC SNR Gain (avg dB):':<30} {mrc_gain:.2f}")

        print("=" * 50)

    def plot_statistics(self):
        """Gera graficos sobre desempenho da rede."""
        sf_counts = Counter(device.sf for device in self.devices)
        snr_values = [
            np.mean(device.snr) if isinstance(device.snr, (list, np.ndarray)) and device.snr is not None else device.snr
            for device in self.devices if isinstance(device.snr, (int, float))
        ]

        n_plots = 3
        has_battery = self.battery_capacity_mah is not None
        if has_battery:
            n_plots = 4

        fig, axes = plt.subplots(1, n_plots, figsize=(5 * n_plots, 5))

        axes[0].bar(sf_counts.keys(), sf_counts.values(), color='blue')
        axes[0].set_xlabel("Spreading Factor")
        axes[0].set_ylabel("Numero de Dispositivos")
        axes[0].set_title("Distribuicao de SFs")

        if self.energy_log:
            time_steps = range(len(self.energy_log))
            axes[1].plot(time_steps, self.energy_log, marker='o', linestyle='-', color='red', markersize=2)
            axes[1].set_xlabel("Tempo (s)")
            axes[1].set_ylabel("Energia Total Consumida (J)")
            axes[1].set_title("Energia Total Consumida")
        else:
            axes[1].set_title("Sem Dados de Energia")

        if snr_values:
            axes[2].hist(snr_values, bins=10, color='orange', edgecolor='black')
            axes[2].set_xlabel("SNR (dB)")
            axes[2].set_ylabel("Numero de Dispositivos")
            axes[2].set_title("Distribuicao de SNR")
        else:
            axes[2].set_title("Sem Dados de SNR")

        if has_battery:
            soc_values = [d.battery.soc_percent() for d in self.devices if d.battery is not None]
            if soc_values:
                axes[3].hist(soc_values, bins=20, color='green', edgecolor='black')
                axes[3].set_xlabel("SoC (%)")
                axes[3].set_ylabel("Numero de Dispositivos")
                axes[3].set_title("Distribuicao Battery SoC")

        plt.tight_layout()
        plt.savefig('statistics.png')
        plt.close()

    def plot_mobility(self):
        """Plota a ultima posicao dos dispositivos, diferenciando por SF e cobertura."""
        plt.figure(figsize=(8, 8))

        sf_colors = {7: 'blue', 8: 'green', 9: 'orange', 10: 'purple', 11: 'brown', 12: 'pink'}
        plotted_labels = set()
        fora_cobertura_legenda = False

        for device in self.devices:
            pos_list = self.positions["devices"].get(device.device_id, [])
            if isinstance(pos_list, tuple):
                pos_list = [pos_list]
            if not pos_list or not isinstance(pos_list[-1], (tuple, list)):
                continue

            last_position = pos_list[-1]
            x_val, y_val = last_position

            if not device.coverage_status:
                color = 'gray'
                label = "Fora da Cobertura"
                if not fora_cobertura_legenda:
                    plt.scatter(x_val, y_val, marker='o', color=color, label=label)
                    fora_cobertura_legenda = True
                else:
                    plt.scatter(x_val, y_val, marker='o', color=color)
            else:
                color = sf_colors.get(device.sf, 'black')
                label = f'SF {device.sf}'
                if label not in plotted_labels:
                    plt.scatter(x_val, y_val, marker='o', color=color, label=label)
                    plotted_labels.add(label)
                else:
                    plt.scatter(x_val, y_val, marker='o', color=color)

        for gw_id, (x, y) in self.positions["gateways"].items():
            label = f'Gateway {gw_id}'
            if label not in plotted_labels:
                plt.scatter(x, y, marker='s', color='red', label=label, s=100)
                plotted_labels.add(label)
            else:
                plt.scatter(x, y, marker='s', color='red')

        plt.xlim(0, self.area_size)
        plt.ylim(0, self.area_size)
        plt.xlabel("X Position (m)")
        plt.ylabel("Y Position (m)")
        plt.title(f"Posicao dos Devices ({self.deployment_type} deploy, {self.region.name})")
        plt.legend()
        plt.grid()
        plt.savefig('mobility.png')
        plt.close()

    def run_simulation(self):
        network_logger.info(f"Iniciando simulacao com {len(self.devices)} dispositivos...")


#==================================================================== Main ==================================================================

if __name__ == '__main__':
    network = Network(num_devices, num_gateways, area_size, lambda_rate, speed, sf_range,
                      tx_power, frequency_mhz, ht_m, hr_m,
                      bw, cr, pl, simulation_time, adr_enabled, model_pathloss)

    print(f"\nDispositivos ativos na simulacao: {len(network.devices)}\n")

    network.toggle_adr()
    network.set_mobility(mobility_enabled)
    network.simulate_transmissions()
    network.detect_collisions_and_interference()
    stats = network.packet_tracker.get_stats()
    covered_devices = network.throughput_calculator.calculate_covered_devices()
    network.plot_statistics()
    network.plot_mobility()

    # Export analytics
    from analytics import compute_metrics, export_json, export_csv_detailed, export_device_summary, export_npz, compare_with_analytical
    metrics = compute_metrics(network)
    export_json(metrics)
    export_csv_detailed(network)
    export_device_summary(network)
    export_npz(network)
    comparison = compare_with_analytical(network)
    print(f"\nValidacao Analitica: PDR_sim={comparison['simulation_pdr']:.3f}  "
          f"PDR_anal={comparison['analytical_pdr']:.3f}  "
          f"diff={comparison['difference']:.3f}")

    total_energy = sum(device.energy_model.energy_consumed for device in network.devices)
    print(f"Consumo total de energia da rede: {total_energy:.2f} mJ\n")

    # Packet tracker export
    network.packet_tracker.export_device_log(network)

    # Graficos expandidos (Sprint 7)
    from plots import plot_all
    plot_all(network)
