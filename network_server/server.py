"""Network Server principal.

Coordena Device Registry, Gateway Manager, Controller e Downlink Scheduler.
"""

import os, struct
from .device_registry import DeviceRegistry
from .gateway_manager import GatewayManager
from .scheduler import DownlinkScheduler, DownlinkPacket
from .controller import NetworkController
from .components.adr import ADRComponent
from .components.duty_cycle import DutyCycleComponent
from .components.link_check import LinkCheckComponent
from .components.dev_status import DevStatusComponent


class NetworkServer:
    """Network Server LoRaWAN.

    Equivalente ao ns-3 NetworkServer + NetworkStatus + NetworkController.
    """

    def __init__(self, region=None, adr_method="average"):
        self.device_registry = DeviceRegistry()
        self.gateway_manager = GatewayManager()
        self.dl_scheduler = DownlinkScheduler()
        self.controller = NetworkController()

        # Adiciona componentes default
        self.adr_component = ADRComponent(method=adr_method)
        self.dc_component = DutyCycleComponent(region=region)
        self.link_check = LinkCheckComponent()
        self.dev_status = DevStatusComponent()

        self.controller.add_component(self.adr_component)
        self.controller.add_component(self.dc_component)
        self.controller.add_component(self.link_check)
        self.controller.add_component(self.dev_status)

        from .components.new_channel import NewChannelComponent
        self.new_channel = NewChannelComponent()
        self.controller.add_component(self.new_channel)

        self.total_uplinks = 0
        self.total_downlinks = 0

    def register_gateway(self, gateway):
        """Registra gateway no NS."""
        self.gateway_manager.register(gateway)

    def on_join(self, device_id):
        """Processa join request."""
        status = self.device_registry.register(device_id)
        self.controller.on_join(device_id, status)
        return status

    def on_join_request(self, device_id, dev_eui, app_eui, dev_nonce, app_key):
        """Processa JoinRequest OTAA e retorna parametros do JoinAccept.

        Gera AppNonce aleatorio, deriva session keys, registra device.
        Conforme LoRaWAN 1.0.x sec 6.2.5.

        Returns:
            dict com app_nonce, net_id, dev_addr — ou None se falhar
        """
        from security import derive_session_keys

        app_nonce = int.from_bytes(os.urandom(3), 'little')
        net_id = 0x000001  # NetID fixo para simulacao
        dev_addr = int.from_bytes(os.urandom(4), 'little') & 0x01FFFFFF

        nwk_skey, app_skey = derive_session_keys(app_key, app_nonce, net_id, dev_nonce)

        # Registra no device registry com session keys
        status = self.device_registry.register(device_id, dev_addr=f"{dev_addr:08X}")
        status.nwk_skey = nwk_skey
        status.app_skey = app_skey
        status.dev_addr = dev_addr
        status.app_nonce = app_nonce
        status.joined = True

        self.controller.on_join(device_id, status)

        return {
            'app_nonce': app_nonce,
            'net_id': net_id,
            'dev_addr': dev_addr,
        }

    def on_uplink_received(self, packet, gateway):
        """Processa uplink recebido por um gateway.

        Retorna lista de MAC commands para downlink.
        """
        self.total_uplinks += 1

        # Sprint 10: Valida MIC se disponivel (OTAA session keys)
        if getattr(packet, 'mic', None) is not None:
            status_check = self.device_registry.get_status(packet.device_id)
            if status_check is not None and getattr(status_check, 'nwk_skey', None) is not None:
                from security import verify_frame_mic
                payload_sim = bytes(20)  # payload simulado (zeros, mesmo usado no device)
                mic_ok = verify_frame_mic(
                    status_check.nwk_skey,
                    status_check.dev_addr,
                    packet.frame_counter,
                    0,  # uplink
                    payload_sim,
                    packet.mic,
                )
                packet.mic_valid = mic_ok

        # Valida frame counter
        if not self.device_registry.validate_frame_counter(packet.device_id, packet.frame_counter):
            return []

        # Atualiza registro do device
        status = self.device_registry.update(packet, gateway)

        # Executa componentes do controller
        mac_commands = self.controller.on_new_packet(packet, status)

        # Agenda ACK se pacote confirmed e nao colidido
        if packet.confirmed and not packet.collided:
            ack = DownlinkPacket(packet.device_id, packet_type="ack")
            self.dl_scheduler.schedule(packet.device_id, ack)
            status.needs_ack = True

        # Agenda MAC commands
        if mac_commands:
            for cmd in mac_commands:
                mc_pkt = DownlinkPacket(packet.device_id, payload=cmd, packet_type="mac_command")
                self.dl_scheduler.schedule(packet.device_id, mc_pkt)

        return mac_commands

    def get_downlink(self, device_id):
        """Retorna proximo downlink para device."""
        _, packet = self.dl_scheduler.get_next(device_id)
        if packet:
            self.total_downlinks += 1
        return packet

    def stats(self):
        return {
            "total_uplinks": self.total_uplinks,
            "total_downlinks": self.total_downlinks,
            "registered_devices": len(self.device_registry.devices),
            "dl_scheduler": self.dl_scheduler.stats(),
        }
