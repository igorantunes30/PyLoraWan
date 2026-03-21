"""NewChannel Component - configura canais opcionais nos devices via NewChannelReq.

Envia NewChannelReq uma vez por device apos o join, adicionando
canais opcionais EU868 (867.1, 867.3, 867.5 MHz) alem dos 3 obrigatorios.
"""

# Canais opcionais EU868 (indices 3-7 conforme LoRaWAN Regional Parameters)
EU868_OPTIONAL_CHANNELS = [
    (3, 867.1),
    (4, 867.3),
    (5, 867.5),
]


class NewChannelComponent:
    """Envia NewChannelReq para adicionar canais opcionais apos join."""

    def __init__(self, channels=None):
        self.channels = channels or EU868_OPTIONAL_CHANNELS
        self.configured = set()  # device_ids ja configurados

    def on_packet(self, packet, device_status):
        """Envia NewChannelReq uma unica vez por device."""
        device_id = packet.device_id
        if device_id in self.configured:
            return []

        self.configured.add(device_id)

        from mac_commands import NewChannelReq
        commands = []
        for ch_index, freq in self.channels:
            commands.append(NewChannelReq(
                ch_index=ch_index,
                frequency=freq,
                min_dr=0,
                max_dr=5,
            ))
        return commands
