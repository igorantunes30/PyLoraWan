"""Network Controller - controlador com componentes plugaveis.

Gerencia componentes de controle (ADR, Link Check, Duty Cycle)
e coordena acoes no Network Server.
"""


class NetworkController:
    """Controlador de rede com componentes plugaveis."""

    def __init__(self):
        self.components = []

    def add_component(self, component):
        """Adiciona componente de controle."""
        self.components.append(component)

    def on_new_packet(self, packet, device_status):
        """Processa novo pacote com todos os componentes.

        Cada componente pode gerar MAC commands em resposta.
        """
        mac_commands = []
        for component in self.components:
            cmds = component.on_packet(packet, device_status)
            if cmds:
                mac_commands.extend(cmds)
        return mac_commands

    def on_join(self, device_id, device_status):
        """Notifica componentes sobre novo join."""
        for component in self.components:
            if hasattr(component, 'on_join'):
                component.on_join(device_id, device_status)
