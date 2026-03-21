"""Fixtures compartilhadas para testes do PyLoRaWAN."""

import sys
import os
import pytest

# Adiciona diretorio raiz ao path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


@pytest.fixture
def default_network():
    """Cria rede padrao para testes: 50 EDs, 1 GW, EU868."""
    from network import Network
    from parametors import (num_devices, num_gateways, area_size, lambda_rate, speed,
                            sf_range, tx_power, frequency_mhz, ht_m, hr_m,
                            bw, cr, pl, simulation_time, adr_enabled, model_pathloss)
    return Network(
        num_devices=10, num_gateways=1, area_size=area_size,
        lambda_rate=lambda_rate, speed=speed, sf_range=sf_range,
        tx_power=tx_power, frequency_mhz=frequency_mhz,
        ht_m=ht_m, hr_m=hr_m, bw=bw, cr=cr, pl=pl,
        simulation_time=60, adr_enabled=False,
        model_pathloss="log_normal_shadowing"
    )


@pytest.fixture
def single_device_network():
    """Cria rede com 1 device e 1 GW para testes unitarios."""
    from network import Network
    from parametors import (area_size, lambda_rate, speed, sf_range, tx_power,
                            frequency_mhz, ht_m, hr_m, bw, cr, pl)
    return Network(
        num_devices=1, num_gateways=1, area_size=area_size,
        lambda_rate=0.01, speed=0, sf_range=[7],
        tx_power=tx_power, frequency_mhz=frequency_mhz,
        ht_m=ht_m, hr_m=hr_m, bw=bw, cr=cr, pl=pl,
        simulation_time=10, adr_enabled=False,
        model_pathloss="fspl"
    )


@pytest.fixture
def eu868_region():
    """Retorna regiao EU868."""
    from regions import EU868
    return EU868()


@pytest.fixture
def us915_region():
    """Retorna regiao US915."""
    from regions import US915
    return US915()
