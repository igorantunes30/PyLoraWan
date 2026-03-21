"""Carregador de configuracao YAML/JSON.

Suporta:
- Carregamento de config YAML ou JSON
- Conversao para parametros do Network
- Batch mode com sweep de parametros
"""

import json
import os

try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False


def load_config(filename):
    """Carrega configuracao de arquivo YAML ou JSON."""
    ext = os.path.splitext(filename)[1].lower()

    with open(filename) as f:
        if ext in ('.yaml', '.yml'):
            if not HAS_YAML:
                raise ImportError("PyYAML nao instalado. Use: pip install pyyaml")
            config = yaml.safe_load(f)
        elif ext == '.json':
            config = json.load(f)
        else:
            raise ValueError(f"Formato desconhecido: {ext}. Use .yaml ou .json")

    return config


def config_to_params(config):
    """Converte config dict para parametros do Network."""
    sim = config.get("simulation", {})
    net = config.get("network", {})
    dev = config.get("devices", {})
    phy = config.get("physical", {})
    energy = config.get("energy", {})

    params = {
        "num_devices": net.get("num_devices", 50),
        "num_gateways": net.get("num_gateways", 1),
        "area_size": net.get("area_size_m", 10000),
        "lambda_rate": 1.0 / max(dev.get("traffic_interval_s", 300), 0.001),
        "speed": dev.get("mobility", {}).get("speed_mps", 0),
        "sf_range": [7, 8, 9, 10, 11, 12],
        "tx_power": dev.get("tx_power_dbm", 14),
        "frequency_mhz": [868.1, 868.3, 868.5],
        "ht_m": 1.5,
        "hr_m": 30,
        "bw": [125000],
        "cr": 1,
        "pl": dev.get("payload_bytes", 20),
        "simulation_time": sim.get("duration_s", 3600),
        "adr_enabled": dev.get("adr_enabled", True),
        "model_pathloss": phy.get("pathloss_model", "log_normal_shadowing"),
        "region_name": sim.get("region", "EU868"),
        "deployment_type": net.get("deployment", "grid"),
        "deployment_radius": net.get("deployment_radius_m"),
        "battery_capacity_mah": energy.get("battery_capacity_mah"),
    }

    # Energy harvesting
    harvesting = energy.get("harvesting", {})
    if harvesting.get("enabled", False):
        params["energy_harvesting"] = {
            "model": harvesting.get("model", "solar"),
            "peak_power_mw": harvesting.get("peak_power_mw", 100),
        }

    return params


def run_from_config(config_file):
    """Executa simulacao a partir de arquivo de configuracao."""
    from network import Network
    config = load_config(config_file)
    params = config_to_params(config)

    # Set seed
    sim = config.get("simulation", {})
    seed = sim.get("seed", 42)
    import random
    import numpy as np
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)

    network = Network(**params)
    network.simulate_transmissions()
    return network
