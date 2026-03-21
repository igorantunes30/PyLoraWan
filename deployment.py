"""Estrategias de Deployment para dispositivos e gateways.

Suporta: grid, circular, annular, hexagonal, random_uniform, clustered, from_file.
"""

import csv
import numpy as np


def deploy_grid(n_devices, area_size):
    """Deploy em grade regular (matriz).

    Posiciona devices uniformemente em grid.
    """
    rows = int(np.ceil(np.sqrt(n_devices)))
    cols = int(np.ceil(n_devices / rows))
    x_grid = np.linspace(0, area_size, cols, endpoint=False) + (area_size / (2 * cols))
    y_grid = np.linspace(0, area_size, rows, endpoint=False) + (area_size / (2 * rows))

    positions = []
    for i in range(n_devices):
        r = i // cols
        c = i % cols
        positions.append((x_grid[c], y_grid[r]))
    return positions


def deploy_circular(n_devices, radius, center_x=None, center_y=None):
    """Deploy uniforme em disco circular (como FLoRa).

    Usa sqrt(U) para distribuicao uniforme na area.
    """
    if center_x is None:
        center_x = radius
    if center_y is None:
        center_y = radius

    positions = []
    for _ in range(n_devices):
        r = radius * np.sqrt(np.random.uniform(0, 1))
        theta = np.random.uniform(0, 2 * np.pi)
        x = center_x + r * np.cos(theta)
        y = center_y + r * np.sin(theta)
        positions.append((x, y))
    return positions


def deploy_annular(n_devices, r_min, r_max, center_x=None, center_y=None):
    """Deploy em anel [r_min, r_max] (como LoRaWANSim)."""
    if center_x is None:
        center_x = r_max
    if center_y is None:
        center_y = r_max

    positions = []
    for _ in range(n_devices):
        r = np.sqrt(np.random.uniform(r_min**2, r_max**2))
        theta = np.random.uniform(0, 2 * np.pi)
        x = center_x + r * np.cos(theta)
        y = center_y + r * np.sin(theta)
        positions.append((x, y))
    return positions


def deploy_hexagonal(n_positions, inter_distance):
    """Deploy em grade hexagonal (como ns-3).

    Ideal para posicionamento de gateways.
    """
    positions = []
    row = 0
    while len(positions) < n_positions:
        x_offset = (inter_distance / 2) if row % 2 else 0
        col = 0
        while len(positions) < n_positions:
            x = col * inter_distance + x_offset
            y = row * inter_distance * np.sqrt(3) / 2
            positions.append((x, y))
            col += 1
            if col * inter_distance > (n_positions * inter_distance):
                break
        row += 1
        if row > n_positions:
            break
    return positions[:n_positions]


def deploy_random_uniform(n_devices, area_size):
    """Deploy uniforme aleatorio na area."""
    positions = []
    for _ in range(n_devices):
        x = np.random.uniform(0, area_size)
        y = np.random.uniform(0, area_size)
        positions.append((x, y))
    return positions


def deploy_clustered(n_devices, n_clusters, area_size, cluster_std=None):
    """Deploy em clusters gaussianos.

    Devices sao distribuidos em n_clusters, cada cluster com distribuicao gaussiana.
    """
    if cluster_std is None:
        cluster_std = area_size / (n_clusters * 4)

    devices_per_cluster = n_devices // n_clusters
    remainder = n_devices % n_clusters

    # Gera centros dos clusters
    cluster_centers = [(np.random.uniform(cluster_std * 2, area_size - cluster_std * 2),
                        np.random.uniform(cluster_std * 2, area_size - cluster_std * 2))
                       for _ in range(n_clusters)]

    positions = []
    for i, (cx, cy) in enumerate(cluster_centers):
        count = devices_per_cluster + (1 if i < remainder else 0)
        for _ in range(count):
            x = np.clip(np.random.normal(cx, cluster_std), 0, area_size)
            y = np.clip(np.random.normal(cy, cluster_std), 0, area_size)
            positions.append((x, y))

    return positions


def deploy_from_file(filename):
    """Carrega posicoes de arquivo CSV (compativel com ns-3/FLoRa exports).

    Formato esperado: CSV com colunas x,y (header opcional).
    """
    positions = []
    with open(filename) as f:
        reader = csv.reader(f)
        first_row = next(reader)
        # Tenta detectar header
        try:
            x, y = float(first_row[0]), float(first_row[1])
            positions.append((x, y))
        except ValueError:
            pass  # Era header, ignora

        for row in reader:
            if len(row) >= 2:
                positions.append((float(row[0]), float(row[1])))
    return positions


DEPLOYMENT_STRATEGIES = {
    "grid": deploy_grid,
    "circular": deploy_circular,
    "annular": deploy_annular,
    "hexagonal": deploy_hexagonal,
    "random_uniform": deploy_random_uniform,
    "clustered": deploy_clustered,
    "from_file": deploy_from_file,
}


def deploy(strategy, **kwargs):
    """Interface unificada para deployment.

    Parametros:
    - strategy: nome da estrategia (grid, circular, annular, hexagonal,
                random_uniform, clustered, from_file)
    - **kwargs: parametros especificos da estrategia
    """
    func = DEPLOYMENT_STRATEGIES.get(strategy)
    if func is None:
        raise ValueError(f"Estrategia desconhecida: {strategy}. "
                         f"Disponiveis: {list(DEPLOYMENT_STRATEGIES.keys())}")
    return func(**kwargs)
