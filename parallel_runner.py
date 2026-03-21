"""Executor paralelo multi-seed.

Permite rodar a mesma simulacao com multiplas seeds em paralelo
usando ProcessPoolExecutor.
"""

import random
import numpy as np
from concurrent.futures import ProcessPoolExecutor, as_completed


def _run_single(config_params, seed):
    """Executa simulacao unica com dado seed (worker function)."""
    random.seed(seed)
    np.random.seed(seed)

    from network import Network
    network = Network(**config_params)
    network.simulate_transmissions()

    stats = network.packet_tracker.get_stats()
    total_energy = sum(d.energy_model.energy_consumed for d in network.devices)

    return {
        "seed": seed,
        "pdr": stats.get("Taxa de Entrega (PDR)", 0),
        "collisions": stats.get("Colisoes", 0),
        "total_packets": stats.get("Total Pacotes", 0),
        "retransmissions": stats.get("Retransmissoes", 0),
        "total_energy_mj": total_energy,
    }


def run_multiple_seeds(config_params, seeds, n_workers=4):
    """Executa simulacao com multiplas seeds em paralelo.

    Parametros:
    - config_params: dict de parametros para Network()
    - seeds: lista de seeds
    - n_workers: numero de workers paralelos
    """
    results = []

    with ProcessPoolExecutor(max_workers=n_workers) as executor:
        futures = {
            executor.submit(_run_single, config_params, seed): seed
            for seed in seeds
        }
        for future in as_completed(futures):
            seed = futures[future]
            try:
                result = future.result()
                results.append(result)
                print(f"Seed {seed}: PDR={result['pdr']:.1f}%, "
                      f"Collisions={result['collisions']}, "
                      f"Energy={result['total_energy_mj']:.0f} mJ")
            except Exception as e:
                print(f"Seed {seed}: ERRO - {e}")

    return aggregate_results(results)


def aggregate_results(results):
    """Agrega resultados de multiplas seeds."""
    if not results:
        return {}

    pdrs = [r["pdr"] for r in results]
    collisions = [r["collisions"] for r in results]
    energies = [r["total_energy_mj"] for r in results]

    return {
        "n_seeds": len(results),
        "pdr_mean": float(np.mean(pdrs)),
        "pdr_std": float(np.std(pdrs)),
        "pdr_min": float(np.min(pdrs)),
        "pdr_max": float(np.max(pdrs)),
        "collisions_mean": float(np.mean(collisions)),
        "collisions_std": float(np.std(collisions)),
        "energy_mean_mj": float(np.mean(energies)),
        "energy_std_mj": float(np.std(energies)),
        "individual_results": results,
    }


if __name__ == "__main__":
    from parametors import (num_devices, num_gateways, area_size, lambda_rate, speed,
                            sf_range, tx_power, frequency_mhz, ht_m, hr_m,
                            bw, cr, pl, simulation_time, adr_enabled, model_pathloss)

    config = {
        "num_devices": num_devices, "num_gateways": num_gateways,
        "area_size": area_size, "lambda_rate": lambda_rate, "speed": speed,
        "sf_range": sf_range, "tx_power": tx_power,
        "frequency_mhz": frequency_mhz, "ht_m": ht_m, "hr_m": hr_m,
        "bw": bw, "cr": cr, "pl": pl,
        "simulation_time": simulation_time, "adr_enabled": adr_enabled,
        "model_pathloss": model_pathloss,
    }

    seeds = [42, 123, 456, 789, 1024]
    results = run_multiple_seeds(config, seeds, n_workers=4)
    print(f"\nResultados Agregados:")
    print(f"  PDR: {results['pdr_mean']:.1f}% +/- {results['pdr_std']:.1f}%")
    print(f"  Collisions: {results['collisions_mean']:.0f} +/- {results['collisions_std']:.0f}")
    print(f"  Energy: {results['energy_mean_mj']:.0f} +/- {results['energy_std_mj']:.0f} mJ")
