"""Modelos Analiticos e Saida de Dados.

Implementa:
- Modelo analitico ALOHA puro para validacao
- Modelo analitico por SF
- Saida JSON/CSV/HDF5
- Metricas expandidas
- Comparacao com FLoRa/ns-3
"""

import json
import csv
import numpy as np


# ============================================================
# Modelos Analiticos de Validacao
# ============================================================

def analytical_pdr_aloha(n_devices, toa_s, period_s, n_channels=1):
    """Probabilidade de sucesso via modelo ALOHA puro.

    G = carga de trafego oferecida por canal.
    P_success = exp(-2*G) para ALOHA puro.
    """
    G = n_devices * toa_s / (period_s * n_channels)
    return np.exp(-2 * G)


def analytical_pdr_per_sf(n_devices, sf_distribution, toa_per_sf, period_s, n_channels):
    """PDR analitico considerando ortogonalidade parcial entre SFs.

    Parametros:
    - n_devices: numero total de devices
    - sf_distribution: dict {sf: fracao} (ex: {7: 0.3, 8: 0.2, ...})
    - toa_per_sf: dict {sf: toa_s}
    - period_s: periodo medio entre transmissoes
    - n_channels: numero de canais
    """
    pdr = {}
    for sf in range(7, 13):
        n_sf = n_devices * sf_distribution.get(sf, 0)
        toa = toa_per_sf.get(sf, 0)
        if toa <= 0 or n_sf <= 0:
            pdr[sf] = 1.0
            continue
        G_sf = n_sf * toa / (period_s * n_channels)
        pdr[sf] = np.exp(-2 * G_sf)
    return pdr


def analytical_pdr_with_capture(n_devices, toa_s, period_s, n_channels, capture_threshold_db=6):
    """PDR analitico com capture effect.

    Modelo simplificado que considera probabilidade de captura
    quando ha colisao entre 2 pacotes.
    """
    G = n_devices * toa_s / (period_s * n_channels)
    p_no_collision = np.exp(-2 * G)
    p_two_colliders = 2 * G * np.exp(-2 * G)
    p_capture = 0.5  # Probabilidade simplificada de captura
    return p_no_collision + p_two_colliders * p_capture


# ============================================================
# Modelo Analitico Ps1/Ps2 (G11) — equivalente ao LoRaWANSim
# Referencia: Magrin et al. 2017 "Performance Evaluation of LoRa"
# ============================================================

def _toa_analytical(sf, bw=125000, pl=20, cr=1):
    """Calcula Time-on-Air (s) para parametros dados — formula SX1272."""
    import math
    Tsym = (2.0 ** sf) / bw
    Tpream = (8 + 4.25) * Tsym
    DE = 1 if Tsym > 0.016 else 0
    H, CRC = 0, 1
    numerator = 8.0 * pl - 4.0 * sf + 28 + 16 * CRC - 20 * H
    denominator = 4.0 * (sf - 2 * DE)
    payloadSymbNB = 8 + max(math.ceil(numerator / denominator) * (cr + 4), 0)
    return Tpream + payloadSymbNB * Tsym


def analytical_ps1_per_sf(network):
    """Calcula Ps1 por SF: probabilidade de sucesso na janela RX1.

    Ps1(SF) = P_no_collision(SF) x P_coverage(SF)

    onde:
    - P_no_collision = exp(-2*G_SF/M)  [ALOHA puro por canal]
    - G_SF = n_SF * ToA_SF / (avg_period * M)  [carga oferecida]
    - P_coverage = fracao de devices com SF=s que tem cobertura (do simulador)

    Equivalente ao modelo Ps1 do LoRaWANSim (Magrin et al. 2017).
    """
    import math
    n_channels = len(network.frequency_mhz) if isinstance(network.frequency_mhz, list) else 1
    avg_period = 1.0 / max(network.lambda_rate, 0.001)

    results = {}
    for sf in range(7, 13):
        sf_devices = [d for d in network.devices if d.sf == sf]
        n_sf = len(sf_devices)
        if n_sf == 0:
            results[sf] = {
                'n_devices': 0, 'toa_s': 0.0, 'G': 0.0,
                'p_no_collision': 1.0, 'p_coverage': 0.0, 'Ps1': 0.0,
            }
            continue

        toa = _toa_analytical(sf, bw=125000, pl=network.pl, cr=network.cr)
        G_sf = (n_sf * toa) / (avg_period * n_channels)
        p_no_coll = math.exp(-2 * G_sf)

        # P(coverage) — estimado do estado final dos devices na simulacao
        covered = sum(1 for d in sf_devices if d.coverage_status)
        p_coverage = covered / n_sf

        results[sf] = {
            'n_devices': n_sf,
            'toa_s': round(toa, 6),
            'G': round(G_sf, 6),
            'p_no_collision': round(p_no_coll, 4),
            'p_coverage': round(p_coverage, 4),
            'Ps1': round(p_no_coll * p_coverage, 4),
        }
    return results


def analytical_ps2_rx2(network):
    """Calcula Ps2: probabilidade de sucesso na janela RX2 dado falha em RX1.

    RX2 usa SF12 (EU868: 869.525 MHz) — canal unico dedicado ao fallback.
    Ps2 = P_no_collision_RX2 x P_coverage_SF12

    onde:
    - P_no_collision_RX2 = exp(-2 * G_RX2)
    - G_RX2 = lambda_failed * ToA_SF12 / 1  [so 1 canal RX2]
    - lambda_failed = taxa de pacotes que falharam RX1 (confirmed, colidiram)
    """
    import math
    n_channels_rx2 = 1  # RX2 e canal unico
    toa_rx2 = _toa_analytical(sf=12, bw=125000, pl=13, cr=1)  # payload minimo (ACK)

    # Taxa de falhas em RX1 que precisam de RX2 (confirmed e colidiram)
    all_packets = network.packet_tracker.packets
    failed_rx1 = sum(1 for p in all_packets if p.collided and p.confirmed)
    sim_time = max(network.simulation_time, 1.0)
    lambda_rx2 = failed_rx1 / sim_time

    G_rx2 = lambda_rx2 * toa_rx2 / n_channels_rx2
    p_no_coll_rx2 = math.exp(-2 * G_rx2) if G_rx2 > 0 else 1.0

    # P(coverage SF12) — todos os devices tem SF12 como fallback em RX2
    covered = sum(1 for d in network.devices if d.coverage_status)
    p_coverage_rx2 = covered / max(len(network.devices), 1)

    return {
        'toa_rx2_s': round(toa_rx2, 6),
        'failed_rx1_pkts': failed_rx1,
        'lambda_rx2': round(lambda_rx2, 6),
        'G_rx2': round(G_rx2, 6),
        'p_no_collision': round(p_no_coll_rx2, 4),
        'p_coverage': round(p_coverage_rx2, 4),
        'Ps2': round(p_no_coll_rx2 * p_coverage_rx2, 4),
    }


def compare_ps1_ps2(network):
    """Compara PDR simulado vs modelo analitico Ps1/Ps2 por SF.

    Retorna dicionario com:
    - ps1_per_sf: Ps1 analitico por SF
    - ps2: Ps2 analitico (RX2 fallback)
    - overall_analytical_pdr: PDR total estimado = media ponderada por SF
    - simulation_pdr: PDR da simulacao
    - per_sf_comparison: tabela comparativa sim vs analytical por SF
    """
    stats = network.packet_tracker.get_stats()
    sim_pdr = stats.get("Taxa de Entrega (PDR)", 0) / 100.0

    ps1_results = analytical_ps1_per_sf(network)
    ps2_result = analytical_ps2_rx2(network)
    ps2 = ps2_result['Ps2']

    # PDR analitico ponderado pela distribuicao de SFs
    total_n = sum(r['n_devices'] for r in ps1_results.values())
    if total_n > 0:
        weighted_ps1 = sum(r['Ps1'] * r['n_devices'] for r in ps1_results.values()) / total_n
    else:
        weighted_ps1 = 0.0

    # PDR total = Ps1 + (1 - Ps1) * Ps2 [confirmed messages only]
    confirmed_ratio = 0.3  # 30% confirmed (parametors.py default)
    # Nao-confirmados: PDR = Ps1
    # Confirmados com RX2: PDR = Ps1 + (1-Ps1)*Ps2
    analytical_pdr = (1 - confirmed_ratio) * weighted_ps1 + confirmed_ratio * (
        weighted_ps1 + (1 - weighted_ps1) * ps2)

    # Comparativo por SF (sim vs analytical)
    all_packets = network.packet_tracker.packets
    per_sf_comparison = {}
    for sf in range(7, 13):
        sf_pkts = [p for p in all_packets if p.sf == sf]
        if sf_pkts:
            sim_sf_pdr = sum(1 for p in sf_pkts if not p.collided) / len(sf_pkts)
            anal_sf_pdr = ps1_results[sf].get('Ps1', 0.0)
            per_sf_comparison[sf] = {
                'n_packets': len(sf_pkts),
                'sim_pdr': round(sim_sf_pdr, 4),
                'analytical_ps1': anal_sf_pdr,
                'difference': round(abs(sim_sf_pdr - anal_sf_pdr), 4),
            }

    return {
        'ps1_per_sf': ps1_results,
        'ps2': ps2_result,
        'weighted_ps1': round(weighted_ps1, 4),
        'analytical_pdr': round(analytical_pdr, 4),
        'simulation_pdr': round(sim_pdr, 4),
        'difference': round(abs(analytical_pdr - sim_pdr), 4),
        'within_15pct': abs(analytical_pdr - sim_pdr) < 0.15,
        'per_sf_comparison': per_sf_comparison,
    }


# ============================================================
# Metricas Expandidas
# ============================================================

def compute_metrics(network):
    """Computa metricas completas da simulacao."""
    from energymodel import RadioState
    stats = network.packet_tracker.get_stats()
    all_packets = network.packet_tracker.packets + network.packet_tracker.retransmitted_packets

    # PDR e energia por SF
    pdr_per_sf = {}
    energy_per_sf = {}
    for sf in range(7, 13):
        sf_packets = [p for p in all_packets if p.sf == sf]
        if sf_packets:
            success = sum(1 for p in sf_packets if not p.collided)
            pdr_per_sf[sf] = round(success / len(sf_packets) * 100, 2)
        else:
            pdr_per_sf[sf] = 0
        sf_devices = [d for d in network.devices if d.sf == sf]
        if sf_devices:
            energy_per_sf[sf] = round(
                float(np.mean([d.energy_model.energy_consumed for d in sf_devices])), 2)
        else:
            energy_per_sf[sf] = 0

    # Distribuicao de SF
    sf_distribution = {}
    for sf in range(7, 13):
        sf_distribution[sf] = sum(1 for d in network.devices if d.sf == sf)

    # Energia total e breakdown por estado
    total_energy_mj = sum(d.energy_model.energy_consumed for d in network.devices)
    energy_per_device = [d.energy_model.energy_consumed for d in network.devices]
    breakdown_totals = {state: 0.0 for state in RadioState}
    for d in network.devices:
        for state, val in d.energy_model.energy_breakdown.items():
            breakdown_totals[state] += val
    energy_breakdown_pct = {}
    if total_energy_mj > 0:
        for state, val in breakdown_totals.items():
            energy_breakdown_pct[state.name] = round(val / total_energy_mj * 100, 1)
    else:
        energy_breakdown_pct = {state.name: 0.0 for state in RadioState}

    # Battery lifetime (dias) estimado por device
    battery_lifetime_days = []
    for d in network.devices:
        if d.battery is not None and network.simulation_time > 0:
            initial_soc = 100.0
            current_soc = d.battery.soc_percent()
            consumed_pct = initial_soc - current_soc
            if consumed_pct > 0:
                rate_pct_per_s = consumed_pct / network.simulation_time
                lifetime_s = 100.0 / rate_pct_per_s
                battery_lifetime_days.append(lifetime_s / 86400)

    # Delays
    delays = []
    for p in all_packets:
        if not p.collided and p.rectime:
            delays.append(p.rectime * 1000)  # ms

    # SINR
    sinr_values = [p.sinr for p in all_packets if p.sinr is not None]

    # ADR
    adr_adjustments = 0
    for d in network.devices:
        if hasattr(d, '_adr_changes'):
            adr_adjustments += d._adr_changes

    # Retransmission rate
    total_pkts = stats.get("Total Pacotes", 0)
    retransmissions = stats.get("Retransmissoes", 0)
    retransmission_rate_pct = round(retransmissions / max(total_pkts, 1) * 100, 2)

    # PDR vs distancia: buckets de 1km
    dist_buckets = {}
    gw = network.gateways[0] if network.gateways else None
    if gw is not None:
        for p in all_packets:
            dev = next((d for d in network.devices if d.device_id == p.device_id), None)
            if dev is None:
                continue
            dist_km = int(np.hypot(dev.x - gw.x, dev.y - gw.y) / 1000)
            if dist_km not in dist_buckets:
                dist_buckets[dist_km] = {"sent": 0, "success": 0}
            dist_buckets[dist_km]["sent"] += 1
            if not p.collided:
                dist_buckets[dist_km]["success"] += 1
    pdr_vs_distance = {
        str(k): round(v["success"] / max(v["sent"], 1) * 100, 1)
        for k, v in sorted(dist_buckets.items())
    }

    metrics = {
        "simulation": {
            "duration_s": network.simulation_time,
            "num_devices": network.num_devices,
            "num_gateways": network.num_gateways,
            "region": network.region.name if hasattr(network.region, 'name') else str(network.region),
            "model_pathloss": network.model_pathloss,
            "deployment_type": network.deployment_type,
            "seed": getattr(network, '_seed_used', None),
        },
        "performance": {
            "pdr_percent": stats.get("Taxa de Entrega (PDR)", 0),
            "pdr_per_sf": pdr_per_sf,
            "pdr_vs_distance_km": pdr_vs_distance,
            "total_packets": total_pkts,
            "collisions": stats.get("Colisoes", 0),
            "retransmissions": retransmissions,
            "retransmission_rate_percent": retransmission_rate_pct,
            "successful": stats.get("Entregues com Sucesso", 0),
            "avg_delay_ms": round(float(np.mean(delays)), 3) if delays else 0,
            "p50_delay_ms": round(float(np.percentile(delays, 50)), 3) if delays else 0,
            "p95_delay_ms": round(float(np.percentile(delays, 95)), 3) if delays else 0,
            "collision_rate_percent": round(
                stats.get("Colisoes", 0) / max(total_pkts, 1) * 100, 2),
        },
        "energy": {
            "total_network_mj": round(total_energy_mj, 2),
            "avg_per_device_mj": round(float(np.mean(energy_per_device)), 2) if energy_per_device else 0,
            "min_per_device_mj": round(float(np.min(energy_per_device)), 2) if energy_per_device else 0,
            "max_per_device_mj": round(float(np.max(energy_per_device)), 2) if energy_per_device else 0,
            "avg_per_sf_mj": energy_per_sf,
            "breakdown_percent": energy_breakdown_pct,
            "avg_battery_lifetime_days": round(float(np.mean(battery_lifetime_days)), 1) if battery_lifetime_days else None,
        },
        "radio": {
            "sf_distribution": sf_distribution,
            "avg_sinr_db": round(float(np.mean(sinr_values)), 2) if sinr_values else 0,
            "min_sinr_db": round(float(np.min(sinr_values)), 2) if sinr_values else 0,
            "p5_sinr_db": round(float(np.percentile(sinr_values, 5)), 2) if sinr_values else 0,
        },
        "adr": {
            "adjustments_count": adr_adjustments,
            "sf_distribution_final": sf_distribution,
        },
    }

    try:
        metrics["analytical"] = {
            "ps1_per_sf": {str(sf): v for sf, v in analytical_ps1_per_sf(network).items()},
            "ps2": analytical_ps2_rx2(network),
        }
    except Exception:
        metrics["analytical"] = {}

    return metrics


# ============================================================
# Saida de Dados
# ============================================================

def export_json(metrics, filename="results.json"):
    """Exporta metricas para JSON."""
    with open(filename, 'w') as f:
        json.dump(metrics, f, indent=2, default=str)


def export_npz(network, filename="results.npz"):
    """Exporta dados de simulacao em formato binario compacto (numpy .npz).

    Equivalente ao HDF5 sem dependencia de h5py.
    Compativel com numpy.load() e MATLAB (via scipy.io.loadmat workaround).

    Arrays salvos:
    - packet_time, packet_device_id, packet_sf, packet_rssi, packet_snr,
      packet_sinr, packet_collided, packet_freq, packet_airtime_ms
    - device_id, device_sf, device_x, device_y, device_energy_mj,
      device_pdr_percent
    """
    all_packets = network.packet_tracker.packets + network.packet_tracker.retransmitted_packets
    all_packets_sorted = sorted(all_packets, key=lambda p: p.arrival_time)

    n = len(all_packets_sorted)
    pkt_time = np.zeros(n)
    pkt_dev = np.zeros(n, dtype=np.int32)
    pkt_sf = np.zeros(n, dtype=np.int32)
    pkt_rssi = np.full(n, np.nan)
    pkt_snr = np.full(n, np.nan)
    pkt_sinr = np.full(n, np.nan)
    pkt_collided = np.zeros(n, dtype=np.int8)
    pkt_freq = np.zeros(n)
    pkt_airtime_ms = np.full(n, np.nan)

    for i, p in enumerate(all_packets_sorted):
        pkt_time[i] = p.arrival_time
        pkt_dev[i] = p.device_id
        pkt_sf[i] = p.sf
        if p.rssi is not None:
            pkt_rssi[i] = p.rssi
        if p.snr is not None:
            pkt_snr[i] = p.snr
        if p.sinr is not None:
            pkt_sinr[i] = p.sinr
        pkt_collided[i] = int(p.collided)
        pkt_freq[i] = p.freq if p.freq else 0.0
        if p.rectime is not None:
            pkt_airtime_ms[i] = p.rectime * 1000

    m = len(network.devices)
    dev_id = np.array([d.device_id for d in network.devices], dtype=np.int32)
    dev_sf = np.array([d.sf for d in network.devices], dtype=np.int32)
    dev_x = np.array([d.x for d in network.devices])
    dev_y = np.array([d.y for d in network.devices])
    dev_energy = np.array([d.energy_model.energy_consumed for d in network.devices])
    dev_pdr = np.zeros(m)
    for j, d in enumerate(network.devices):
        ds = network.packet_tracker.get_device_stats(d.device_id)
        tot = ds.get("Total Pacotes", 0)
        col = ds.get("Colisoes", 0)
        dev_pdr[j] = (tot - col) / tot * 100 if tot > 0 else 0.0

    np.savez_compressed(
        filename,
        # Pacotes
        packet_time=pkt_time,
        packet_device_id=pkt_dev,
        packet_sf=pkt_sf,
        packet_rssi=pkt_rssi,
        packet_snr=pkt_snr,
        packet_sinr=pkt_sinr,
        packet_collided=pkt_collided,
        packet_freq=pkt_freq,
        packet_airtime_ms=pkt_airtime_ms,
        # Devices
        device_id=dev_id,
        device_sf=dev_sf,
        device_x=dev_x,
        device_y=dev_y,
        device_energy_mj=dev_energy,
        device_pdr_percent=dev_pdr,
    )


def export_csv_detailed(network, filename="results_detailed.csv"):
    """Exporta dados detalhados por pacote para CSV."""
    all_packets = network.packet_tracker.packets + network.packet_tracker.retransmitted_packets
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["time", "device_id", "sf", "tx_power", "freq", "bw",
                         "rssi", "snr", "sinr", "collided", "confirmed",
                         "ack_received", "is_retransmission", "frame_counter",
                         "airtime_ms"])
        for p in sorted(all_packets, key=lambda x: x.arrival_time):
            writer.writerow([
                f"{p.arrival_time:.4f}",
                p.device_id, p.sf, p.tx_power, p.freq, p.bw,
                f"{p.rssi:.2f}" if p.rssi else "N/A",
                f"{p.snr:.2f}" if p.snr else "N/A",
                f"{p.sinr:.2f}" if p.sinr else "N/A",
                p.collided, p.confirmed, p.ack_received,
                p.is_retransmission, p.frame_counter,
                f"{p.rectime * 1000:.2f}" if p.rectime else "N/A",
            ])


def export_device_summary(network, filename="device_summary.csv"):
    """Exporta resumo por device."""
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["device_id", "sf", "tx_power", "freq", "class",
                         "x", "y", "coverage", "energy_mj",
                         "packets_sent", "packets_collided", "pdr_percent"])
        for d in network.devices:
            d_stats = network.packet_tracker.get_device_stats(d.device_id)
            total = d_stats.get("Total Pacotes", 0)
            collided = d_stats.get("Colisoes", 0)
            pdr = ((total - collided) / total * 100) if total > 0 else 0
            writer.writerow([
                d.device_id, d.sf, d.tx_power, d.freq, d.lorawan_class,
                f"{d.x:.1f}", f"{d.y:.1f}", d.coverage_status,
                f"{d.energy_model.energy_consumed:.2f}",
                total, collided, f"{pdr:.1f}",
            ])


# ============================================================
# Validacao Cruzada
# ============================================================

def compare_with_analytical(network):
    """Compara resultados da simulacao com modelo analitico."""
    stats = network.packet_tracker.get_stats()
    sim_pdr = stats.get("Taxa de Entrega (PDR)", 0) / 100.0

    # Calcula parametros para modelo analitico
    n_devices = network.num_devices
    avg_toa = float(np.mean([d.calculate_airtime() for d in network.devices]))
    period = 1.0 / max(network.lambda_rate, 0.001)
    n_channels = len(network.frequency_mhz) if isinstance(network.frequency_mhz, list) else 1

    anal_pdr = analytical_pdr_aloha(n_devices, avg_toa, period, n_channels)

    return {
        "simulation_pdr": round(sim_pdr, 4),
        "analytical_pdr": round(anal_pdr, 4),
        "difference": round(abs(sim_pdr - anal_pdr), 4),
        "within_10pct": abs(sim_pdr - anal_pdr) < 0.10,
        "ps1_ps2": compare_ps1_ps2(network),
    }
