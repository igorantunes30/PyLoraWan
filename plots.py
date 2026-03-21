"""Graficos expandidos para o PyLoRaWAN Simulator — Sprint 7.

Funcoes:
- plot_pdr_per_sf: PDR por SF (barras)
- plot_pdr_vs_distance: PDR vs distancia ao GW
- plot_energy_per_sf: Energia media por SF
- plot_delay_cdf: CDF dos delays de recepcao
- plot_sinr_distribution: Histograma de SINR
- plot_collision_heatmap: Heatmap espacial de colisoes
- plot_throughput_over_time: Throughput acumulado ao longo do tempo
- plot_battery_soc_over_time: SoC de bateria por device (se disponivel)
- plot_all: Gera todos os graficos acima
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def plot_pdr_per_sf(network, filename="plot_pdr_per_sf.png"):
    """PDR (%) por Spreading Factor."""
    all_packets = network.packet_tracker.packets + network.packet_tracker.retransmitted_packets

    sfs = list(range(7, 13))
    pdr_values = []
    counts = []
    for sf in sfs:
        pkts = [p for p in all_packets if p.sf == sf]
        if pkts:
            success = sum(1 for p in pkts if not p.collided)
            pdr_values.append(success / len(pkts) * 100)
            counts.append(len(pkts))
        else:
            pdr_values.append(0)
            counts.append(0)

    fig, ax = plt.subplots(figsize=(7, 5))
    bars = ax.bar(sfs, pdr_values, color='steelblue', edgecolor='black')
    for bar, count in zip(bars, counts):
        if count > 0:
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.5,
                    f'n={count}', ha='center', va='bottom', fontsize=8)
    ax.set_xlabel("Spreading Factor")
    ax.set_ylabel("PDR (%)")
    ax.set_title("PDR por Spreading Factor")
    ax.set_ylim(0, 110)
    ax.set_xticks(sfs)
    ax.grid(axis='y', alpha=0.4)
    plt.tight_layout()
    plt.savefig(filename, dpi=100)
    plt.close()


def plot_pdr_vs_distance(network, filename="plot_pdr_vs_distance.png", bucket_km=1):
    """PDR (%) vs distancia ao gateway mais proximo (buckets de bucket_km km)."""
    all_packets = network.packet_tracker.packets + network.packet_tracker.retransmitted_packets
    if not network.gateways:
        return
    gw = network.gateways[0]

    buckets = {}
    for p in all_packets:
        dev = next((d for d in network.devices if d.device_id == p.device_id), None)
        if dev is None:
            continue
        dist_km = int(np.hypot(dev.x - gw.x, dev.y - gw.y) / 1000)
        if dist_km not in buckets:
            buckets[dist_km] = {"sent": 0, "success": 0}
        buckets[dist_km]["sent"] += 1
        if not p.collided:
            buckets[dist_km]["success"] += 1

    if not buckets:
        return

    sorted_keys = sorted(buckets.keys())
    dist_labels = [f"{k}-{k+1}km" for k in sorted_keys]
    pdr_vals = [buckets[k]["success"] / max(buckets[k]["sent"], 1) * 100 for k in sorted_keys]
    n_vals = [buckets[k]["sent"] for k in sorted_keys]

    fig, ax = plt.subplots(figsize=(max(7, len(sorted_keys)), 5))
    ax.plot(range(len(sorted_keys)), pdr_vals, marker='o', color='teal', linewidth=2)
    ax.fill_between(range(len(sorted_keys)), pdr_vals, alpha=0.2, color='teal')
    ax.set_xticks(range(len(sorted_keys)))
    ax.set_xticklabels(dist_labels, rotation=30, ha='right')
    ax.set_xlabel("Distancia ao Gateway")
    ax.set_ylabel("PDR (%)")
    ax.set_title("PDR vs Distancia ao Gateway")
    ax.set_ylim(0, 110)
    ax.grid(alpha=0.4)
    # Anotacao de contagem
    for i, (pdr, n) in enumerate(zip(pdr_vals, n_vals)):
        ax.annotate(f'n={n}', (i, pdr), textcoords="offset points",
                    xytext=(0, 6), ha='center', fontsize=7)
    plt.tight_layout()
    plt.savefig(filename, dpi=100)
    plt.close()


def plot_energy_per_sf(network, filename="plot_energy_per_sf.png"):
    """Energia media consumida por device, agrupado por SF."""
    sfs = list(range(7, 13))
    means, stds, counts = [], [], []
    for sf in sfs:
        devs = [d for d in network.devices if d.sf == sf]
        if devs:
            energies = [d.energy_model.energy_consumed for d in devs]
            means.append(float(np.mean(energies)))
            stds.append(float(np.std(energies)))
            counts.append(len(devs))
        else:
            means.append(0)
            stds.append(0)
            counts.append(0)

    fig, ax = plt.subplots(figsize=(7, 5))
    bars = ax.bar(sfs, means, yerr=stds, color='coral', edgecolor='black',
                  capsize=4, error_kw=dict(elinewidth=1.2))
    for bar, count in zip(bars, counts):
        if count > 0:
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + max(stds) * 0.05 + 0.01,
                    f'n={count}', ha='center', va='bottom', fontsize=8)
    ax.set_xlabel("Spreading Factor")
    ax.set_ylabel("Energia Media (mJ)")
    ax.set_title("Energia Media por Spreading Factor")
    ax.set_xticks(sfs)
    ax.grid(axis='y', alpha=0.4)
    plt.tight_layout()
    plt.savefig(filename, dpi=100)
    plt.close()


def plot_delay_cdf(network, filename="plot_delay_cdf.png"):
    """CDF dos delays de recepcao (pacotes bem-sucedidos)."""
    all_packets = network.packet_tracker.packets + network.packet_tracker.retransmitted_packets
    delays_ms = [p.rectime * 1000 for p in all_packets
                 if not p.collided and p.rectime is not None]
    if not delays_ms:
        return

    delays_sorted = np.sort(delays_ms)
    cdf = np.arange(1, len(delays_sorted) + 1) / len(delays_sorted)

    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot(delays_sorted, cdf * 100, color='darkgreen', linewidth=2)
    # Marcadores de percentil
    for pct in [50, 90, 95]:
        val = float(np.percentile(delays_sorted, pct))
        ax.axvline(val, linestyle='--', alpha=0.6, label=f'P{pct}={val:.1f}ms')
    ax.set_xlabel("Delay de Recepcao (ms)")
    ax.set_ylabel("CDF (%)")
    ax.set_title("CDF do Delay de Recepcao (pacotes com sucesso)")
    ax.legend(fontsize=9)
    ax.grid(alpha=0.4)
    plt.tight_layout()
    plt.savefig(filename, dpi=100)
    plt.close()


def plot_sinr_distribution(network, filename="plot_sinr_distribution.png"):
    """Histograma da distribuicao de SINR."""
    all_packets = network.packet_tracker.packets + network.packet_tracker.retransmitted_packets
    sinr_vals = [p.sinr for p in all_packets if p.sinr is not None]
    if not sinr_vals:
        return

    fig, ax = plt.subplots(figsize=(7, 5))
    ax.hist(sinr_vals, bins=30, color='mediumpurple', edgecolor='black', alpha=0.8)
    ax.axvline(float(np.mean(sinr_vals)), color='red', linestyle='--',
               label=f'Media={np.mean(sinr_vals):.1f} dB')
    ax.axvline(float(np.percentile(sinr_vals, 5)), color='orange', linestyle=':',
               label=f'P5={np.percentile(sinr_vals, 5):.1f} dB')
    ax.set_xlabel("SINR (dB)")
    ax.set_ylabel("Contagem de Pacotes")
    ax.set_title("Distribuicao de SINR")
    ax.legend(fontsize=9)
    ax.grid(axis='y', alpha=0.4)
    plt.tight_layout()
    plt.savefig(filename, dpi=100)
    plt.close()


def plot_collision_heatmap(network, filename="plot_collision_heatmap.png", bins=20):
    """Heatmap espacial das colisoes de pacotes."""
    all_packets = network.packet_tracker.packets + network.packet_tracker.retransmitted_packets
    collided_pkts = [p for p in all_packets if p.collided]
    if not collided_pkts:
        return

    xs, ys = [], []
    for p in collided_pkts:
        dev = next((d for d in network.devices if d.device_id == p.device_id), None)
        if dev:
            xs.append(dev.x)
            ys.append(dev.y)

    if not xs:
        return

    fig, ax = plt.subplots(figsize=(7, 6))
    h = ax.hist2d(xs, ys, bins=bins, cmap='hot_r',
                  range=[[0, network.area_size], [0, network.area_size]])
    plt.colorbar(h[3], ax=ax, label='Numero de Colisoes')

    # Plotar gateways
    for gw in network.gateways:
        ax.scatter(gw.x, gw.y, marker='*', color='blue', s=200,
                   zorder=5, label=f'GW {gw.gateway_id}')

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Heatmap Espacial de Colisoes")
    ax.legend(fontsize=8)
    plt.tight_layout()
    plt.savefig(filename, dpi=100)
    plt.close()


def plot_throughput_over_time(network, filename="plot_throughput_over_time.png",
                              window_s=60):
    """Throughput de pacotes bem-sucedidos ao longo do tempo (janela deslizante)."""
    all_packets = network.packet_tracker.packets + network.packet_tracker.retransmitted_packets
    success_pkts = sorted([p for p in all_packets if not p.collided],
                          key=lambda p: p.arrival_time)
    if not success_pkts or network.simulation_time <= 0:
        return

    bins = np.arange(0, network.simulation_time + window_s, window_s)
    times = [p.arrival_time for p in success_pkts]
    counts, _ = np.histogram(times, bins=bins)
    bin_centers = (bins[:-1] + bins[1:]) / 2

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.step(bin_centers, counts, where='mid', color='darkcyan', linewidth=1.5)
    ax.fill_between(bin_centers, counts, step='mid', alpha=0.2, color='darkcyan')
    ax.set_xlabel("Tempo (s)")
    ax.set_ylabel(f"Pacotes com Sucesso / {window_s}s")
    ax.set_title("Throughput ao Longo do Tempo")
    ax.grid(alpha=0.4)
    plt.tight_layout()
    plt.savefig(filename, dpi=100)
    plt.close()


def plot_battery_soc_timeline(network, filename="plot_battery_soc.png"):
    """SoC final de bateria de todos os devices (histograma)."""
    soc_values = [d.battery.soc_percent() for d in network.devices
                  if d.battery is not None]
    if not soc_values:
        return

    fig, ax = plt.subplots(figsize=(7, 5))
    ax.hist(soc_values, bins=20, range=(0, 100), color='forestgreen',
            edgecolor='black', alpha=0.8)
    ax.axvline(float(np.mean(soc_values)), color='red', linestyle='--',
               label=f'Media={np.mean(soc_values):.1f}%')
    ax.set_xlabel("SoC de Bateria (%)")
    ax.set_ylabel("Numero de Devices")
    ax.set_title(f"Distribuicao de SoC de Bateria (final da simulacao, n={len(soc_values)})")
    ax.set_xlim(0, 100)
    ax.legend(fontsize=9)
    ax.grid(axis='y', alpha=0.4)
    plt.tight_layout()
    plt.savefig(filename, dpi=100)
    plt.close()


def plot_sf_distribution(network, filename="plot_sf_distribution.png"):
    """Distribuicao de SF dos devices (barras com contagem)."""
    from collections import Counter
    sf_counts = Counter(d.sf for d in network.devices)
    sfs = sorted(sf_counts.keys())
    vals = [sf_counts[sf] for sf in sfs]

    fig, ax = plt.subplots(figsize=(7, 5))
    bars = ax.bar(sfs, vals, color='royalblue', edgecolor='black')
    for bar, val in zip(bars, vals):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.1,
                str(val), ha='center', va='bottom', fontsize=10, fontweight='bold')
    ax.set_xlabel("Spreading Factor")
    ax.set_ylabel("Numero de Devices")
    ax.set_title(f"Distribuicao de SFs ({network.region.name if hasattr(network.region, 'name') else ''}, "
                 f"{network.deployment_type} deploy)")
    ax.set_xticks(sfs)
    ax.grid(axis='y', alpha=0.4)
    plt.tight_layout()
    plt.savefig(filename, dpi=100)
    plt.close()


def plot_all(network, prefix=""):
    """Gera todos os graficos expandidos.

    Parametros:
    - network: instancia de Network apos simulacao
    - prefix: prefixo para nomes de arquivo (ex: 'run1_')
    """
    p = prefix
    plot_sf_distribution(network, f"{p}plot_sf_distribution.png")
    plot_pdr_per_sf(network, f"{p}plot_pdr_per_sf.png")
    plot_pdr_vs_distance(network, f"{p}plot_pdr_vs_distance.png")
    plot_energy_per_sf(network, f"{p}plot_energy_per_sf.png")
    plot_delay_cdf(network, f"{p}plot_delay_cdf.png")
    plot_sinr_distribution(network, f"{p}plot_sinr_distribution.png")
    plot_collision_heatmap(network, f"{p}plot_collision_heatmap.png")
    plot_throughput_over_time(network, f"{p}plot_throughput_over_time.png")
    plot_battery_soc_timeline(network, f"{p}plot_battery_soc.png")
