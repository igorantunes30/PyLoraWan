# Performance Evaluation Metrics — PyLoRaWAN

## Visão Geral

O simulador coleta métricas de desempenho em dois níveis: contínuo durante a execução (por pacote, via `PacketTracker`) e agregado ao final da simulação (via `compute_metrics()` em `analytics.py`). As métricas abrangem confiabilidade, qualidade de sinal, energia e escalabilidade, e são exportadas em quatro formatos para pós-processamento e visualização.

---

## 1. Coleta Contínua — `PacketTracker`

`PacketTracker` em `packettracker.py` armazena referências a todos os objetos `Packet` durante a simulação:

```python
class PacketTracker:
    def __init__(self):
        self.packets              = []   # pacotes originais
        self.retransmitted_packets = []  # retransmissões
        self.packet_history       = {}   # device_id → [Packet, ...]
        self.total_retransmissions = 0
        self.unique_packet_count  = 0
```

`add_packet(packet, is_retransmission=False)` separa pacotes originais de retransmissões e mantém histórico por device. Como os objetos `Packet` são armazenados por referência, campos preenchidos em etapas posteriores (SNR em `_on_tx_end`, ACK em `_on_rx1/rx2_open`) ficam disponíveis automaticamente na agregação final.

---

## 2. Packet Delivery Ratio (PDR)

### 2.1 PDR Global

Calculado em `PacketTracker.get_stats()`:

```python
total_packets      = len(packets) + len(retransmitted_packets)
collided_packets   = sum(1 for p in all if p.collided)
successful_packets = total_packets - collided_packets
pdr = successful_packets / total_packets * 100
```

**Critério de sucesso:** pacote não colidido (`packet.collided = False`). A colisão é determinada em `channel.py` por sobreposição temporal + limiar de energia (G14) ou pelo teste de preamble lock (G13).

### 2.2 PDR por SF

`compute_metrics()` em `analytics.py` segmenta por SF:

```python
for sf in range(7, 13):
    sf_packets = [p for p in all_packets if p.sf == sf]
    success = sum(1 for p in sf_packets if not p.collided)
    pdr_per_sf[sf] = success / len(sf_packets) * 100
```

### 2.3 PDR por Distância

Agrupado em buckets de 1 km até o gateway mais próximo:

```python
dist_km = int(hypot(dev.x - gw.x, dev.y - gw.y) / 1000)
dist_buckets[dist_km]["sent"] += 1
if not p.collided:
    dist_buckets[dist_km]["success"] += 1

pdr_vs_distance = {str(k): success/sent * 100 for k, v in sorted(dist_buckets)}
```

---

## 3. Taxa de Colisão

`compute_metrics()` expõe:

```python
"collision_rate_percent": collisions / total_packets * 100
```

`collisions = stats.get("Colisoes", 0)` conta pacotes com `packet.collided = True`. A taxa de retransmissão é calculada separadamente:

```python
retransmission_rate_percent = retransmissions / total_packets * 100
```

---

## 4. Estatísticas de Delay de Transmissão

O delay considerado é o `rectime` (airtime) dos pacotes recebidos com sucesso, convertido para ms:

```python
delays = [p.rectime * 1000 for p in all_packets if not p.collided and p.rectime]

"avg_delay_ms": np.mean(delays)
"p50_delay_ms": np.percentile(delays, 50)
"p95_delay_ms": np.percentile(delays, 95)
```

`rectime` é o Time-on-Air calculado por `calculate_airtime()` no device (função do SF, BW, payload e CR). Para SF7/BW125: ~0.071 s (71 ms); para SF12/BW125: ~2.465 s (2465 ms).

---

## 5. Distribuição de Spreading Factor

Contagem de devices por SF no estado final da simulação:

```python
sf_distribution = {sf: sum(1 for d in network.devices if d.sf == sf)
                   for sf in range(7, 13)}
```

Este campo reflete o SF após todos os ajustes ADR aplicados durante a simulação — não o SF inicial. Inclui também a distribuição final no bloco `"adr"`:

```python
"adr": {
    "adjustments_count": adr_adjustments,     # soma de _adr_changes por device
    "sf_distribution_final": sf_distribution,
}
```

---

## 6. Distribuição de SINR

`process_uplink()` em `gateway.py` calcula o SINR de cada pacote usando modelo energy-based (G14):

```python
sinr_linear = signal_linear / (interference_power_linear + noise_linear)
packet.sinr = 10 * log10(sinr_linear)
```

`compute_metrics()` agrega:

```python
sinr_values = [p.sinr for p in all_packets if p.sinr is not None]

"avg_sinr_db": np.mean(sinr_values)
"min_sinr_db": np.min(sinr_values)
"p5_sinr_db":  np.percentile(sinr_values, 5)
```

---

## 7. Consumo de Energia por Device

Agregado de `EnergyModel.energy_consumed` de cada device:

```python
total_energy_mj    = sum(d.energy_model.energy_consumed for d in network.devices)
energy_per_device  = [d.energy_model.energy_consumed for d in network.devices]
energy_per_sf      = {sf: mean(energy) for devices with d.sf == sf}

"energy": {
    "total_network_mj":       total_energy_mj,
    "avg_per_device_mj":      np.mean(energy_per_device),
    "min_per_device_mj":      np.min(energy_per_device),
    "max_per_device_mj":      np.max(energy_per_device),
    "avg_per_sf_mj":          energy_per_sf,         # {7: x, 8: x, ..., 12: x}
    "breakdown_percent":      energy_breakdown_pct,  # {"TX": %, "RX": %, "STANDBY": %, "SLEEP": %}
    "avg_battery_lifetime_days": float | None,
}
```

O breakdown percentual soma `energy_breakdown` de todos os devices e normaliza pelo total:

```python
breakdown_totals[state] += d.energy_model.energy_breakdown[state]
energy_breakdown_pct[state.name] = breakdown_totals[state] / total_energy_mj * 100
```

---

## 8. Throughput de Rede

`plot_throughput_over_time()` em `plots.py` computa o throughput com janela deslizante de 60 s:

```python
window_s = 60
for t in time_bins:
    window_packets = [p for p in successful if t - window_s <= p.arrival_time < t]
    throughput = len(window_packets) / window_s   # pacotes/s
```

O throughput instantâneo não é armazenado em `compute_metrics()` — é calculado diretamente dos timestamps de `packet.arrival_time` nos gráficos.

---

## 9. Estrutura Completa de `compute_metrics()`

```python
metrics = {
    "simulation": {
        "duration_s", "num_devices", "num_gateways",
        "region", "model_pathloss", "deployment_type", "seed"
    },
    "performance": {
        "pdr_percent",                 # PDR global (%)
        "pdr_per_sf",                  # {7: %, 8: %, ..., 12: %}
        "pdr_vs_distance_km",          # {"0": %, "1": %, ...}
        "total_packets",
        "collisions",
        "retransmissions",
        "retransmission_rate_percent",
        "successful",
        "avg_delay_ms", "p50_delay_ms", "p95_delay_ms",
        "collision_rate_percent",
    },
    "energy": {
        "total_network_mj",
        "avg_per_device_mj", "min_per_device_mj", "max_per_device_mj",
        "avg_per_sf_mj",               # {7: mJ, ..., 12: mJ}
        "breakdown_percent",           # {"TX": %, "RX": %, "STANDBY": %, "SLEEP": %}
        "avg_battery_lifetime_days",   # None se battery_capacity_mah não configurado
    },
    "radio": {
        "sf_distribution",             # {7: n, ..., 12: n}
        "avg_sinr_db", "min_sinr_db", "p5_sinr_db",
    },
    "adr": {
        "adjustments_count",
        "sf_distribution_final",
    },
    "analytical": {
        "ps1_per_sf",                  # Ps1 ALOHA por SF (Magrin et al. 2017)
        "ps2",                         # Ps2 fallback RX2
    }
}
```

---

## 10. Exportação

Quatro funções de exportação em `analytics.py`:

| Função | Arquivo | Conteúdo |
|---|---|---|
| `export_json(metrics)` | `results.json` | Dict completo de `compute_metrics()` |
| `export_npz(network)` | `results.npz` | 15 arrays numpy: 9 por pacote + 6 por device |
| `export_csv_detailed(network)` | `results_detailed.csv` | 1 linha por pacote: time, sf, rssi, snr, sinr, collided, ack, airtime |
| `export_device_summary(network)` | `device_summary.csv` | 1 linha por device: sf, pos, energy, pdr |

**Arrays do NPZ** (`export_npz()`):

| Array | Tipo | Descrição |
|---|---|---|
| `packet_time` | float64 | `arrival_time` (s) |
| `packet_device_id` | int32 | ID do device |
| `packet_sf` | int32 | SF do pacote |
| `packet_rssi` | float64 | RSSI (dBm), NaN se ausente |
| `packet_snr` | float64 | SNR (dB), NaN se ausente |
| `packet_sinr` | float64 | SINR (dB), NaN se ausente |
| `packet_collided` | int8 | 0/1 |
| `packet_freq` | float64 | Frequência (MHz) |
| `packet_airtime_ms` | float64 | Airtime (ms), NaN se ausente |
| `device_id` | int32 | — |
| `device_sf` | int32 | SF final pós-ADR |
| `device_x`, `device_y` | float64 | Posição final (m) |
| `device_energy_mj` | float64 | Energia acumulada total |
| `device_pdr_percent` | float64 | PDR individual (%) |

---

## 11. Visualizações — `plot_all()` (`plots.py`)

`plot_all(network, prefix="")` gera 9 gráficos em sequência:

| Função | Arquivo | Métrica |
|---|---|---|
| `plot_sf_distribution` | `plot_sf_distribution.png` | Barras com contagem de devices por SF |
| `plot_pdr_per_sf` | `plot_pdr_per_sf.png` | PDR (%) por SF |
| `plot_pdr_vs_distance` | `plot_pdr_vs_distance.png` | PDR (%) em buckets de 1 km |
| `plot_energy_per_sf` | `plot_energy_per_sf.png` | Energia média por device por SF (mJ) |
| `plot_delay_cdf` | `plot_delay_cdf.png` | CDF do airtime dos pacotes bem-sucedidos |
| `plot_sinr_distribution` | `plot_sinr_distribution.png` | Histograma de SINR (dB) |
| `plot_collision_heatmap` | `plot_collision_heatmap.png` | Heatmap espacial de colisões (bins=20) |
| `plot_throughput_over_time` | `plot_throughput_over_time.png` | Throughput (pkts/s) com janela de 60 s |
| `plot_battery_soc_timeline` | `plot_battery_soc.png` | Histograma de SoC final por device |

---

## 12. Modelos Analíticos de Validação

`analytics.py` inclui modelos analíticos para comparação com os resultados simulados:

| Função | Modelo | Referência |
|---|---|---|
| `analytical_pdr_aloha` | ALOHA puro: `P = exp(−2G)` | G = n×ToA / (T×M) |
| `analytical_pdr_per_sf` | ALOHA por SF | Carga G por SF independente |
| `analytical_ps1_per_sf` | Ps1 = P_no_collision × P_coverage | Magrin et al. 2017 |
| `analytical_ps2_rx2` | Ps2 = P_no_collision_RX2 × P_coverage_SF12 | RX2 canal único, SF12 |
| `compare_ps1_ps2` | PDR total = (1−cr)×Ps1 + cr×(Ps1 + (1−Ps1)×Ps2) | `confirmed_ratio=0.3` |

`compare_with_analytical(network)` retorna a comparação por SF com a diferença absoluta entre PDR simulado e analítico, indicando se a diferença está dentro de 15%.
