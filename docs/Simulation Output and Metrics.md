# Simulation Output and Metrics — PyLoRaWAN

## Visão Geral

Durante a simulação, o framework registra estatísticas detalhadas de cada pacote via `PacketTracker`. Ao final da execução, `compute_metrics()` agrega os dados em métricas de desempenho e os exporta em quatro formatos estruturados para análise e visualização.

---

## 1. Métricas Primárias Coletadas

| Métrica | Fonte | Campo/Cálculo |
|---|---|---|
| Packet Delivery Ratio (PDR) | `PacketTracker.get_stats()` | `(total − collided) / total × 100` |
| Pacotes transmitidos | `PacketTracker.unique_packet_count` | Contagem de `packets` (originais) |
| Pacotes recebidos | `get_stats()["Entregues com Sucesso"]` | `total − collided` |
| Taxa de colisão | `compute_metrics()` | `collisions / total × 100` |
| Energia por device | `device.energy_model.energy_consumed` | mJ acumulado por `EnergyModel` |
| Distribuição de SINR | `packet.sinr` | Energy-based G14 em `gateway.py` |
| Distribuição de SF | `device.sf` | Estado final pós-ADR |

---

## 2. Packet Delivery Ratio (PDR)

### 2.1 PDR Global

```python
# packettracker.py — get_stats()
total_packets    = len(packets) + len(retransmitted_packets)
collided_packets = sum(1 for p in all if p.collided)
pdr = (total_packets - collided_packets) / total_packets * 100
```

### 2.2 PDR por SF

```python
# analytics.py — compute_metrics()
for sf in range(7, 13):
    sf_packets = [p for p in all_packets if p.sf == sf]
    success    = sum(1 for p in sf_packets if not p.collided)
    pdr_per_sf[sf] = success / len(sf_packets) * 100
```

### 2.3 PDR por Distância

Agrupado em buckets de 1 km ao gateway mais próximo:

```python
dist_km = int(hypot(dev.x - gw.x, dev.y - gw.y) / 1000)
pdr_vs_distance[str(dist_km)] = success / sent * 100
```

---

## 3. Pacotes Transmitidos e Recebidos

`PacketTracker.add_packet()` separa pacotes originais de retransmissões:

```python
def add_packet(self, packet, is_retransmission=False):
    if is_retransmission:
        self.retransmitted_packets.append(packet)
        self.total_retransmissions += 1
    else:
        self.packets.append(packet)
        self.unique_packet_count += 1
    self.packet_history[packet.device_id].append(packet)
```

Acesso por device via `get_device_stats(device_id)`:

```python
{
    "Total Pacotes":          int,
    "Colisões":               int,
    "Retransmissões":         int,
    "Entregues com Sucesso":  int,
    "Taxa de Entrega (PDR)":  "XX.XX%"
}
```

---

## 4. Taxa de Colisão

```python
# compute_metrics()
"collision_rate_percent":     collisions / total_packets * 100
"retransmission_rate_percent": retransmissions / total_packets * 100
```

Uma colisão é determinada em dois pontos do pipeline:

1. **Preamble timing (G13)** — `channel.py`: dois pacotes com sobreposição temporal; o que chegar depois perde o lock de preamble se o primeiro tiver RSSI mais alto
2. **Energy ratio (G14)** — `gateway.py`: `SINR = S / (I + N)`; a matriz de interferência (Semtech/Goursaud) define o threshold por par de SFs

---

## 5. Consumo de Energia por Device

Cada `EnergyModel` acumula energia por estado:

```python
# energymodel.py — transition()
energy = current_ma * voltage * duration   # mJ
self.energy_consumed                      += energy
self.energy_breakdown[current_state]      += energy
```

`compute_metrics()` agrega:

```python
"energy": {
    "total_network_mj":          sum(d.energy_model.energy_consumed),
    "avg_per_device_mj":         np.mean(energy_per_device),
    "min_per_device_mj":         np.min(energy_per_device),
    "max_per_device_mj":         np.max(energy_per_device),
    "avg_per_sf_mj":             {sf: mean for sf in 7..12},
    "breakdown_percent":         {"TX": %, "RX": %, "STANDBY": %, "SLEEP": %},
    "avg_battery_lifetime_days": float | None,
}
```

---

## 6. Distribuição de SINR

Coletado de `packet.sinr` (calculado em `gateway.py`):

```python
sinr_values = [p.sinr for p in all_packets if p.sinr is not None]

"radio": {
    "avg_sinr_db": np.mean(sinr_values),
    "min_sinr_db": np.min(sinr_values),
    "p5_sinr_db":  np.percentile(sinr_values, 5),
}
```

O percentil 5 identifica os 5% de pacotes com piores condições de sinal — relevante para análise de confiabilidade no limite da cobertura.

---

## 7. Distribuição de Spreading Factor

Contagem final após todos os ajustes ADR:

```python
sf_distribution = {sf: sum(1 for d in network.devices if d.sf == sf)
                   for sf in range(7, 13)}
```

Exposto nos blocos `"radio"` e `"adr"` de `compute_metrics()`. A distribuição inicial é aleatória (SF sorteado em `sf_range`); após ADR, concentra-se nos SFs mínimos que satisfazem o link budget de cada device.

---

## 8. Estrutura Exportada por `compute_metrics()`

```python
metrics = {
    "simulation":   {duration_s, num_devices, num_gateways, region, model_pathloss,
                     deployment_type, seed},
    "performance":  {pdr_percent, pdr_per_sf, pdr_vs_distance_km,
                     total_packets, collisions, retransmissions,
                     retransmission_rate_percent, successful,
                     avg_delay_ms, p50_delay_ms, p95_delay_ms,
                     collision_rate_percent},
    "energy":       {total_network_mj, avg/min/max_per_device_mj,
                     avg_per_sf_mj, breakdown_percent,
                     avg_battery_lifetime_days},
    "radio":        {sf_distribution, avg_sinr_db, min_sinr_db, p5_sinr_db},
    "adr":          {adjustments_count, sf_distribution_final},
    "analytical":   {ps1_per_sf, ps2},   # modelos ALOHA/Ps1/Ps2 para validação
}
```

---

## 9. Formatos de Exportação

Quatro funções em `analytics.py` exportam os resultados ao final da simulação:

| Função | Arquivo de saída | Conteúdo |
|---|---|---|
| `export_json(metrics)` | `results.json` | Dict completo de `compute_metrics()` |
| `export_npz(network)` | `results.npz` | 15 arrays numpy: 9 por pacote + 6 por device |
| `export_csv_detailed(network)` | `results_detailed.csv` | 1 linha por pacote com time, sf, rssi, snr, sinr, collided, ack, airtime |
| `export_device_summary(network)` | `device_summary.csv` | 1 linha por device com sf, posição, energia, pdr |

**Arrays do NPZ** — carregáveis diretamente com `numpy.load()`:

```python
data = np.load("results.npz")
# Arrays por pacote
data["packet_time"]      # arrival_time (s)
data["packet_rssi"]      # dBm — NaN se não calculado
data["packet_sinr"]      # dB  — NaN se não calculado
data["packet_collided"]  # 0/1
data["packet_airtime_ms"]
# Arrays por device
data["device_energy_mj"]
data["device_pdr_percent"]
data["device_sf"]        # SF final pós-ADR
```

---

## 10. Visualizações

`plot_all(network, prefix="")` em `plots.py` gera 9 gráficos automaticamente:

| Gráfico | Arquivo | Métrica |
|---|---|---|
| SF distribution | `plot_sf_distribution.png` | Contagem de devices por SF final |
| PDR por SF | `plot_pdr_per_sf.png` | PDR (%) por SF |
| PDR vs distância | `plot_pdr_vs_distance.png` | PDR (%) em buckets de 1 km |
| Energia por SF | `plot_energy_per_sf.png` | Energia média (mJ) por device por SF |
| Delay CDF | `plot_delay_cdf.png` | CDF do airtime (pacotes bem-sucedidos) |
| SINR distribution | `plot_sinr_distribution.png` | Histograma de SINR (dB) |
| Collision heatmap | `plot_collision_heatmap.png` | Heatmap espacial de colisões |
| Throughput | `plot_throughput_over_time.png` | Pacotes/s com janela deslizante de 60 s |
| Battery SoC | `plot_battery_soc.png` | Histograma de SoC final (se bateria configurada) |

---

## 11. Validação Analítica

`compare_with_analytical(network)` compara o PDR simulado com o modelo analítico Ps1/Ps2:

```python
# PDR total analítico (Magrin et al. 2017)
# Não-confirmados: PDR = Ps1
# Confirmados com RX2: PDR = Ps1 + (1 − Ps1) × Ps2
analytical_pdr = (1 − 0.3) × Ps1 + 0.3 × (Ps1 + (1 − Ps1) × Ps2)

{
    "simulation_pdr":    float,     # PDR da simulação
    "analytical_pdr":    float,     # PDR do modelo ALOHA Ps1/Ps2
    "difference":        float,     # |sim − analytical|
    "within_15pct":      bool,      # diferença < 15%
    "per_sf_comparison": {sf: {...}}
}
```
