# Metrics Collection — PyLoRaWAN

## Visão Geral

Durante a simulação, o framework registra continuamente métricas de desempenho relacionadas ao comportamento da rede. Essas métricas incluem estatísticas de entrega de pacotes, atrasos de transmissão, consumo de energia, eventos de colisão e indicadores de qualidade de sinal. Ao final da simulação, os dados coletados são agregados e exportados para análise e visualização.

---

## 1. Arquivos Envolvidos

| Arquivo | Responsabilidade |
|---|---|
| `packet.py` | `Packet` — objeto que carrega todos os campos por transmissão |
| `packettracker.py` | `PacketTracker` — acumula pacotes e computa estatísticas |
| `analytics.py` | `compute_metrics()` — agrega tudo ao final; funções de exportação |
| `plots.py` | 9 funções de visualização geradas pós-simulação |
| `throughput.py` | `NetworkThroughput` — carga de tráfego e throughput analítico |
| `network.py` | Ponto de injeção: `add_packet()`, `energy_log`, colisões |

---

## 2. O Objeto `Packet` — Portador de Métricas

Cada transmissão cria um objeto `Packet` em `_on_device_send()`. Todos os campos são populados progressivamente pelos handlers de evento:

```python
# packet.py — campos relevantes para métricas
self.arrival_time      # float (s)  — timestamp do início da TX
self.rectime           # float (s)  — airtime (= duração da TX)
self.sf                # int        — Spreading Factor
self.tx_power          # int (dBm)  — potência de transmissão
self.bw                # int (Hz)   — largura de banda
self.freq              # float(MHz) — canal selecionado
self.rssi              # float(dBm) — RSSI calculado no momento da TX
self.snr               # float(dB)  — SNR após avaliação do canal
self.snr_mrc           # float(dB)  — SNR combinado MRC (multi-GW, Sprint 9)
self.sinr              # float(dB)  — SINR após interferência
self.noise_floor       # float(dBm) — piso de ruído estimado
self.collided          # bool       — True se perdido por colisão/SINR
self.received          # bool       — True se decodificado com sucesso
self.confirmed         # bool       — True se uplink confirmado (requer ACK)
self.ack_received      # bool       — True se ACK chegou via RX1 ou RX2
self.is_retransmission # bool       — True se retransmissão de pacote colido
self.frame_counter     # int        — FCnt do uplink
self.mic               # bytes      — MIC do frame (OTAA)
self.mic_valid         # bool|None  — resultado da verificação de MIC no NS
self.phy_type          # str        — "CSS" ou "LR-FHSS"
self.interference_per_sf # dict     — potência de interferência acumulada por SF
```

**Campos preenchidos por handler:**

| Campo | Preenchido em | Handler |
|---|---|---|
| `rssi` | criação | `_on_device_send()` via `find_best_gateway()` |
| `snr`, `sinr`, `noise_floor` | avaliação do canal | `_on_tx_end()` → `channel.evaluate_reception()` |
| `collided` | avaliação de interferência | `_on_tx_end()` |
| `ack_received` | entrega de downlink | `_on_rx1_open()` / `_on_rx2_open()` |
| `snr_mrc` | combinação MRC | `_on_device_send()` (Sprint 9, multi-GW) |
| `mic_valid` | verificação OTAA | `ns.on_uplink_received()` |

---

## 3. Registro Contínuo Durante a Simulação

### 3.1 Registro de Pacotes — `PacketTracker.add_packet()`

Chamado em `_on_device_send()` após criar o pacote:

```python
# network.py — _on_device_send()
self.packet_tracker.add_packet(packet)                      # pacote original

# network.py — _on_retransmit()
self.packet_tracker.add_packet(packet, is_retransmission=True)  # retransmissão
```

`PacketTracker` separa os dois conjuntos internamente:

```python
class PacketTracker:
    self.packets               = []   # transmissões originais
    self.retransmitted_packets = []   # retransmissões
    self.packet_history        = {}   # device_id → [Packet, ...]
    self.total_retransmissions = 0
    self.unique_packet_count   = 0
```

Cada pacote também é indexado em `packet_history[device_id]` — permitindo consultas por device via `get_device_stats(device_id)`.

### 3.2 Campos Complementados Após o Registro

O pacote é adicionado ao tracker **antes** da avaliação de canal e interferência. Os campos `snr`, `sinr` e `collided` são preenchidos nos handlers subsequentes (`_on_tx_end`) e ficam visíveis no objeto já armazenado, pois é armazenado por referência:

```
t = T_send:
  Packet criado (rssi preenchido)
  packet_tracker.add_packet(packet)    ← referência armazenada
  scheduler.schedule(airtime, TX_END, ...)

t = T_send + airtime:
  _on_tx_end():
    channel.evaluate_reception(packet, gw)
      packet.snr     = calculado       ← atualiza o mesmo objeto
      packet.sinr    = calculado
      packet.collided = resultado
    # packet_tracker já vê os valores atualizados
```

### 3.3 Amostragem de Energia — `energy_log`

A cada tick de mobilidade (1,0 s), `_on_mobility_update()` registra um snapshot do consumo total da rede:

```python
# network.py — _on_mobility_update()
total_energy = sum(d.energy_model.energy_consumed for d in self.devices) / 1000
self.energy_log.append(total_energy)   # Joules, 1 ponto por segundo
```

`energy_log` é uma lista com `simulation_time` entradas (≈ 3600 para 1h), usada para o plot de throughput de energia ao longo do tempo.

### 3.4 Colisões — Detectadas em `_on_tx_end()`

Para CSS, cada pacote passa pela avaliação do canal ao final da transmissão:

```python
# network.py — _on_tx_end()
received = self.channel.evaluate_reception(packet, device.current_gateway)
packet.received = received
if not received:
    packet.collided = True
```

O campo `packet.collided` é o indicador primário usado em todas as agregações subsequentes.

### 3.5 Downlinks e ACKs

Quando um downlink é entregue com sucesso em RX1 ou RX2, o campo é atualizado no pacote original:

```python
# network.py — _on_rx1_open() / _on_rx2_open()
if dl_packet.packet_type == "ack" and not packet.collided:
    packet.ack_received = True
```

---

## 4. Fontes de Dados por Categoria de Métrica

### 4.1 Entrega de Pacotes (PDR)

| Fonte | O que fornece |
|---|---|
| `PacketTracker.packets` | Total de transmissões originais |
| `PacketTracker.retransmitted_packets` | Total de retransmissões |
| `packet.collided` | Se o pacote foi perdido |
| `packet_history[device_id]` | Estatísticas por device |

`get_stats()` agrega sobre `packets + retransmitted_packets`:

```python
def get_stats(self):
    total    = len(self.packets) + len(self.retransmitted_packets)
    collided = sum(1 for p in packets + retransmitted if p.collided)
    success  = total - collided
    pdr      = success / total * 100
    return {"Total Pacotes": total, "Colisões": collided,
            "Entregues com Sucesso": success, "Taxa de Entrega (PDR)": pdr, ...}
```

### 4.2 Atraso de Transmissão

O atraso é medido como o **airtime** do pacote (`rectime`), em milissegundos. Corresponde ao Time-on-Air (ToA) calculado pela fórmula SX1272:

```python
# analytics.py — compute_metrics()
delays = [p.rectime * 1000   # ms
          for p in all_packets
          if not p.collided and p.rectime is not None]

avg_delay_ms = np.mean(delays)
p50_delay_ms = np.percentile(delays, 50)
p95_delay_ms = np.percentile(delays, 95)
```

O P95 identifica o limite superior de latência experimentado por 95% dos pacotes bem-sucedidos.

### 4.3 Colisões

```python
# analytics.py — compute_metrics()
collision_rate_percent = collisions / total_pkts * 100
retransmission_rate_pct = retransmissions / total_pkts * 100
```

O heatmap espacial (`plot_collision_heatmap`) mapeia colisões para as coordenadas `(device.x, device.y)` no momento da transmissão.

### 4.4 Qualidade de Sinal (SINR)

```python
# analytics.py — compute_metrics()
sinr_values = [p.sinr for p in all_packets if p.sinr is not None]

avg_sinr_db = np.mean(sinr_values)
min_sinr_db = np.min(sinr_values)
p5_sinr_db  = np.percentile(sinr_values, 5)   # limite inferior (5% pior)
```

O P5 de SINR representa as condições de enlace mais adversas enfrentadas durante a simulação.

### 4.5 Energia

Lida diretamente dos acumuladores do `EnergyModel` de cada device (ver `ENERGY_ACCOUNTING.md`):

```python
# analytics.py — compute_metrics()
energy_per_device = [d.energy_model.energy_consumed for d in network.devices]
total_energy_mj   = sum(energy_per_device)
energy_per_sf[sf] = np.mean([d.energy_model.energy_consumed
                              for d in devices if d.sf == sf])
```

### 4.6 PDR vs Distância

Calculado em buckets de 1 km usando a posição **final** do device (após mobilidade):

```python
# analytics.py — compute_metrics()
for p in all_packets:
    dev = next(d for d in network.devices if d.device_id == p.device_id)
    dist_km = int(hypot(dev.x - gw.x, dev.y - gw.y) / 1000)
    dist_buckets[dist_km]["sent"] += 1
    if not p.collided:
        dist_buckets[dist_km]["success"] += 1

pdr_vs_distance = {k: success/sent*100 for k, v in dist_buckets.items()}
```

---

## 5. Agregação Final — `compute_metrics()`

Chamado após `simulate_transmissions()`, consome todos os dados acumulados e retorna um dict estruturado:

```python
metrics = compute_metrics(network)
```

### Estrutura completa retornada

```python
{
  "simulation": {
      "duration_s":      3600,
      "num_devices":     50,
      "num_gateways":    1,
      "region":          "EU868",
      "model_pathloss":  "log_normal_shadowing",
      "deployment_type": "grid",
      "seed":            42,
  },
  "performance": {
      "pdr_percent":              float,    # % geral
      "pdr_per_sf":               {7..12: float},
      "pdr_vs_distance_km":       {"0": %, "1": %, ...},
      "total_packets":            int,
      "collisions":               int,
      "retransmissions":          int,
      "retransmission_rate_percent": float,
      "successful":               int,
      "avg_delay_ms":             float,
      "p50_delay_ms":             float,
      "p95_delay_ms":             float,
      "collision_rate_percent":   float,
  },
  "energy": {
      "total_network_mj":          float,
      "avg_per_device_mj":         float,
      "min_per_device_mj":         float,
      "max_per_device_mj":         float,
      "avg_per_sf_mj":             {7..12: float},
      "breakdown_percent":         {"TX":%, "RX":%, "STANDBY":%, "SLEEP":%},
      "avg_battery_lifetime_days": float | None,
  },
  "radio": {
      "sf_distribution":  {7..12: int},   # devices por SF
      "avg_sinr_db":      float,
      "min_sinr_db":      float,
      "p5_sinr_db":       float,
  },
  "adr": {
      "adjustments_count":      int,      # soma de _adr_changes por device
      "sf_distribution_final":  {7..12: int},
  },
  "analytical": {
      "ps1_per_sf": {sf: {n_devices, toa_s, G, p_no_collision, p_coverage, Ps1}},
      "ps2":        {toa_rx2_s, failed_rx1_pkts, lambda_rx2, G_rx2, Ps2},
  },
}
```

---

## 6. Exportação dos Dados

### 6.1 `export_json(metrics, filename)` — Resumo estruturado

```python
export_json(metrics, "results.json")
# Saída: JSON completo com todas as chaves de compute_metrics()
```

### 6.2 `export_csv_detailed(network, filename)` — Por pacote

Uma linha por pacote (original + retransmissão), ordenado por `arrival_time`:

```
time, device_id, sf, tx_power, freq, bw, rssi, snr, sinr,
collided, confirmed, ack_received, is_retransmission, frame_counter, airtime_ms
```

### 6.3 `export_device_summary(network, filename)` — Por device

Uma linha por device com estado final:

```
device_id, sf, tx_power, freq, class, x, y, coverage,
energy_mj, packets_sent, packets_collided, pdr_percent
```

### 6.4 `export_npz(network, filename)` — Arrays numpy binários

15 arrays compactos para análise numpy/MATLAB, ordenados por `arrival_time`:

**Arrays de pacotes (N = total de transmissões):**

| Array | Dtype | Descrição |
|---|---|---|
| `packet_time` | float64 | `arrival_time` (s) |
| `packet_device_id` | int32 | ID do device |
| `packet_sf` | int32 | SF |
| `packet_rssi` | float64 | RSSI (NaN se ausente) |
| `packet_snr` | float64 | SNR (NaN se ausente) |
| `packet_sinr` | float64 | SINR (NaN se ausente) |
| `packet_collided` | int8 | 1 = colisão, 0 = sucesso |
| `packet_freq` | float64 | Frequência (MHz) |
| `packet_airtime_ms` | float64 | Airtime em ms (NaN se ausente) |

**Arrays de devices (M = num_devices):**

| Array | Dtype | Descrição |
|---|---|---|
| `device_id` | int32 | ID do device |
| `device_sf` | int32 | SF final |
| `device_x` | float64 | Coordenada X (m) |
| `device_y` | float64 | Coordenada Y (m) |
| `device_energy_mj` | float64 | Energia total consumida (mJ) |
| `device_pdr_percent` | float64 | PDR individual (%) |

### 6.5 `PacketTracker.export_device_log(network, filename)` — Log raw

CSV com todos os campos de nível PHY por pacote:

```
Tempo, Device ID, SF, TX Power, RSSI, SNR, Freq, BW, Collided, Confirmed, Frame Counter
```

---

## 7. Visualizações — `plots.py`

Todas as funções leem diretamente de `network.packet_tracker` e `network.devices`. São geradas após o fim da simulação via `plot_all(network, prefix="")`:

| Função | Arquivo gerado | Fonte de dados |
|---|---|---|
| `plot_sf_distribution` | `plot_sf_distribution.png` | `device.sf` |
| `plot_pdr_per_sf` | `plot_pdr_per_sf.png` | `packet.collided` por SF |
| `plot_pdr_vs_distance` | `plot_pdr_vs_distance.png` | `packet.collided` + `device.(x,y)` |
| `plot_energy_per_sf` | `plot_energy_per_sf.png` | `energy_model.energy_consumed` por SF |
| `plot_delay_cdf` | `plot_delay_cdf.png` | `packet.rectime` (não colidos) |
| `plot_sinr_distribution` | `plot_sinr_distribution.png` | `packet.sinr` |
| `plot_collision_heatmap` | `plot_collision_heatmap.png` | `packet.collided` + `device.(x,y)` |
| `plot_throughput_over_time` | `plot_throughput_over_time.png` | `arrival_time` (janela 60 s) |
| `plot_battery_soc_timeline` | `plot_battery_soc.png` | `battery.soc_percent()` final |

---

## 8. Fluxo Completo de Coleta

```
simulate_transmissions()
│
├── [t=0..T] Loop de eventos
│     │
│     ├── DEVICE_SEND @ t
│     │     ├── Packet(device_id, sf, tx_power, bw, freq, rssi, t, airtime)
│     │     ├── packet_tracker.add_packet(packet)       ← registro imediato
│     │     └── scheduler.schedule(airtime, TX_END)
│     │
│     ├── PACKET_TX_END @ t + airtime
│     │     ├── channel.evaluate_reception(packet, gw)
│     │     │     ├── packet.snr    = calculado          ← atualiza objeto existente
│     │     │     ├── packet.sinr   = calculado
│     │     │     └── packet.collided = resultado
│     │     └── scheduler.schedule(rd1, RX1_OPEN)
│     │
│     ├── RX1/RX2_OPEN @ t + rd
│     │     └── packet.ack_received = True (se DL entregue)
│     │
│     └── MOBILITY_UPDATE @ t = k × 1s
│           └── energy_log.append(Σ energy_consumed / 1000)  ← snapshot J
│
└── [fim da simulação]
      │
      ├── detect_collisions_and_interference()   ← reavalia colisões SINR
      │
      ├── metrics = compute_metrics(network)
      │     ├── packet_tracker.get_stats()          → PDR, colisões, retransmissões
      │     ├── [p.collided for p in all_packets]   → pdr_per_sf, collision_rate
      │     ├── [p.rectime for p in all_packets]    → delay P50, P95
      │     ├── [p.sinr for p in all_packets]       → avg/min/P5 SINR
      │     ├── [d.energy_model... for d in devices]→ energia total/por SF/breakdown
      │     ├── [d.battery.soc_percent()]           → lifetime estimado
      │     ├── dist_buckets por 1km                → pdr_vs_distance
      │     └── analytical_ps1_per_sf() + ps2_rx2() → validação analítica
      │
      ├── export_json(metrics, "results.json")
      ├── export_csv_detailed(network, "results_detailed.csv")
      ├── export_device_summary(network, "device_summary.csv")
      ├── export_npz(network, "results.npz")
      └── plot_all(network, prefix="")            → 9 PNGs
```

---

## 9. Métricas Disponíveis por Consulta em Tempo Real

Durante a simulação, é possível consultar o estado atual a qualquer momento:

```python
# Estatísticas acumuladas até o momento
stats = network.packet_tracker.get_stats()
# {"Total Pacotes": N, "Colisões": C, "Taxa de Entrega (PDR)": X.XX}

# Por device
dev_stats = network.packet_tracker.get_device_stats(device_id)

# Energia atual de um device
energy_mj = device.energy_model.energy_consumed

# Cobertura
covered = sum(1 for d in network.devices if d.coverage_status)
```

`get_stats()` recalcula sobre todos os pacotes em `packets + retransmitted_packets` a cada chamada — sem estado intermediário mantido.
