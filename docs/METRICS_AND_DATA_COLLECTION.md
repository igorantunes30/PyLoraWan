# Metrics and Data Collection Module

**PyLoRaWAN Simulator — Technical Reference**

---

## Table of Contents

1. [Module Overview](#1-module-overview)
2. [Packet Data Model](#2-packet-data-model)
3. [PacketTracker](#3-packettracker)
4. [NetworkThroughput](#4-networkthroughput)
5. [Analytical Validation Models](#5-analytical-validation-models)
6. [compute_metrics — Full Metrics Pipeline](#6-compute_metrics--full-metrics-pipeline)
7. [Data Export](#7-data-export)
8. [Visualization — plots.py](#8-visualization--plotspy)
9. [Collision Detection (Legacy)](#9-collision-detection-legacy)
10. [Data Flow Diagram](#10-data-flow-diagram)
11. [Numerical Examples](#11-numerical-examples)
12. [Comparison with ns-3 and FLoRa](#12-comparison-with-ns-3-and-flora)

---

## 1. Module Overview

The metrics and data collection subsystem captures every transmission event during the simulation, computes performance indicators, validates results against analytical models, and exports data in multiple formats.

### Source Files

| File | Responsibility |
|------|---------------|
| `packet.py` | `Packet` dataclass — primary data carrier |
| `packettracker.py` | `PacketTracker` — per-packet and per-device statistics |
| `throughput.py` | `NetworkThroughput` — traffic load and throughput calculations |
| `analytics.py` | Analytical models, `compute_metrics()`, export functions |
| `plots.py` | 9 matplotlib visualization functions |
| `network.py` | `detect_collisions_and_interference()`, `print_statistics()` |

### Metrics Taxonomy

```
Metrics
├── Performance
│   ├── PDR (%)           — overall and per SF
│   ├── PDR vs Distance   — 1km buckets
│   ├── Collision rate    — % of total TX
│   ├── Retransmission rate
│   ├── Delay             — P50, P95 (ms)
│   └── Throughput        — pkt/s over time
├── Radio
│   ├── SINR distribution — mean, P5 (dB)
│   ├── SF distribution   — count per SF
│   └── RSSI              — per packet
├── Energy
│   ├── Total network (mJ)
│   ├── Per device (mean/min/max)
│   ├── Per SF (mean)
│   ├── State breakdown (TX/RX/SLEEP/STANDBY %)
│   └── Battery lifetime (days, if battery model active)
├── ADR
│   └── SF adjustments count
└── Analytical
    ├── ALOHA pure PDR
    ├── Ps1 per SF (RX1 success probability)
    └── Ps2 (RX2 fallback probability)
```

---

## 2. Packet Data Model

### 2.1 `Packet` Class (`packet.py`)

The `Packet` object is the central data structure — every uplink and retransmission creates one instance. It is populated progressively as it travels through the simulation pipeline.

```python
class Packet:
    def __init__(self, device_id, sf, tx_power, bw, freq, rssi,
                 arrival_time, rectime, packet_type="uplink"):
```

### 2.2 Field Reference

| Field | Type | Set by | Description |
|-------|------|--------|-------------|
| `packet_id` | UUID | `__init__` | Globally unique identifier (`uuid.uuid4()`) |
| `device_id` | int | `__init__` | Source device ID |
| `sf` | int | `__init__` | Spreading Factor (7–12) |
| `tx_power` | float | `__init__` | TX power (dBm) |
| `bw` | int | `__init__` | Bandwidth (Hz): 125000/250000/500000 |
| `freq` | float | `__init__` | Carrier frequency (MHz) |
| `rssi` | float | `__init__` | Received Signal Strength at GW (dBm) |
| `arrival_time` | float | `__init__` | TX start time (s) |
| `rectime` | float | `__init__` | Time-on-Air (s) — same as airtime |
| `tx_start` | float | `__init__` | Alias for `arrival_time` |
| `tx_end` | float | `__init__` | `arrival_time + rectime` |
| `collided` | bool | `channel.py` / `gateway.py` | Packet lost due to interference |
| `received` | bool | `channel.py` | Packet successfully decoded |
| `packet_type` | str | `__init__` | "uplink", "downlink", "critical" |
| `snr` | float\|None | `gateway.py` | SNR at gateway (dB) |
| `snr_mrc` | float\|None | `network.py` | MRC-combined SNR from multiple GWs (dB) |
| `mrc_gw_count` | int | `network.py` | Number of GWs that received the packet |
| `sinr` | float\|None | `gateway.py` | SINR after interference accumulation (dB) |
| `sir` | float\|None | `gateway.py` | Signal-to-Interference Ratio (dB) |
| `noise_floor` | float\|None | `gateway.py` | Thermal noise floor (dBm) |
| `confirmed` | bool | `network.py` | Requires ACK (30% of packets) |
| `ack_received` | bool | `network.py` | ACK delivered in RX1 or RX2 |
| `mic` | bytes\|None | `network.py` | 4-byte MIC (computed by device using NwkSKey) |
| `mic_valid` | bool\|None | `network_server` | NS MIC verification result |
| `is_retransmission` | bool | `network.py` | True for `_on_retransmit` packets |
| `retry_count` | int | — | Legacy field (currently unused) |
| `gateway_id` | int\|None | `gateway.py` | Receiving gateway ID |
| `frame_counter` | int | `network.py` | Uplink frame counter (FCntUp) |
| `phy_type` | str | `network.py` | "CSS" or "LR-FHSS" |
| `lrfhss_fragments` | list\|None | `network.py` | List of `LRFHSSFragment` (LR-FHSS only) |
| `interference_per_sf` | dict | `gateway.py` | `{sf: power_linear}` — energy accumulated per SF |

### 2.3 Packet `__repr__`

```python
<Packet ID=3f7a... | Device=12 | SF=9 | TP: 14 RSSI=-118.45 dBm |
 SINR=8.32 dB | SNR=12.10 dB | NF=-132.50 dBm | Collided=False | Type=uplink>
```

### 2.4 Packet Lifecycle

```
EndDevice._on_device_send()
  → Packet created: device_id, sf, tx_power, bw, freq, rssi, arrival_time, rectime
  → packet.frame_counter = device.frame_counter_up
  → packet.mic = compute_frame_mic(...)
  → packet.confirmed = (random() < 0.3)

Gateway.process_uplink()
  → packet.snr = RSSI - noise_floor
  → packet.noise_floor = calculate_noise_floor(bw)
  → packet.sinr = calculated after interference accumulation
  → packet.gateway_id = self.gw_id
  → packet.interference_per_sf = {sf: energy} accumulated

Channel.evaluate_reception()
  → packet.collided = True/False (based on SINR vs interference matrix)
  → packet.received = True/False

NS.on_uplink_received()
  → packet.mic_valid = verify_frame_mic(...)

_on_rx1_open() / _on_rx2_open()
  → packet.ack_received = True (if DL delivered)
```

---

## 3. PacketTracker

### 3.1 Class Overview (`packettracker.py`)

```python
class PacketTracker:
    packets: list[Packet]                    # original uplinks
    retransmitted_packets: list[Packet]      # retransmissions only
    packet_history: dict[int, list[Packet]]  # device_id → all packets
    total_retransmissions: int               # global retransmit count
    unique_packet_count: int                 # original packets count
```

### 3.2 `add_packet(packet, is_retransmission=False)`

Routes packet to the correct list and updates the per-device history:

```
if is_retransmission:
    retransmitted_packets.append(packet)
    total_retransmissions += 1
else:
    packets.append(packet)
    unique_packet_count += 1

packet_history[packet.device_id].append(packet)  # always
```

**Important:** Both original and retransmission packets are stored in `packet_history[device_id]`. This means `get_device_stats()` includes retransmissions for that device.

### 3.3 `get_stats()` — Global Statistics

```python
{
    "Total Pacotes":          total_packets,          # packets + retransmitted_packets
    "Colisões":               collided_count,          # sum(p.collided)
    "Retransmissões":         total_retransmissions,   # counter
    "Entregues com Sucesso":  total - collided,        # non-collided
    "Taxa de Entrega (PDR)":  pdr_percent              # 0–100
}
```

**PDR formula:**
```
PDR = (total_packets - collided_packets) / total_packets × 100
```

Note: `total_packets = len(packets) + len(retransmitted_packets)`. Retransmissions are included in both numerator and denominator — a retransmit that succeeds counts as one success; its original collision also counts as one failure.

### 3.4 `get_device_stats(device_id)`

Returns per-device statistics from `packet_history[device_id]`:

```python
{
    "Device ID":              device_id,
    "Total Pacotes":          total,
    "Colisões":               collided,
    "Retransmissões":         retransmissions,          # those in retransmitted_packets
    "Entregues com Sucesso":  total - collided,
    "Taxa de Entrega (PDR)":  f"{pdr:.2f}%"
}
```

**Retransmission detection:**
```python
retransmissions = sum(1 for p in packets if p in self.retransmitted_packets)
```
This uses object identity (`in` operator on list) — O(R) per device, where R = len(retransmitted_packets).

### 3.5 `export_device_log(network, filename)`

Exports all packets (sorted by arrival time) to CSV:

```
Columns: Tempo, Device ID, SF, TX Power, RSSI, SNR, Freq, BW, Collided,
         Confirmed, Frame Counter
```

### 3.6 `clear_packets()`

Resets all state: clears all lists, dicts, and counters. Used between simulation runs.

---

## 4. NetworkThroughput

### 4.1 Class Overview (`throughput.py`)

`NetworkThroughput` provides analytical throughput estimates based on ALOHA theory. It is instantiated in `Network.__init__()` as `self.throughput_calculator`.

### 4.2 `calculate_sf_proportions()`

```
sf_proportions[sf] = count(devices with sf) / total_devices
```

Returns `{7: 0.17, 8: 0.17, ..., 12: 0.17}` for uniform SF distribution.

### 4.3 `calculate_avg_airtime()`

```
avg_airtime[sf] = mean(device.calculate_airtime() for devices with sf)
```

Each device recomputes its current airtime (using the SX1272 ToA formula with current SF/BW/PL).

### 4.4 `calculate_traffic_load()`

Traffic load G per SF:
```
G[sf] = 46 × n_sf × Ps × ToA_sf
```

Note: The constant `46` in the formula is a legacy coefficient — it does not follow the standard ALOHA definition `G = n × toa / T`. The canonical formula from `analytics.py` is:
```
G_sf = n_sf × ToA_sf / (period × n_channels)
```

### 4.5 `calculate_total_throughput()`

```
T = Σ_sf [ 46 × Ps[sf] × Nc × exp(-2 × G[sf] / 10000) ]
```

Returns estimated throughput in bits/s. This is an approximation combining ALOHA success probability with the number of covered devices (`Nc`).

---

## 5. Analytical Validation Models

### 5.1 Overview (`analytics.py`)

Three levels of analytical models, increasing in accuracy:

| Function | Model | Notes |
|----------|-------|-------|
| `analytical_pdr_aloha` | Pure ALOHA (aggregate) | Single G for all devices |
| `analytical_pdr_per_sf` | Pure ALOHA per SF | Separate G per SF |
| `analytical_pdr_with_capture` | ALOHA + simplified capture | P_capture = 0.5 |
| `analytical_ps1_per_sf` | Ps1 (LoRaWANSim Magrin 2017) | + P_coverage from simulation |
| `analytical_ps2_rx2` | Ps2 RX2 fallback | Single-channel fallback |
| `compare_ps1_ps2` | Full Ps1+Ps2 comparison | sim vs analytical |

---

### 5.2 Pure ALOHA Model

**Traffic load:**
```
G = n_devices × ToA / (period × n_channels)
```

Where:
- `n_devices`: number of transmitting devices
- `ToA`: time-on-air (s) — average across all SFs
- `period`: `1 / lambda_rate` (s) — mean inter-arrival time
- `n_channels`: number of available uplink channels (3 for EU868 default)

**Success probability:**
```
P_success = exp(-2G)
```

This is the classic Poisson-process collision probability: a packet succeeds if no other packet overlaps it during `[t - ToA, t + ToA]` (guard interval of one ToA before and after).

**Example (reference scenario):**
```
n = 50, ToA(mean) ≈ 0.5s, period = 10s, M = 3

G = 50 × 0.5 / (10 × 3) = 0.833
P_success = exp(-2 × 0.833) = exp(-1.667) ≈ 18.9%
```

This is a worst-case estimate — the simulation achieves ~90% PDR because:
1. Not all devices use SF12 (where ToA is high)
2. The SF orthogonality allows co-channel coexistence
3. Capture effect (SINR-based) rescues packets even with overlap

---

### 5.3 Per-SF ALOHA (`analytical_pdr_per_sf`)

Computes separate G for each SF, exploiting the fact that different SFs interfere with each other at attenuated thresholds:

```python
for sf in range(7, 13):
    n_sf = n_devices × sf_distribution[sf]
    toa = toa_per_sf[sf]
    G_sf = n_sf × toa / (period × n_channels)
    pdr[sf] = exp(-2 × G_sf)
```

This model treats cross-SF interference as zero (perfect orthogonality assumption). The real simulator uses the Semtech/Goursaud interference matrices, which allow cross-SF interference at attenuated levels.

---

### 5.4 `analytical_ps1_per_sf(network)` — LoRaWANSim Ps1 Model

Reference: Magrin et al., "Performance Evaluation of LoRa Networks in a Smart City Scenario," 2017.

**Ps1** is the probability that a packet is successfully received in the **RX1 window**:

```
Ps1(SF) = P_no_collision(SF) × P_coverage(SF)
```

**P_no_collision (ALOHA per SF):**
```
G_SF = (n_SF × ToA_SF) / (avg_period × M)
P_no_collision = exp(-2 × G_SF)
```

**P_coverage (from simulation state):**
```
P_coverage = covered_devices[SF] / n_devices[SF]
```
This is the fraction of SF-s devices that have `coverage_status=True` at the end of the simulation — computed by querying each device's final state after `find_best_gateway()`.

**Per-SF output dictionary:**
```python
{
    sf: {
        'n_devices': int,
        'toa_s': float,
        'G': float,
        'p_no_collision': float,
        'p_coverage': float,
        'Ps1': float,        # = p_no_collision × p_coverage
    }
}
```

---

### 5.5 `analytical_ps2_rx2(network)` — RX2 Fallback

**Ps2** is the probability of success in the **RX2 fallback window**, given that RX1 failed:

```
G_RX2 = λ_failed × ToA_SF12 / 1 channel
Ps2   = exp(-2 × G_RX2) × P_coverage_SF12
```

Where:
- `λ_failed = count(confirmed_packets that collided) / simulation_time` — rate of confirmed packets needing RX2
- `ToA_SF12` at 125 kHz BW, 13B payload (ACK frame) = 2.465 s

Note: RX2 uses a **single channel** (EU868: 869.525 MHz @ SF12), so `n_channels = 1`. High RX2 load can create significant contention.

---

### 5.6 `compare_ps1_ps2(network)` — Full Comparison

Computes the **effective analytical PDR** accounting for both RX1 and RX2 success paths:

```
For unconfirmed (70%): PDR = Ps1
For confirmed (30%):   PDR = Ps1 + (1 - Ps1) × Ps2  [RX2 gives second chance]

Total PDR = 0.7 × weighted_Ps1 + 0.3 × (weighted_Ps1 + (1 - weighted_Ps1) × Ps2)
```

**Weighted Ps1** averages across SFs:
```
weighted_Ps1 = Σ_sf [ Ps1(sf) × n_devices(sf) ] / total_devices
```

**Output:**
```python
{
    'ps1_per_sf': {...},               # per-SF Ps1 results
    'ps2': {...},                      # RX2 model results
    'weighted_ps1': float,
    'analytical_pdr': float,           # 0..1
    'simulation_pdr': float,           # 0..1
    'difference': float,               # |analytical - simulation|
    'within_15pct': bool,              # |diff| < 0.15 (15 percentage points)
    'per_sf_comparison': {
        sf: {
            'n_packets': int,
            'sim_pdr': float,
            'analytical_ps1': float,
            'difference': float,
        }
    }
}
```

---

## 6. `compute_metrics` — Full Metrics Pipeline

### 6.1 Function Signature

```python
def compute_metrics(network) -> dict
```

Computes the full set of simulation metrics after `simulate_transmissions()` has completed. Called after simulation in typical usage:

```python
from analytics import compute_metrics, export_json
metrics = compute_metrics(network)
export_json(metrics, "results.json")
```

### 6.2 Output Structure

```python
{
    "simulation": {
        "duration_s":       int,
        "num_devices":      int,
        "num_gateways":     int,
        "region":           str,   # "EU868", "US915", etc.
        "model_pathloss":   str,   # "log_normal_shadowing"
        "deployment_type":  str,   # "grid", "circular", etc.
        "seed":             int|None,
    },
    "performance": {
        "pdr_percent":              float,     # 0.0–100.0
        "pdr_per_sf":               {7..12: float},
        "pdr_vs_distance_km":       {"0": float, "1": float, ...},
        "total_packets":            int,
        "collisions":               int,
        "retransmissions":          int,
        "retransmission_rate_percent": float,
        "successful":               int,
        "avg_delay_ms":             float,     # ToA mean of successful packets
        "p50_delay_ms":             float,
        "p95_delay_ms":             float,
        "collision_rate_percent":   float,
    },
    "energy": {
        "total_network_mj":         float,
        "avg_per_device_mj":        float,
        "min_per_device_mj":        float,
        "max_per_device_mj":        float,
        "avg_per_sf_mj":            {7..12: float},
        "breakdown_percent":        {"TX": %, "RX": %, "SLEEP": %, "STANDBY": %},
        "avg_battery_lifetime_days": float|None,
    },
    "radio": {
        "sf_distribution":          {7..12: int},  # count of devices
        "avg_sinr_db":              float,
        "min_sinr_db":              float,
        "p5_sinr_db":               float,         # 5th percentile SINR
    },
    "adr": {
        "adjustments_count":        int,
        "sf_distribution_final":    {7..12: int},
    },
    "analytical": {
        "ps1_per_sf":               {...},    # analytical_ps1_per_sf()
        "ps2":                      {...},    # analytical_ps2_rx2()
    }
}
```

### 6.3 PDR per SF Computation

```python
for sf in range(7, 13):
    sf_packets = [p for p in all_packets if p.sf == sf]
    if sf_packets:
        success = sum(1 for p in sf_packets if not p.collided)
        pdr_per_sf[sf] = success / len(sf_packets) × 100
```

`all_packets = packets + retransmitted_packets` — retransmissions contribute to their respective SF's statistics.

### 6.4 PDR vs Distance

Packets are binned into 1 km buckets using Euclidean distance to the first gateway:

```python
dist_km = int(hypot(dev.x - gw.x, dev.y - gw.y) / 1000)
dist_buckets[dist_km]["sent"] += 1
if not p.collided:
    dist_buckets[dist_km]["success"] += 1
pdr_vs_distance[str(dist_km)] = success / sent × 100
```

Only the first gateway (`network.gateways[0]`) is used as reference. For multi-GW scenarios, this is the GW at grid position (0).

### 6.5 Delay Metric

"Delay" in this context is the **Time-on-Air (ToA)** of successfully received packets, expressed in milliseconds:

```python
delays = [p.rectime × 1000 for p in all_packets
          if not p.collided and p.rectime is not None]
```

This is not end-to-end latency (which would include RX window delays). It measures the transmission duration of packets that were not lost — a proxy for PHY-layer efficiency.

**Percentiles:**
```python
p50 = np.percentile(delays, 50)   # median ToA
p95 = np.percentile(delays, 95)   # 95th percentile
```

### 6.6 Energy Breakdown

```python
total_energy = sum(d.energy_model.energy_consumed for d in network.devices)
for state in RadioState:    # SLEEP, STANDBY, RX, TX
    total_by_state = sum(d.energy_model.energy_breakdown[state] for d in network.devices)
    breakdown_pct[state.name] = total_by_state / total_energy × 100
```

`energy_breakdown` is the dict maintained by `EnergyModel.transition()` for each device.

### 6.7 Battery Lifetime Estimation

```python
consumed_pct = 100.0 - battery.soc_percent()
rate_pct_per_s = consumed_pct / simulation_time
lifetime_s = 100.0 / rate_pct_per_s        # time to drain from 100%
lifetime_days = lifetime_s / 86400
```

This assumes linear battery consumption throughout the simulation — valid if traffic rate is constant. Averaged across all devices that have a `BatteryModel` attached.

### 6.8 SINR Statistics

```python
sinr_values = [p.sinr for p in all_packets if p.sinr is not None]
avg_sinr = np.mean(sinr_values)
min_sinr  = np.min(sinr_values)
p5_sinr   = np.percentile(sinr_values, 5)   # worst 5% — link robustness indicator
```

`p5_sinr` is useful for characterizing edge-case performance: if P5 SINR > SNR threshold for the device's SF, then even the weakest links are statistically robust.

---

## 7. Data Export

### 7.1 `export_json(metrics, filename="results.json")`

Serializes the `compute_metrics()` dict to indented JSON:

```python
json.dump(metrics, f, indent=2, default=str)
```

`default=str` handles non-serializable types (numpy floats, RadioState enum instances) by converting them to strings. Output file is ~2–5 KB for the reference scenario.

---

### 7.2 `export_npz(network, filename="results.npz")`

Binary compressed numpy format — equivalent to HDF5 without `h5py` dependency.

**15 arrays saved:**

| Array name | dtype | Shape | Content |
|-----------|-------|-------|---------|
| `packet_time` | float64 | (N,) | TX start time (s) |
| `packet_device_id` | int32 | (N,) | Source device ID |
| `packet_sf` | int32 | (N,) | Spreading Factor |
| `packet_rssi` | float64 | (N,) | RSSI at GW (dBm), NaN if missing |
| `packet_snr` | float64 | (N,) | SNR (dB), NaN if missing |
| `packet_sinr` | float64 | (N,) | SINR (dB), NaN if missing |
| `packet_collided` | int8 | (N,) | 0/1 collision flag |
| `packet_freq` | float64 | (N,) | Carrier frequency (MHz) |
| `packet_airtime_ms` | float64 | (N,) | ToA (ms), NaN if missing |
| `device_id` | int32 | (M,) | Device IDs |
| `device_sf` | int32 | (M,) | Final SF per device |
| `device_x` | float64 | (M,) | X position (m) |
| `device_y` | float64 | (M,) | Y position (m) |
| `device_energy_mj` | float64 | (M,) | Total energy consumed (mJ) |
| `device_pdr_percent` | float64 | (M,) | Per-device PDR (%) |

N = total packets (originals + retransmissions), M = number of devices.

Packets are sorted by `arrival_time` before extraction.

**Loading:**
```python
data = np.load("results.npz")
collided = data["packet_collided"]
pdr = (1 - collided.mean()) × 100
sinr = data["packet_sinr"][~np.isnan(data["packet_sinr"])]
```

**Compatibility:** Can be loaded in MATLAB via `scipy.io.loadmat` workaround (not native).

---

### 7.3 `export_csv_detailed(network, filename)`

Per-packet CSV with 15 columns:

```
time, device_id, sf, tx_power, freq, bw, rssi, snr, sinr,
collided, confirmed, ack_received, is_retransmission, frame_counter, airtime_ms
```

Sorted by `arrival_time`. RSSI/SNR/SINR fields show "N/A" if not computed (e.g., packet rejected before GW processing).

---

### 7.4 `export_device_summary(network, filename)`

Per-device CSV with 12 columns:

```
device_id, sf, tx_power, freq, class, x, y, coverage,
energy_mj, packets_sent, packets_collided, pdr_percent
```

Calls `get_device_stats(device_id)` for each device to obtain collision and PDR counts.

---

### 7.5 `PacketTracker.export_device_log(network, filename)`

Alternative per-packet CSV from `packettracker.py`, with 11 columns:

```
Tempo, Device ID, SF, TX Power, RSSI, SNR, Freq, BW, Collided, Confirmed, Frame Counter
```

Note: Does not include SINR, `ack_received`, or `is_retransmission` fields (unlike `export_csv_detailed`).

---

### 7.6 Export Format Comparison

| Format | Function | Size (50 dev, 1h) | Contains SINR | Per-device | Binary |
|--------|----------|------------------|--------------|-----------|--------|
| JSON | `export_json` | ~3 KB | Aggregated | No | No |
| NPZ | `export_npz` | ~50 KB | Per-packet | Yes | Yes (compressed) |
| CSV detailed | `export_csv_detailed` | ~500 KB | Per-packet | No | No |
| CSV device | `export_device_summary` | ~2 KB | No | Yes | No |
| CSV log | `export_device_log` | ~400 KB | No | No | No |

---

## 8. Visualization — `plots.py`

### 8.1 `plot_all(network, prefix="")`

Generates all 9 plots in a single call:

```python
from plots import plot_all
plot_all(network, prefix="run1_")
# Creates: run1_plot_sf_distribution.png, run1_plot_pdr_per_sf.png, ...
```

All plots use `matplotlib.use('Agg')` (non-interactive backend) — safe for headless servers.

---

### 8.2 `plot_sf_distribution` — SF Distribution

**Type:** Bar chart
**Data source:** `Counter(d.sf for d in network.devices)`
**X-axis:** SF7–SF12
**Y-axis:** Number of devices
**Annotation:** Count displayed above each bar
**Title:** Includes region name and deployment type

Useful for verifying ADR convergence: after a long simulation with ADR enabled, devices should cluster on lower SFs (SF7/SF8) if they have good coverage.

---

### 8.3 `plot_pdr_per_sf` — PDR by SF

**Type:** Bar chart
**Data source:** `packets + retransmitted_packets` grouped by `p.sf`
**X-axis:** SF7–SF12
**Y-axis:** PDR (%), Y-limit 0–110
**Annotation:** `n={count}` above each bar

Expected pattern: PDR decreases for higher SFs due to longer ToA → more collision probability. With ADR disabled (uniform SF distribution), higher SFs have lower PDR.

---

### 8.4 `plot_pdr_vs_distance` — PDR vs Distance

**Type:** Line + fill chart
**Data source:** All packets, binned to 1 km intervals from `network.gateways[0]`
**X-axis:** Distance buckets ("0-1km", "1-2km", ...)
**Y-axis:** PDR (%), Y-limit 0–110
**Annotation:** `n={count}` above each point

Expected pattern: PDR ≈ 100% for 0–2 km, then drops sharply at coverage edge. For `log_normal_shadowing` model, the drop is gradual with variance; for `fspl`, it is sharper.

---

### 8.5 `plot_energy_per_sf` — Energy by SF

**Type:** Bar chart with error bars
**Data source:** `d.energy_model.energy_consumed` per device, grouped by SF
**X-axis:** SF7–SF12
**Y-axis:** Mean energy (mJ)
**Error bars:** ±1 standard deviation (std)

Expected pattern: Energy increases significantly with SF. SF12 devices consume ~49× more per TX than SF7 (due to ToA scaling as 2^SF). However, SF12 devices that are duty-cycle limited transmit less frequently, partially offsetting this.

---

### 8.6 `plot_delay_cdf` — Delay CDF

**Type:** CDF line
**Data source:** `p.rectime × 1000` for all non-collided packets
**X-axis:** ToA (ms)
**Y-axis:** CDF (%)
**Percentile markers:** Vertical dashed lines at P50, P90, P95

"Delay" here is ToA, not network latency. The multi-modal CDF reflects the discrete SF distribution: each SF corresponds to a distinct ToA value. A step pattern indicates few SFs are used; a smooth curve indicates broad SF distribution.

---

### 8.7 `plot_sinr_distribution` — SINR Histogram

**Type:** Histogram (30 bins)
**Data source:** `p.sinr` for all packets with SINR computed
**X-axis:** SINR (dB)
**Markers:** Red dashed = mean; Orange dotted = P5

The SINR distribution is right-skewed: most packets have high SINR (low interference), with a tail of low-SINR packets near the gateway sensitivity threshold. Packets with SINR < threshold are marked as `collided=True`.

---

### 8.8 `plot_collision_heatmap` — Spatial Collision Heatmap

**Type:** 2D histogram (20×20 bins)
**Data source:** `(dev.x, dev.y)` for all collided packets
**Colormap:** `hot_r` (white=low, black/red=high)
**Overlay:** Gateway positions (blue star markers)

High collision density near the gateway indicates co-channel congestion (many devices with similar RSSI competing). High density at range edges indicates SNR-limited packets close to the sensitivity floor.

---

### 8.9 `plot_throughput_over_time` — Time-Series Throughput

**Type:** Step chart + fill
**Data source:** Successful packets binned into 60s windows
**X-axis:** Simulation time (s)
**Y-axis:** Packets per 60s window

A flat profile indicates Poisson arrivals (expected). Drops indicate duty cycle blocks (after SF12 devices exhaust their time budget). Spikes can indicate retransmission bursts after a high-interference period.

---

### 8.10 `plot_battery_soc_timeline` — Battery SoC Distribution

**Type:** Histogram (20 bins, range 0–100%)
**Data source:** `d.battery.soc_percent()` for devices with BatteryModel
**Only generated** if any device has `battery is not None`
**Marker:** Red dashed = mean SoC

A narrow, high-SoC distribution (e.g., 95–100%) indicates low energy consumption relative to capacity — long battery lifetime expected. Devices below 50% SoC after 1h simulation may deplete within a few hours.

---

## 9. Collision Detection (Legacy)

### 9.1 `detect_collisions_and_interference()` in `network.py`

This is a **post-simulation** collision detection algorithm, retained for comparison and debugging. The primary collision detection during simulation happens inside `gateway.py:process_uplink()` (energy-based, real-time).

```python
def detect_collisions_and_interference(self):
    packets = sorted(self.packet_tracker.packets, key=lambda p: p.arrival_time)
    for i, p1 in enumerate(packets):
        for j in range(i + 1, len(packets)):
            p2 = packets[j]
            if p2.arrival_time >= p1.arrival_time + p1.rectime:
                break                          # early exit: no more overlaps
            if p1.device_id == p2.device_id:
                continue
            if p1.freq != p2.freq:
                continue                       # different channel: no interference
            # Compute overlap
            overlap_start = max(p1.arrival_time, p2.arrival_time)
            overlap_end   = min(p1.tx_end, p2.tx_end)
            if overlap_end <= overlap_start:
                continue
            # Apply interference matrix
            threshold = interference_matrix[12-p1.sf][12-p2.sf]
            if p1.rssi - p2.rssi < threshold:
                p1.collided = True
            threshold2 = interference_matrix[12-p2.sf][12-p1.sf]
            if p2.rssi - p1.rssi < threshold2:
                p2.collided = True
```

**Complexity:** O(N²) worst case, O(N × k) average case where k = packets overlapping at time t. The early-break optimization (`break` when `p2.arrival_time >= p1.tx_end`) is effective because packets are sorted by arrival time.

**Difference from real-time detection:**
- Does not implement preamble locking (G13)
- Does not use energy-based overlap weighting (G14)
- Does not account for gateway reception paths (SX1301 8 paths)
- Used as a validation tool, not the primary interference model

---

## 10. Data Flow Diagram

```
Simulation Run
│
├─ For each TX event (DEVICE_SEND):
│  └─ Packet created → PacketTracker.add_packet(packet)
│                       │
│                       ├─ packets[] or retransmitted_packets[]
│                       └─ packet_history[device_id][]
│
├─ For each TX_END event:
│  └─ channel.evaluate_reception(packet, gw)
│     └─ packet.collided = True/False
│
└─ Post-simulation:
   │
   ├─ PacketTracker
   │  ├─ get_stats()         → PDR, collisions, retransmissions
   │  ├─ get_device_stats()  → per-device breakdown
   │  └─ export_device_log() → CSV
   │
   ├─ analytics.compute_metrics(network)
   │  ├─ PDR per SF
   │  ├─ PDR vs distance (1km buckets)
   │  ├─ Energy breakdown
   │  ├─ SINR statistics
   │  ├─ Battery lifetime estimate
   │  └─ Analytical Ps1/Ps2 comparison
   │
   ├─ analytics.export_json(metrics)     → results.json
   ├─ analytics.export_npz(network)      → results.npz (15 arrays)
   ├─ analytics.export_csv_detailed()    → results_detailed.csv
   ├─ analytics.export_device_summary()  → device_summary.csv
   │
   └─ plots.plot_all(network, prefix="")
      ├─ plot_sf_distribution.png
      ├─ plot_pdr_per_sf.png
      ├─ plot_pdr_vs_distance.png
      ├─ plot_energy_per_sf.png
      ├─ plot_delay_cdf.png
      ├─ plot_sinr_distribution.png
      ├─ plot_collision_heatmap.png
      ├─ plot_throughput_over_time.png
      └─ plot_battery_soc.png
```

---

## 11. Numerical Examples

### Example 1: PDR Computation

**Scenario:** 18,000 TX attempts (50 devices × 360 TX/h), 1,620 collisions, 180 retransmissions (of which 40 also collide).

```
packets:                18,000 - 180 retransmissions = 17,820 originals
retransmitted_packets:  180

total_packets = 17,820 + 180 = 18,000
collided:       1,620 (across both lists)
successful:     18,000 - 1,620 = 16,380

PDR = 16,380 / 18,000 × 100 = 91.0%

unique_packet_count = 17,820
total_retransmissions = 180
```

### Example 2: Analytical Model vs Simulation

**Reference scenario:** 50 devices, λ=0.1 Hz, SF distribution uniform [7..12], 3 channels EU868.

```
ToA values: SF7=56.6ms, SF8=102ms, SF9=185ms, SF10=329ms, SF11=659ms, SF12=1319ms
Average ToA: (56.6+102+185+329+659+1319)/6 = 441.8 ms = 0.4418 s

G (aggregate ALOHA):
G = 50 × 0.4418 / (10 × 3) = 0.736
P_success = exp(-2 × 0.736) = exp(-1.47) = 22.9%

Per-SF ALOHA (8 devices per SF, period=10s):
  SF7:  G = 8 × 0.0566 / (10 × 3) = 0.0151 → PDR = 97.0%
  SF8:  G = 8 × 0.102 / 30 = 0.0272 → PDR = 94.7%
  SF9:  G = 8 × 0.185 / 30 = 0.0493 → PDR = 90.6%
  SF10: G = 8 × 0.329 / 30 = 0.0877 → PDR = 83.9%
  SF11: G = 8 × 0.659 / 30 = 0.176  → PDR = 70.4%
  SF12: G = 8 × 1.319 / 30 = 0.352  → PDR = 49.7%

Weighted Ps1 (uniform SF): (97.0+94.7+90.6+83.9+70.4+49.7)/6 = 81.1%
Analytical total PDR ≈ 81.1%

Simulation PDR ≈ 90% (higher due to capture effect + SINR matrix)
Difference = |0.811 - 0.900| = 0.089 < 0.15 → within_15pct = True
```

### Example 3: NPZ Loading and Analysis

```python
import numpy as np

data = np.load("results.npz")

# Basic PDR
collided = data["packet_collided"]
pdr = (1 - collided.mean()) * 100
print(f"PDR: {pdr:.1f}%")

# SINR distribution (excluding NaN)
sinr = data["packet_sinr"]
sinr_valid = sinr[~np.isnan(sinr)]
print(f"SINR: mean={sinr_valid.mean():.1f} dB, P5={np.percentile(sinr_valid,5):.1f} dB")

# Per-SF collision rate
sf = data["packet_sf"]
for s in range(7, 13):
    mask = sf == s
    if mask.sum() > 0:
        cr = collided[mask].mean() * 100
        print(f"  SF{s}: {mask.sum()} packets, {cr:.1f}% collision rate")

# Energy per device
energy = data["device_energy_mj"]
print(f"Energy: mean={energy.mean():.1f} mJ, max={energy.max():.1f} mJ")
```

**Expected output (reference scenario):**
```
PDR: 91.0%
SINR: mean=14.2 dB, P5=1.8 dB
  SF7: 2800 packets, 3.1% collision rate
  SF8: 2900 packets, 5.2% collision rate
  SF9: 3100 packets, 8.7% collision rate
  SF10: 3300 packets, 13.1% collision rate
  SF11: 3000 packets, 23.5% collision rate
  SF12: 2900 packets, 50.4% collision rate
Energy: mean=966.3 mJ, max=3841.2 mJ
```

### Example 4: Battery Lifetime Estimation

**Scenario:** 50 devices, 2400 mAh battery, 3.3V, SF9, λ=0.1 Hz, T=3600s.

```
Energy per device (SF9, 1h) ≈ 1200 mJ (from energy model)

Battery capacity: 2400 mAh × 3.3V × 3600 = 28,512 J = 28,512,000 mJ

SoC consumed in 1h = 1200 / 28512000 × 100 = 0.0042%
Rate = 0.0042% / 3600 s = 1.17×10⁻⁶ %/s
Lifetime = 100% / (1.17×10⁻⁶ %/s) = 85,470,085 s ≈ 989 days

→ Battery lifetime ≈ 2.7 years for SF9 at λ=0.1 Hz
```

### Example 5: `compare_ps1_ps2` Interpretation

```python
result = compare_ps1_ps2(network)
# Expected output (reference scenario):
{
    'weighted_ps1': 0.812,
    'analytical_pdr': 0.831,   # incl. RX2 for 30% confirmed
    'simulation_pdr': 0.910,
    'difference': 0.079,
    'within_15pct': True,
    'per_sf_comparison': {
        7:  {'n_packets': 2800, 'sim_pdr': 0.969, 'analytical_ps1': 0.970, 'difference': 0.001},
        8:  {'n_packets': 2900, 'sim_pdr': 0.948, 'analytical_ps1': 0.947, 'difference': 0.001},
        9:  {'n_packets': 3100, 'sim_pdr': 0.913, 'analytical_ps1': 0.906, 'difference': 0.007},
        10: {'n_packets': 3300, 'sim_pdr': 0.874, 'analytical_ps1': 0.839, 'difference': 0.035},
        11: {'n_packets': 3000, 'sim_pdr': 0.794, 'analytical_ps1': 0.704, 'difference': 0.090},
        12: {'n_packets': 2900, 'sim_pdr': 0.694, 'analytical_ps1': 0.497, 'difference': 0.197},
    }
}
```

**Interpretation:**
- SF7–SF9: simulation and analytical agree closely (capture effect negligible at low load)
- SF10–SF11: simulation outperforms analytical by ~5–9 pp — capture effect rescues some collisions
- SF12: simulation outperforms by ~20 pp — high ToA means many overlaps, but capture effect and preamble locking (G13) rescue significant fraction

---

## 12. Comparison with ns-3 and FLoRa

| Feature | PyLoRaWAN | ns-3 LoRaWAN | FLoRa |
|---------|----------|-------------|-------|
| **PDR formula** | (success / total) × 100 | Same | Same |
| **Per-SF PDR** | Yes (`compute_metrics`) | Yes (Traci/Pcap) | Yes (built-in) |
| **PDR vs distance** | 1km buckets | Continuous (Pcap) | Built-in |
| **SINR tracking** | Per-packet `p.sinr` | Per-packet (ns-3 log) | Not tracked |
| **Energy breakdown** | TX/RX/SLEEP/STANDBY % | LoraRadioEnergyModel | Not modeled |
| **Battery lifetime** | Extrapolation from 1h | Not built-in | Not built-in |
| **Analytical validation** | ALOHA + Ps1/Ps2 | External | Ps1 (Magrin 2017) |
| **Export formats** | JSON/NPZ/CSV×3 | PCAP/Traci/CSV | CSV |
| **Binary export** | NPZ (numpy) | PCAP | No |
| **Visualization** | 9 matplotlib plots | ns3-netanim, gnuplot | MATLAB/Python scripts |
| **Collision heatmap** | Yes (`plot_collision_heatmap`) | No | No |
| **Throughput timeline** | Yes (60s windows) | Yes (FlowMonitor) | Yes |
| **Retransmission tracking** | Separate list + flag | FCnt retransmits | Included |
| **MIC validation flag** | `p.mic_valid` bool | Full validation | Not implemented |

---

## Summary

The metrics and data collection subsystem provides:

1. **`Packet` object** with 30+ fields capturing all PHY/MAC/security attributes of every transmission.

2. **`PacketTracker`** with dual lists (originals/retransmissions), per-device history, and PDR computation across both.

3. **`NetworkThroughput`** for ALOHA-based traffic load analysis and throughput estimation.

4. **Analytical models** at three levels: aggregate ALOHA, per-SF ALOHA, and LoRaWANSim Ps1/Ps2 (Magrin 2017) including RX2 fallback and coverage probability.

5. **`compute_metrics()`** producing a hierarchical dict with 30+ KPIs covering performance, energy, radio, ADR, and analytical benchmarks.

6. **Five export formats:** JSON (summary), NPZ (15 binary arrays), CSV per-packet, CSV per-device, CSV log.

7. **Nine matplotlib plots** covering all key result dimensions, callable via `plot_all(network, prefix)`.
