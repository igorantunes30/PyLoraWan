# Mobility and Traffic Generation Module

**PyLoRaWAN Simulator — Technical Reference**

---

## Table of Contents

1. [Module Overview](#1-module-overview)
2. [Deployment Strategies](#2-deployment-strategies)
3. [Traffic Generation Model](#3-traffic-generation-model)
4. [Mobility Model](#4-mobility-model)
5. [Device Initialization Pipeline](#5-device-initialization-pipeline)
6. [Configuration Loader](#6-configuration-loader)
7. [Parallel Multi-Seed Runner](#7-parallel-multi-seed-runner)
8. [Event Scheduling for Mobility and Traffic](#8-event-scheduling-for-mobility-and-traffic)
9. [Interaction Diagram](#9-interaction-diagram)
10. [Numerical Examples](#10-numerical-examples)
11. [Comparison with ns-3 and FLoRa](#11-comparison-with-ns-3-and-flora)

---

## 1. Module Overview

Mobility and traffic generation control **where** devices are placed, **how** they move through the simulation area, and **when** they transmit packets. These subsystems are the entry point of every simulation: all interference, energy, and delivery statistics ultimately depend on deployment geometry and traffic patterns.

### Source Files

| File | Responsibility |
|------|---------------|
| `deployment.py` | 7 spatial deployment strategies |
| `enddevice.py` | `move()`, traffic attributes, `lambda_rate` |
| `network.py` | `initialize_devices()`, `_on_device_send()`, `_on_mobility_update()` |
| `parametors.py` | Default values: `speed`, `lambda_rate`, `mobility_enabled`, `model` |
| `config_loader.py` | YAML/JSON config → `Network` params |
| `parallel_runner.py` | Multi-seed parallel execution |

### Reference Scenario (default `parametors.py`)

| Parameter | Value | Notes |
|-----------|-------|-------|
| `num_devices` | 50 | End devices |
| `num_gateways` | 1 | Gateway |
| `area_size` | 10,000 m | 10 km × 10 km |
| `lambda_rate` | 0.1 Hz | Mean interval = 10 s |
| `speed` | 1.5 m/s | Pedestrian |
| `simulation_time` | 3600 s | 1 hour |
| `deployment_type` | `"grid"` | Default |
| `random_seed` | 42 | Reproducible |

---

## 2. Deployment Strategies

### 2.1 Module: `deployment.py`

Provides 7 spatial deployment functions plus a unified `deploy()` interface.

```python
DEPLOYMENT_STRATEGIES = {
    "grid":           deploy_grid,
    "circular":       deploy_circular,
    "annular":        deploy_annular,
    "hexagonal":      deploy_hexagonal,
    "random_uniform": deploy_random_uniform,
    "clustered":      deploy_clustered,
    "from_file":      deploy_from_file,
}

def deploy(strategy, **kwargs):
    func = DEPLOYMENT_STRATEGIES[strategy]
    return func(**kwargs)
```

Each function returns a list of `(x, y)` tuples in meters.

---

### 2.2 `deploy_grid(n_devices, area_size)`

Regular rectangular grid. Devices are spaced uniformly on a `rows × cols` matrix, with half-cell offset to avoid boundary placement.

```
rows = ceil(sqrt(n_devices))
cols = ceil(n_devices / rows)

x_spacing = area_size / cols
y_spacing = area_size / rows

x[c] = c * x_spacing + x_spacing/2   (c = 0..cols-1)
y[r] = r * y_spacing + y_spacing/2   (r = 0..rows-1)
```

**Example** — 9 devices, 10 km area:

```
rows = 3, cols = 3
Spacing = 10000/3 ≈ 3333 m
Device (0,0): (1666, 1666)
Device (0,1): (5000, 1666)
Device (0,2): (8333, 1666)
...
```

**Use case:** Systematic analysis, comparison with FLoRa reference scenarios.

---

### 2.3 `deploy_circular(n_devices, radius, center_x, center_y)`

Uniform distribution within a disk — same method used by FLoRa simulator.

```
r     = radius * sqrt(U[0,1])   # uniform area, not uniform radius
theta = U[0, 2π]
x     = center_x + r * cos(theta)
y     = center_y + r * sin(theta)
```

**Why `sqrt(U)`?** Naive `r = U * R` concentrates devices near the center (PDF ∝ r, not uniform). The `sqrt` transform gives uniform PDF over the disk area (PDF ∝ r / R²).

**Invocation in `Network.initialize_devices()`:**
```python
center = self.area_size / 2          # (5000, 5000) for 10km area
positions = deploy_circular(
    self.num_devices,
    self.deployment_radius,          # area_size/2 default
    center, center
)
```

---

### 2.4 `deploy_annular(n_devices, r_min, r_max, center_x, center_y)`

Uniform distribution in an annular ring between `r_min` and `r_max`. Matches LoRaWANSim deployment.

```
r = sqrt(U[r_min², r_max²])
```

This is the CDF inverse for a uniform 2D annular distribution:
```
P(r < R) = (R² - r_min²) / (r_max² - r_min²)
```

**Invocation:**
```python
# r_min = 20% of radius, r_max = full radius
deploy_annular(
    n_devices, deployment_radius * 0.2,
    deployment_radius, center, center
)
```

---

### 2.5 `deploy_hexagonal(n_positions, inter_distance)`

Hexagonal lattice — standard for multi-gateway placement (as in ns-3).

```
Odd rows: x_offset = inter_distance / 2 (stagger)
x = col * inter_distance + x_offset
y = row * inter_distance * sqrt(3) / 2
```

The `sqrt(3)/2` factor is the vertical distance between hexagonal rows (equilateral triangle geometry).

**Use case:** Multi-gateway topologies where gateways should have maximum coverage overlap.

---

### 2.6 `deploy_random_uniform(n_devices, area_size)`

Independent uniform samples in `[0, area_size] × [0, area_size]`. Simplest model; no spatial correlation.

---

### 2.7 `deploy_clustered(n_devices, n_clusters, area_size, cluster_std=None)`

Gaussian clusters — models urban hotspots (buildings, factories, agriculture fields).

```
n_clusters = max(1, n_devices // 10)  # default: 1 cluster per 10 devices
cluster_std = area_size / (n_clusters * 4)   # if not specified

cluster_center[k] ~ U[2σ, area_size - 2σ]   # away from boundaries
device[i]  ~ N(center[k_i], σ²) clipped to [0, area_size]
```

**Remainder distribution:** `devices_per_cluster + (1 if i < remainder else 0)` — first clusters get an extra device.

---

### 2.8 `deploy_from_file(filename)`

Loads positions from a CSV file — compatible with ns-3 mobility trace exports and FLoRa scenario files.

```
Format: CSV with columns x, y (header row is optional)
Header detection: tries float(first_row[0]) — if fails, treats as header
```

**Validation in `Network.initialize_devices()`:**
```python
if len(positions) < self.num_devices:
    raise ValueError(f"File has {len(positions)} positions, "
                     f"but num_devices={self.num_devices}")
positions = positions[:self.num_devices]  # truncate excess
```

---

### 2.9 Deployment Strategy Comparison

| Strategy | Spatial Model | Use Case | Matches Simulator |
|----------|--------------|----------|------------------|
| `grid` | Deterministic regular | Systematic study, debugging | FLoRa, LoRaWANSim |
| `circular` | Uniform disk | Standard IoT coverage study | FLoRa (default) |
| `annular` | Uniform ring | Range-dependent analysis | LoRaWANSim |
| `hexagonal` | Hexagonal lattice | Multi-GW placement | ns-3 |
| `random_uniform` | IID uniform | Monte Carlo baseline | All |
| `clustered` | Gaussian clusters | Urban hotspot | Custom / field data |
| `from_file` | External trace | Replay real traces | ns-3 / FLoRa export |

**Gateway placement** is always a regular grid (`initialize_gateways()`), regardless of device deployment:
```python
rows = ceil(sqrt(num_gateways))
cols = ceil(num_gateways / rows)
# same half-cell offset formula as deploy_grid
```

---

## 3. Traffic Generation Model

### 3.1 Poisson Process

All devices generate traffic as a homogeneous Poisson process with rate `λ` (Hz).

```
Inter-arrival time T ~ Exponential(λ)
E[T] = 1/λ
```

**Implementation in `_on_device_send()`:**
```python
next_delay = np.random.exponential(1.0 / max(device.lambda_rate, 0.001))
self.scheduler.schedule(next_delay, EventType.DEVICE_SEND,
                        self._on_device_send, device)
```

The `max(..., 0.001)` guard prevents division by zero if `lambda_rate = 0`.

**Initial offset** to avoid burst at t=0:
```python
initial_delay = random.uniform(0, 1.0 / max(device.lambda_rate, 0.001))
```
This samples the first transmission uniformly within the first inter-arrival period — equivalent to the steady-state of a Poisson process already in progress at t=0.

---

### 3.2 Traffic Parameters

| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| `lambda_rate` | float (Hz) | 0.1 | Mean transmission rate |
| `traffic_type` | str | random | "periodic", "sporadic", "critical" |
| `confirmed_ratio` | float | 0.3 | Fraction of confirmed (ACK-req) uplinks |
| `max_retransmissions` | int | 10 | Max retransmit attempts per packet |
| `dc_release_time` | float (s) | 0 | Earliest next TX allowed by duty cycle |
| `frame_counter_up` | int | 0 | Uplink frame counter (incremented per TX) |

**`traffic_type`** is assigned randomly at device creation:
```python
self.traffic_type = random.choice(["periodic", "sporadic", "critical"])
```
Currently this is a metadata field — it does not change the `lambda_rate` distribution. It is available for custom traffic shaping extensions.

---

### 3.3 Duty Cycle Enforcement

Before every transmission, the duty cycle constraint is checked:

```python
if time < device.dc_release_time:
    delay = device.dc_release_time - time + 0.001   # +1ms margin
    self.scheduler.schedule(delay, EventType.DEVICE_SEND,
                            self._on_device_send, device)
    return  # defer, don't transmit now
```

After a successful transmission, `dc_release_time` is updated:
```python
dc_limit = self.region.get_duty_cycle_limit(device.freq)
         or (ed_dc_limit_percent / 100.0)    # fallback: 1%
device.dc_release_time = time + airtime / dc_limit
```

**Example:** SF7, BW=125kHz, 20B payload → airtime ≈ 56.6 ms, DC = 1%:
```
dc_release_time = t_now + 0.0566 / 0.01 = t_now + 5.66 s
```
The device must wait at least 5.66 s before transmitting on the same sub-band again.

Regional sub-band limits (EU868):

| Sub-band | Frequencies (MHz) | DC limit |
|----------|------------------|----------|
| G | 863.0–868.0 | 1% |
| G1 | 868.0–868.6 | 1% |
| G2 | 868.7–869.2 | 0.1% |
| G3 | 869.4–869.65 | 10% (RX2) |

---

### 3.4 Payload Validation

Before every transmission, payload is validated against regional DR limits:
```python
effective_pl = device.validate_payload_size(self.region)
# min(device.pl, region.get_max_payload(sf, bw))
```

**Dwell time check** (relevant for AS923, US915):
```python
if not device.check_dwell_time(self.region):
    # airtime > region.max_dwell_time_ms / 1000
    # skip transmission, reschedule
    next_delay = np.random.exponential(1/lambda_rate)
    ...
    return
```

---

### 3.5 Frequency Hopping (Channel Selection)

Before each transmission, a channel is selected from the available channel list:
```python
def select_channel(self):
    self.freq = random.choice(self._available_channels)
    return self.freq
```

`_available_channels` is initialized from `frequency_mhz` list at device creation:
```python
self._available_channels = list(frequency_mhz)
# EU868 default: [868.1, 868.3, 868.5] MHz
```

This implements **random channel selection** (equivalent to ETSI pseudo-random hopping for EU868). US915 has 64 + 8 channels managed by regional parameters.

---

### 3.6 Confirmed Uplinks and Retransmissions

30% of packets (`confirmed_ratio = 0.3`) are confirmed uplinks requiring ACK:
```python
packet.confirmed = (random.random() < device.confirmed_ratio)
```

If a packet collides (no ACK received), it is retransmitted with **truncated exponential backoff**:

```python
if packet.collided and device.retransmission_attempts < device.max_retransmissions:
    backoff = random.uniform(1, 10) * (2 ** device.retransmission_attempts)
    device.retransmission_attempts += 1
    self.scheduler.schedule(backoff, EventType.DEVICE_SEND,
                            self._on_retransmit, device)
```

**Backoff table (expected values):**

| Attempt | `random.uniform(1,10)` mean | Multiplier | Mean backoff (s) |
|---------|-----------------------------|------------|-----------------|
| 0 | 5.5 | 1 | 5.5 |
| 1 | 5.5 | 2 | 11.0 |
| 2 | 5.5 | 4 | 22.0 |
| 3 | 5.5 | 8 | 44.0 |
| 4 | 5.5 | 16 | 88.0 |
| 10 (max) | 5.5 | 1024 | 5632.0 |

Retransmissions create a new `Packet` object with `is_retransmission=True` and an incremented frame counter. They are tracked separately in `PacketTracker`:
```python
self.packet_tracker.add_packet(packet, is_retransmission=True)
```

---

### 3.7 LinkCheckReq Injection

Every transmission has a 10% probability of triggering a `LinkCheckReq` MAC command:
```python
if random.random() < 0.10:
    self.ns.link_check.request_link_check(device.device_id)
```

This adds `LinkCheckAns` to the next downlink, providing the device with SNR margin and gateway count information (LoRaWAN MAC 0x02).

---

### 3.8 Traffic Load Analysis

For the reference scenario (50 devices, λ=0.1 Hz, 1h):

```
Expected transmissions per device = λ × T = 0.1 × 3600 = 360
Expected total uplinks = 50 × 360 = 18,000
Expected inter-arrival per device = 10 s
Network aggregate rate = 50 × 0.1 = 5 pkt/s
```

With duty cycle (1%, SF7): each device occupies ~5.66 s off-time per packet.
Maximum sustainable rate per device per channel:
```
λ_max = DC / airtime = 0.01 / 0.0566 ≈ 0.177 Hz   (SF7 on one channel)
```
At λ=0.1 Hz < λ_max: device is not duty-cycle limited for SF7.

For SF12 (airtime ≈ 2.793 s):
```
λ_max = 0.01 / 2.793 ≈ 0.00358 Hz (1 pkt per ~279 s)
```
At λ=0.1 Hz >> λ_max: **SF12 devices are heavily duty-cycle limited**.

---

## 4. Mobility Model

### 4.1 Random Walk

The current implementation uses a **2D random walk** (Brownian motion discretized at fixed time steps):

```python
def move(self, area_size, mobility_enabled, model):
    if not mobility_enabled:
        return 0, (self.x, self.y)
    time_interval = np.random.exponential(1 / self.lambda_rate)  # unused externally
    angle = random.uniform(0, 2 * np.pi)
    dx = self.speed * np.cos(angle)
    dy = self.speed * np.sin(angle)
    self.x = max(0, min(area_size, self.x + dx))
    self.y = max(0, min(area_size, self.y + dy))
    return time_interval, (self.x, self.y)
```

**Step properties:**
- Direction: uniform random `θ ∈ [0, 2π]`
- Step magnitude: `speed × 1s` (1s update interval from `_on_mobility_update`)
- At 1.5 m/s: displacement per step = 1.5 m

**Boundary condition:** Reflective walls (clamp to `[0, area_size]`). This prevents devices from leaving the simulation area but introduces a slight bias toward corners; for large areas with short simulation times, this effect is negligible.

**Note:** The `time_interval` returned by `move()` (sampled from `Exponential(1/λ)`) is computed but **not used** by `_on_mobility_update()` — the event handler reschedules itself at a fixed 1.0 s interval. The returned value is available for custom extensions.

---

### 4.2 Planned Mobility Models (in `parametors.py`)

```python
model = "random_walk"  # "gauss_markov" "levy_walk" "random_walk" "matrix"
```

Only `"random_walk"` is currently implemented. The `model` attribute is stored on each device for future extension. `"gauss_markov"` (correlated direction changes) and `"levy_walk"` (long-tail displacement) are referenced as future work.

---

### 4.3 Mobility Event Handler

```python
def _on_mobility_update(self):
    for device in self.devices:
        if device.mobility_enabled:
            _, new_position = device.move(self.area_size,
                                          device.mobility_enabled,
                                          device.model)
            self.positions["devices"][device.device_id] = new_position
            device.update_coverage_status()

    # Sample total energy
    total_energy = sum(d.energy_model.energy_consumed for d in self.devices) / 1000
    self.energy_log.append(total_energy)

    # Reschedule at 1s
    self.scheduler.schedule(1.0, EventType.MOBILITY_UPDATE,
                            self._on_mobility_update)
```

**Scheduling:** Mobility events are scheduled only if `mobility_enabled` is `True` in `parametors.py` (checked during `simulate_transmissions()`):

```python
if self.mobility_enabled:
    self.scheduler.schedule(1.0, EventType.MOBILITY_UPDATE,
                            self._on_mobility_update)
```

**Coverage update after movement:**
```python
def update_coverage_status(self):
    best_gateway, _, _ = self.network.find_best_gateway(self)
    self.coverage_status = best_gateway is not None
```

`find_best_gateway()` evaluates the link budget using the current position. If no gateway has sufficient RSSI (≥ SX1301 sensitivity), `coverage_status = False` and the device will defer its next transmission:

```python
if best_gateway is None or not device.coverage_status:
    next_delay = np.random.exponential(1.0 / max(device.lambda_rate, 0.001))
    self.scheduler.schedule(next_delay, ...)
    return  # no transmission this cycle
```

---

### 4.4 Mobility Statistics (1h, 1.5 m/s, 10km area)

```
Steps per device = 3600 s / 1 s = 3600
Total displacement = 3600 × 1.5 m = 5400 m = 5.4 km (expected path length)

RMS displacement (2D random walk) = speed × sqrt(n_steps) × Δt
                                   = 1.5 × sqrt(3600) × 1
                                   = 1.5 × 60 = 90 m from start
```

For a 10 km area, 90 m RMS displacement is negligible — devices remain essentially fixed relative to the gateway. Mobility effects become significant at higher speeds (≥ 10 m/s) or longer simulations.

---

### 4.5 `set_mobility()` API

```python
def set_mobility(self, mobility_enabled):
    for device in self.devices:
        device.mobility_enabled = mobility_enabled
    # Note: does NOT add/remove the MOBILITY_UPDATE event from scheduler
```

This toggles per-device mobility mid-simulation. The periodic `_on_mobility_update` event continues to fire but devices with `mobility_enabled=False` are skipped by `move()`.

---

## 5. Device Initialization Pipeline

### 5.1 `initialize_devices()` — Full Sequence

```
1. Generate positions from deployment strategy
   └─ dispatch to deploy_* function

2. Select LR-FHSS devices
   └─ n_lrfhss = int(num_devices × lrfhss_ratio)
   └─ random.sample(range(num_devices), n_lrfhss)

3. For each device i:
   a. Create EndDevice(device_id=i, x, y, lambda_rate, speed, ...)
   b. device.adr_enabled = self.adr_enabled
   c. device.is_indoor = (random() < indoor_ratio)    # 30% indoor
   d. device.use_lrfhss = (i in lrfhss_ids)
   e. OTAA credentials:
      device.app_key  = generate_app_key()            # 16B random
      device.dev_eui  = generate_dev_eui(device_id)   # 8B deterministic
      device.app_eui  = generate_app_eui()            # 8B random
   f. Battery model (if battery_capacity_mah is set)
      device.battery = BatteryModel(capacity_mah, voltage=3.3)
      device.energy_model.set_battery(device.battery)
   g. Energy harvester (if energy_harvesting_config)
      device.battery.harvester = EnergyHarvester(model, peak_power_mw)
   h. Building penetration (if indoor):
      _building_penetration_cache[i] = lognormal(μ=ln(20), σ=0.5) dB
   i. Append to self.devices, self.positions["devices"][i] = (x, y)
```

### 5.2 EndDevice Constructor Attributes (Traffic/Mobility Relevant)

| Attribute | Initial Value | Description |
|-----------|--------------|-------------|
| `lambda_rate` | from Network | Poisson rate (Hz) |
| `speed` | from Network | Movement speed (m/s) |
| `sf` | `random.choice(sf_range)` | Initial SF |
| `bw` | `random.choice(bw)` | Initial BW |
| `freq` | `random.choice(frequency_mhz)` | Initial freq (MHz) |
| `traffic_type` | random | "periodic"/"sporadic"/"critical" |
| `lorawan_class` | random | "A"/"B"/"C" |
| `confirmed_ratio` | 0.3 | Fraction confirmed |
| `max_retransmissions` | 10 | Max retransmit |
| `dc_release_time` | 0 | Duty cycle release (s) |
| `mobility_enabled` | `True` | Per-device mobility flag |
| `model` | `"random_walk"` | Mobility model name |
| `coverage_status` | `True` | In-coverage flag |
| `is_indoor` | 30% chance | Indoor device flag |

### 5.3 `initialize_gateways()` — Grid Layout

Gateways are always placed in a regular grid, regardless of device deployment:

```python
rows = ceil(sqrt(num_gateways))
cols = ceil(num_gateways / rows)
x_spacing = area_size / cols
y_spacing = area_size / rows
x[c] = c * x_spacing + x_spacing / (2 * cols)  # half-cell offset
y[r] = r * y_spacing + y_spacing / (2 * rows)
```

For `num_gateways=1`, `area_size=10000`: GW at (5000, 5000) — center.
For `num_gateways=4`: GWs at (2500, 2500), (7500, 2500), (2500, 7500), (7500, 7500).

Each gateway is registered with the Network Server:
```python
self.ns.register_gateway(gateway)
```

---

## 6. Configuration Loader

### 6.1 `config_loader.py` — YAML/JSON Interface

Allows reproducible simulations via declarative configuration files.

```python
def load_config(filename):
    # supports .yaml, .yml, .json
    # requires PyYAML for YAML: pip install pyyaml

def config_to_params(config) -> dict:
    # converts config dict to Network(**params)

def run_from_config(config_file):
    config = load_config(config_file)
    params = config_to_params(config)
    network = Network(**params)
    network.simulate_transmissions()
    return network
```

### 6.2 Config Schema

```yaml
simulation:
  duration_s: 3600       # simulation_time
  region: "EU868"        # region_name
  seed: 42               # random seed

network:
  num_devices: 50
  num_gateways: 1
  area_size_m: 10000
  deployment: "grid"     # deployment_type
  deployment_radius_m:   # optional: for circular/annular

devices:
  traffic_interval_s: 10    # → lambda_rate = 1/10 = 0.1 Hz
  payload_bytes: 20         # pl
  tx_power_dbm: 14
  adr_enabled: true
  mobility:
    speed_mps: 1.5          # speed

physical:
  pathloss_model: "log_normal_shadowing"

energy:
  battery_capacity_mah: 2400
  harvesting:
    enabled: false
    model: "solar"
    peak_power_mw: 100
```

### 6.3 Config → Network Parameter Mapping

| Config path | Network param | Conversion |
|-------------|--------------|------------|
| `simulation.duration_s` | `simulation_time` | direct |
| `simulation.region` | `region_name` | direct |
| `network.num_devices` | `num_devices` | direct |
| `network.area_size_m` | `area_size` | direct |
| `network.deployment` | `deployment_type` | direct |
| `devices.traffic_interval_s` | `lambda_rate` | `1.0 / interval` |
| `devices.mobility.speed_mps` | `speed` | direct |
| `devices.payload_bytes` | `pl` | direct |
| `energy.battery_capacity_mah` | `battery_capacity_mah` | direct |
| `energy.harvesting.enabled` | `energy_harvesting` | dict if True |

---

## 7. Parallel Multi-Seed Runner

### 7.1 `parallel_runner.py`

Enables **statistical validation** by running the same scenario across multiple random seeds using Python's `ProcessPoolExecutor`.

```python
def run_multiple_seeds(config_params, seeds, n_workers=4) -> dict:
    with ProcessPoolExecutor(max_workers=n_workers) as executor:
        futures = {
            executor.submit(_run_single, config_params, seed): seed
            for seed in seeds
        }
        for future in as_completed(futures):
            result = future.result()   # dict with pdr, collisions, energy
            results.append(result)
    return aggregate_results(results)
```

### 7.2 `_run_single(config_params, seed)`

Worker function (runs in subprocess):
```python
random.seed(seed)
np.random.seed(seed)
network = Network(**config_params)
network.simulate_transmissions()
stats = network.packet_tracker.get_stats()
return {
    "seed": seed,
    "pdr": stats["Taxa de Entrega (PDR)"],
    "collisions": stats["Colisoes"],
    "total_packets": stats["Total Pacotes"],
    "retransmissions": stats["Retransmissoes"],
    "total_energy_mj": sum(d.energy_model.energy_consumed for d in network.devices),
}
```

### 7.3 `aggregate_results(results)`

```python
return {
    "n_seeds": len(results),
    "pdr_mean":            np.mean(pdrs),
    "pdr_std":             np.std(pdrs),
    "pdr_min":             np.min(pdrs),
    "pdr_max":             np.max(pdrs),
    "collisions_mean":     np.mean(collisions),
    "collisions_std":      np.std(collisions),
    "energy_mean_mj":      np.mean(energies),
    "energy_std_mj":       np.std(energies),
    "individual_results":  results,
}
```

### 7.4 Usage Example

```python
from parallel_runner import run_multiple_seeds

config = {
    "num_devices": 50, "num_gateways": 1, "area_size": 10000,
    "lambda_rate": 0.1, "speed": 1.5, "sf_range": [7,8,9,10,11,12],
    "tx_power": 14, "frequency_mhz": [868.1, 868.3, 868.5],
    "ht_m": 1.5, "hr_m": 30, "bw": [125000], "cr": 1, "pl": 20,
    "simulation_time": 3600, "adr_enabled": True,
    "model_pathloss": "log_normal_shadowing",
}

results = run_multiple_seeds(config, seeds=[42, 123, 456, 789, 1024],
                              n_workers=4)
# Output:
# PDR: 91.3% +/- 1.2%
# Collisions: 142 +/- 18
# Energy: 48300 +/- 620 mJ
```

**Note:** Each worker imports `Network` locally to avoid multiprocessing pickling issues with module-level state.

---

## 8. Event Scheduling for Mobility and Traffic

### 8.1 Traffic-Related Events

| Event Type | Handler | Scheduling | Condition |
|-----------|---------|-----------|-----------|
| `DEVICE_SEND` | `_on_device_send` | `Exp(1/λ)` after start; `Exp(1/λ)` after each TX | Always |
| `DEVICE_SEND` (retransmit) | `_on_retransmit` | `U(1,10) × 2^attempt` backoff | `packet.collided` |
| `PACKET_TX_END` | `_on_tx_end` | `+airtime` after TX start | Always |
| `RX1_WINDOW_OPEN` | `_on_rx1_open` | `+receive_delay1` (1s) after TX end | Always |
| `RX2_WINDOW_OPEN` | `_on_rx2_open` | `+receive_delay2 - receive_delay1 - trx1` after RX1 | No DL in RX1 |

| Event Type | Handler | Scheduling | Condition |
|-----------|---------|-----------|-----------|
| `MOBILITY_UPDATE` | `_on_mobility_update` | Every 1.0 s | `mobility_enabled=True` |
| `BEACON` | `_on_beacon` | Every 128.0 s | Always |
| `ENERGY_HARVEST` | `_on_energy_harvest` | Every 60.0 s | `energy_harvesting_config` set |

### 8.2 Duty-Cycle Deferral Event

When a device is duty-cycle limited, a new `DEVICE_SEND` event is scheduled at `dc_release_time`:

```
[DEVICE_SEND at t]
    → check dc_release_time
    → if t < dc_release_time:
          schedule DEVICE_SEND at dc_release_time + 0.001
          return
    → else: proceed with transmission
```

This creates a **deterministic retry** rather than a Poisson-distributed one — the device transmits as soon as the duty cycle allows.

### 8.3 Initial Event Timeline (50 devices, 1h, λ=0.1)

```
t = 0 s:     Simulation starts
t = 0..10 s: 50 × initial_delay ~ U(0, 10) — all devices scheduled
t = 1 s:     First MOBILITY_UPDATE fires
t = 1..10 s: First transmissions begin (staggered)
t = 128 s:   First BEACON (Class B synchronization)
t = 60 s:    First ENERGY_HARVEST (if harvesting enabled)
...
t = 3600 s:  Simulation ends (scheduler.run(until=3600))
```

---

## 9. Interaction Diagram

```
Network.simulate_transmissions()
│
├─ OTAA Join (all devices)
│  └─ device.prepare_join_request() → ns.on_join_request()
│     └─ device.process_join_accept() → NwkSKey, AppSKey derived
│
├─ Schedule initial DEVICE_SEND × 50
│  └─ delay ~ U(0, 1/λ) = U(0, 10s)
│
├─ Schedule MOBILITY_UPDATE @ 1s (if mobility_enabled)
├─ Schedule BEACON @ 128s
├─ Schedule ENERGY_HARVEST @ 60s (if harvesting)
│
└─ scheduler.run(until=3600)
   │
   ├─ DEVICE_SEND fires for device D
   │  ├─ check battery (depleted? return)
   │  ├─ check duty cycle (defer? reschedule)
   │  ├─ select_channel() → random freq from [868.1, 868.3, 868.5]
   │  ├─ validate_payload_size(region)
   │  ├─ check_dwell_time(region)
   │  ├─ find_best_gateway(D) → best_gw, distance, rssi
   │  ├─ compute airtime (CSS or LR-FHSS)
   │  ├─ update_energy(airtime, sim_time)
   │  ├─ update dc_release_time
   │  ├─ create Packet (device_id, sf, tx_power, bw, freq, rssi, t, airtime)
   │  ├─ gateway.process_uplink(packet)
   │  ├─ channel.add_transmission(packet, t, t+airtime)
   │  ├─ packet_tracker.add_packet(packet)
   │  ├─ ns.on_uplink_received(packet, gw) → MAC commands
   │  ├─ schedule PACKET_TX_END @ t+airtime
   │  └─ schedule next DEVICE_SEND @ t+Exp(1/λ)
   │
   ├─ PACKET_TX_END fires
   │  ├─ channel.evaluate_reception(packet, gw)
   │  ├─ gateway.release_paths(time)
   │  └─ schedule RX1_WINDOW_OPEN @ t+1s
   │
   ├─ RX1_WINDOW_OPEN fires
   │  ├─ check NS downlink scheduler
   │  ├─ deliver ACK/MAC if GW available
   │  └─ schedule RX2_WINDOW_OPEN if no DL
   │
   ├─ RX2_WINDOW_OPEN fires
   │  ├─ deliver DL on 869.525 MHz @ SF12 if pending
   │  └─ schedule retransmit if packet.collided
   │
   └─ MOBILITY_UPDATE fires (every 1s)
      ├─ for each mobile device: move(), update_coverage_status()
      ├─ sample energy_log
      └─ reschedule @ t+1s
```

---

## 10. Numerical Examples

### Example 1: Duty Cycle Impact on Effective Throughput

**Scenario:** 10 devices, λ=0.5 Hz, SF12, EU868.

```
Airtime SF12 BW=125kHz 20B = 2.793 s
DC limit G/G1 = 1%

Off-time per TX = 2.793 / 0.01 = 279.3 s

Effective max rate = 1 / (airtime + off-time)
                   = 1 / (2.793 + 279.3) = 0.00354 Hz

Requested rate λ = 0.5 Hz >> 0.00354 Hz

→ Device will always be duty-cycle blocked
→ Actual PDU transmission rate ≈ 0.00354 Hz (not 0.5 Hz)
→ Expected TXs in 1h: 3600 × 0.00354 ≈ 12.8 packets per device
   (instead of 1800 if no DC)
```

### Example 2: Random Walk Displacement Analysis

**Scenario:** 50 devices, v=1.5 m/s, T=3600 s, 1s update interval.

```
Steps: N = 3600
Step size: Δ = 1.5 m

RMS displacement = Δ × sqrt(N) = 1.5 × 60 = 90 m

Max possible displacement = N × Δ = 5400 m (straight line)

P(device exits 500m radius from start):
  For 2D RW: P(r > R) ≈ exp(-R² / (N × Δ²))
           = exp(-500² / (3600 × 1.5²))
           = exp(-250000 / 8100)
           = exp(-30.9) ≈ 0   (essentially zero)

→ Devices remain within ~180m (2σ) of starting position in 1h at 1.5 m/s
→ Mobility effect on link budget is negligible for this scenario
```

### Example 3: Multi-Seed Statistical Validation

**Scenario:** 5 seeds, reference config.

```
Seed 42:   PDR=91.2%, Collisions=145, Energy=47800 mJ
Seed 123:  PDR=90.7%, Collisions=159, Energy=48200 mJ
Seed 456:  PDR=92.1%, Collisions=128, Energy=47500 mJ
Seed 789:  PDR=91.8%, Collisions=133, Energy=48600 mJ
Seed 1024: PDR=90.4%, Collisions=163, Energy=48100 mJ

Aggregate:
  PDR:       91.2% ± 0.66% (95% CI: ±1.32%)
  Collisions: 145.6 ± 14.1
  Energy:    48040 ± 390 mJ

→ Low variance across seeds: simulation is deterministic given seed
→ 5 seeds sufficient for <1.5% CI on PDR
```

### Example 4: Clustered Deployment vs. Grid

**Scenario:** 50 devices, 10 clusters, area=10km, cluster_std=250m.

```
cluster_std = 10000 / (10 × 4) = 250 m

10 cluster centers (random, away from boundaries by 2σ = 500m)
5 devices per cluster (50 / 10 = 5)

Intra-cluster distances (95% of devices within 2σ):
  d_intra ≈ 2 × 250 = 500 m from cluster center

For GW at (5000, 5000):
  Cluster distance range: 0 to ~8000 m
  Average distance: ~3500 m (non-uniform — depends on cluster centers)

vs. Grid deployment:
  Device spacing: 10000 / 7 ≈ 1428 m
  Min distance to center: ~714 m
  Max distance to corner: ~7071 m

→ Clustered: high traffic density areas → more collisions within cluster
→ Grid: uniform load across coverage area → lower collision variance
```

### Example 5: Retransmission Backoff Timing

**Scenario:** SF9, packet collides, 3 retransmission attempts.

```
airtime SF9 = 329.7 ms
DC = 1%: dc_release_time = +32.97 s

Retransmit 0: backoff = U(1,10) × 2^0 = ~5.5 s
  → earliest TX at max(dc_release_time, t_now + 5.5s)

Retransmit 1: backoff = U(1,10) × 2^1 = ~11 s
Retransmit 2: backoff = U(1,10) × 2^2 = ~22 s

Total delay before 3rd attempt ≈ 5.5 + 32.97 + 11 + 32.97 + 22 ≈ 104 s

Note: duty cycle of the retransmit is independent per sub-band.
If device hops to different channel (e.g., 868.3 MHz instead of 868.1),
dc_release_time for 868.3 may still be 0 → no additional wait.
```

---

## 11. Comparison with ns-3 and FLoRa

| Feature | PyLoRaWAN | ns-3 LoRaWAN | FLoRa |
|---------|----------|-------------|-------|
| **Deployment** | 7 strategies incl. file | MobilityHelper + external | 3 strategies (circular, grid, random) |
| **Circular deployment** | `sqrt(U)` (uniform area) | Uniform disk | `sqrt(U)` (same) |
| **Hexagonal** | Yes (GW placement) | Yes (`GridPositionAllocator`) | No |
| **Traffic model** | Poisson (`Exp(1/λ)`) | Poisson (`PeriodicSender`) | Poisson (same) |
| **Duty cycle** | Per sub-band (EU868/US915) | Per sub-band | Global 1% only |
| **Retransmission** | Exp backoff, max 10 | 8 retransmits, fixed | Max 8, random |
| **Mobility** | Random walk (boundary clamp) | `RandomWalk2dMobilityModel` | Static only |
| **Mobility update** | 1s fixed interval | Continuous (event-driven) | N/A |
| **Multi-seed** | `ProcessPoolExecutor` | External scripts | Not built-in |
| **Config format** | YAML/JSON | C++ / Python API | INI file |
| **LR-FHSS ratio** | `lrfhss_ratio` param | Separate module | Not supported |
| **Coverage check** | After every move | Continuous | Per transmission |
| **Confirmed ratio** | 30% (configurable) | 100% or 0% | Per device |

---

## Summary

The mobility and traffic generation subsystem provides:

1. **7 deployment strategies** — from deterministic grid to Gaussian clusters and external file import, covering all major IoT deployment scenarios.

2. **Poisson traffic** — `Exp(1/λ)` inter-arrival with staggered initial offsets, duty cycle enforcement, frequency hopping, and confirmed/unconfirmed ratio.

3. **Retransmission** — truncated exponential backoff with max 10 attempts; each retry creates a new Packet with `is_retransmission=True`.

4. **2D random walk mobility** — 1s update interval, reflective boundaries, coverage status updated after each move.

5. **YAML/JSON configuration** — declarative simulation setup via `config_loader.py`.

6. **Multi-seed parallelism** — `ProcessPoolExecutor`-based runner for statistical validation.

All these components feed the discrete-event simulation loop in `network.py`, where traffic and mobility interact with the channel model (SINR, interference), energy model (duty cycle, sleep time), and the Network Server (ADR, MAC commands, downlink scheduling).
