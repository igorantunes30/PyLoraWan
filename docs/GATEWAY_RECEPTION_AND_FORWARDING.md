# Gateway Reception and Forwarding

**PyLoRaWAN Simulator — Technical Reference**

---

## Table of Contents

1. [Overview](#1-overview)
2. [Gateway Data Model](#2-gateway-data-model)
3. [ReceptionPath — SX1301 Demodulator Model](#3-receptionpath--sx1301-demodulator-model)
4. [process_uplink — Full Reception Pipeline](#4-process_uplink--full-reception-pipeline)
5. [Downlink Duty Cycle Tracking](#5-downlink-duty-cycle-tracking)
6. [Load Balancing](#6-load-balancing)
7. [Multi-Gateway Diversity (MRC)](#7-multi-gateway-diversity-mrc)
8. [Gateway Manager (Network Server Side)](#8-gateway-manager-network-server-side)
9. [Gateway Initialization and Placement](#9-gateway-initialization-and-placement)
10. [Forwarding to the Network Server](#10-forwarding-to-the-network-server)
11. [Downlink Selection for RX Windows](#11-downlink-selection-for-rx-windows)
12. [Complete Reception Sequence Diagram](#12-complete-reception-sequence-diagram)
13. [Numerical Examples](#13-numerical-examples)
14. [Comparison with ns-3 and FLoRa](#14-comparison-with-ns-3-and-flora)

---

## 1. Overview

The gateway is the **RF bridge** between end devices and the Network Server. In PyLoRaWAN it models the **Semtech SX1301** multi-channel concentrator chip, which can demodulate up to 8 packets simultaneously across multiple frequencies and spreading factors.

Reception and forwarding involves three distinct components:

| Component | File | Responsibility |
|-----------|------|---------------|
| `Gateway` | `gateway.py` | RF reception, link budget, SINR, path allocation |
| `GatewayManager` | `network_server/gateway_manager.py` | NS-side duty cycle, diversity, DL gateway selection |
| `Network._on_rx1_open/rx2_open` | `network.py` | Downlink window coordination |

---

## 2. Gateway Data Model

### 2.1 Constructor

```python
class Gateway:
    def __init__(self, gw_id, x, y, network,
                 max_capacity=100, num_reception_paths=8):
```

### 2.2 Attributes

| Attribute | Type | Initial | Description |
|-----------|------|---------|-------------|
| `gw_id` | int | set | Unique gateway identifier |
| `x`, `y` | float | set | Position (meters) |
| `network` | Network | ref | Back-reference to orchestrator |
| `received_packets` | list | `[]` | All packets processed by this GW |
| `max_capacity` | int | 100 | Soft buffer limit before load balancing |
| `num_reception_paths` | int | 8 | SX1301 demodulator count |
| `reception_paths` | list | 8×`ReceptionPath` | Demodulator states |
| `saturation_events` | int | 0 | Count of path-saturation drops |
| `last_dl_time` | dict | `{rx1:0, rx2:0}` | Last DL TX time per window |
| `total_dl_sent` | int | 0 | Total downlinks transmitted |
| `dl_busy_until` | dict | `{}` | `freq → tx_end_time` (G8 DL-UL block) |
| `assigned_frequencies` | set | `set()` | Dynamic frequency assignment tracking |

---

## 3. ReceptionPath — SX1301 Demodulator Model

### 3.1 Class

```python
class ReceptionPath:
    busy: bool       # currently occupied
    packet: Packet   # the packet being demodulated (None if free)
    tx_end: float    # simulation time when demodulation finishes
```

### 3.2 SX1301 Hardware Background

The Semtech SX1301 concentrator contains 8 programmable demodulation paths. Each path can be tuned to a specific (frequency, SF) combination. In the standard LoRa gateway configuration, paths are distributed across channels and SFs, allowing simultaneous reception of up to 8 packets regardless of which SF or channel they use.

PyLoRaWAN models this as 8 independent paths with no per-path channel/SF constraint — any packet can be assigned to any free path, matching the behavior of a fully flexible demodulator bank.

### 3.3 `is_free(current_time)`

```python
def is_free(self, current_time):
    return not self.busy or self.tx_end <= current_time
```

A path is considered free if:
- It was never assigned (`busy=False`), OR
- Its previous packet's transmission has ended (`tx_end ≤ current_time`)

This allows the gateway to reuse a path the instant a packet finishes transmitting, without waiting for an explicit release call.

### 3.4 `assign(packet, tx_end)`

```python
def assign(self, packet, tx_end):
    self.busy = True
    self.packet = packet
    self.tx_end = tx_end   # = current_time + packet.rectime
```

### 3.5 `release(current_time)`

```python
def release(self, current_time):
    if self.tx_end <= current_time:
        self.busy = False
        self.packet = None
```

Called from `Gateway.release_paths(current_time)` at `TX_END` events.

### 3.6 `try_allocate_path(packet, current_time)`

```python
def try_allocate_path(self, packet, current_time):
    tx_end = current_time + packet.rectime
    for path in self.reception_paths:
        if path.is_free(current_time):
            path.assign(packet, tx_end)
            return True
    self.saturation_events += 1
    return False
```

Iterates the 8 paths in order, assigns the first free one, and returns `True`. If all 8 are occupied, increments `saturation_events` and returns `False` — the packet is lost.

**Saturation probability:** With 8 paths and typical LoRaWAN traffic (50 devices, λ=0.1 Hz), the probability of all 8 paths being simultaneously occupied is very low for SF7 (ToA=56ms) but non-negligible for SF12 (ToA=1319ms):

| SF | ToA (ms) | E[simultaneous TX, 50 dev] | P(saturate) approx |
|----|---------|--------------------------|-------------------|
| 7 | 56.6 | 50×0.1×0.0566 = 0.28 | ~0% |
| 9 | 185.4 | 50×0.1×0.185 = 0.93 | ~0% |
| 12 | 1319 | 50×0.1×1.319 = 6.6 | significant if all SF12 |

For the reference scenario with mixed SFs, saturation is rare. Saturation becomes relevant when many SF12 devices are duty-cycle limited (bursting upon DC release) or in high-density networks (>200 devices).

### 3.7 `active_paths_count(current_time)`

```python
def active_paths_count(self, current_time):
    return sum(1 for p in self.reception_paths if not p.is_free(current_time))
```

Used for logging and load balancing decisions.

---

## 4. `process_uplink` — Full Reception Pipeline

**Signature:** `Gateway.process_uplink(packet)`

This is the central function that executes the full RF reception pipeline from DL-UL check through SINR computation.

### 4.1 Step 1 — Input Validation

```python
if not packet or not hasattr(packet, 'device_id'):
    return

device = next((d for d in self.network.devices if d.device_id == packet.device_id), None)
if not device:
    return
```

### 4.2 Step 2 — DL-UL Interference Check (G8)

```python
current_time = packet.arrival_time

if self.dl_busy_until.get(packet.freq, 0) > current_time:
    packet.collided = True
    return
```

`dl_busy_until[freq]` records the time until which the gateway is transmitting a downlink on that frequency. An uplink arriving during a DL transmission is physically undetectable by the half-duplex SX1301.

### 4.3 Step 3 — Reception Path Allocation

```python
if not self.try_allocate_path(packet, current_time):
    packet.collided = True
    return
```

If all 8 demodulators are busy, the packet is immediately dropped. No further processing occurs.

### 4.4 Step 4 — Link Budget

```python
distance = max(
    sqrt((device.x - self.x)² + (device.y - self.y)² + (ht_m - hr_m)²),
    1.0  # minimum 1m to avoid log(0)
)

path_loss = self.network.pathloss(
    distance, device.freq,
    model_pathloss=self.network.model_pathloss,
    device_x=device.x, device_y=device.y   # for correlated shadowing (G2)
)
path_loss += self.network.get_building_penetration(device)   # BPL (G3)

packet.rssi = device.tx_power + ed_antenna_gain + gw_antenna_gain - path_loss
```

**3D distance:** The Euclidean distance includes the **height difference** between the device (`ht_m=1.5m`) and the gateway (`hr_m=30m`):
```
distance = sqrt(d_xy² + (ht - hr)²)
         = sqrt(d_xy² + (1.5 - 30)²)
         = sqrt(d_xy² + 812.25)
```

At typical distances (>100m), the height term contributes < 0.1% — but it prevents `distance=0` issues when device and GW are co-located.

**Building penetration (G3):** Indoor devices have a cached lognormal BPL:
```python
self._building_penetration_cache[i] = np.random.lognormal(mean=np.log(20), sigma=0.5)
# μ = ln(20) dB → median BPL = 20 dB
# σ = 0.5 in log scale → P90 ≈ 33 dB, P10 ≈ 12 dB
```

### 4.5 Step 5 — Interference Accumulation (G14)

Full description in `INTERFERENCE_EVALUATION.md`. Summary:

```python
interference_power_linear = 0
interference_per_sf = {}

for (other_pkt, tx_start, tx_end) in self.network.channel.on_air:
    if same packet / different frequency / no overlap: continue

    overlap_ratio = overlap_duration / packet.rectime
    other_rssi_linear = 10 ** (other_rssi_dbm / 10)
    interference_power_linear += other_rssi_linear * overlap_ratio
    interference_per_sf[other_pkt.sf] += other_rssi_linear * overlap_ratio

packet.interference_per_sf = interference_per_sf
```

### 4.6 Step 6 — SINR, SNR, SIR

```python
packet.noise_floor = self.network.calculate_noise_floor(device.bw)
# = 10*log10(k × T × BW) + 30 + NF  [dBm]

noise_linear  = 10 ** (packet.noise_floor / 10)
signal_linear = 10 ** (packet.rssi / 10)

sinr_linear = signal_linear / (interference_power_linear + noise_linear)
packet.sinr = 10 * log10(sinr_linear)

packet.sir = (packet.rssi - 10*log10(interference_power_linear)
              if interference_power_linear > 0 else float('inf'))
packet.snr = packet.rssi - packet.noise_floor
device.snr = packet.snr   # update device for ADR history
```

### 4.7 Step 7 — Buffer and Load Balance

```python
if len(self.received_packets) >= self.max_capacity:
    self.balance_load(packet, device)
    return

self.received_packets.append(packet)
```

Under normal operation, packets are appended to `received_packets`. At 100 packets (max_capacity), `balance_load()` redirects to an alternative gateway.

### 4.8 Summary Table

| Step | Guard condition | On fail |
|------|----------------|---------|
| Validation | Invalid packet or unknown device | `return` |
| DL-UL (G8) | `dl_busy_until[freq] > t` | `collided=True; return` |
| Path allocation | All 8 paths busy | `collided=True; return` |
| Link budget | — | — (RSSI computed) |
| Interference | — | — (SINR computed) |
| Buffer | `len(received_packets) >= 100` | Redirect to alt GW |

---

## 5. Downlink Duty Cycle Tracking

The gateway tracks downlink duty cycle at two levels:

### 5.1 Per-Window Timestamps (`last_dl_time`)

```python
self.last_dl_time = {"rx1": 0, "rx2": 0}
```

Updated by `GatewayManager.record_downlink()`:
```python
gw_status.last_tx_time[window] = current_time + airtime
```

Used by `GatewayManager.can_send_downlink()`:
```python
dc_limit = self.rx1_dc if window == "rx1" else self.rx2_dc
# rx1_dc = 1%, rx2_dc = 10%
off_time = airtime / dc_limit - airtime
return current_time >= last_tx + off_time
```

**Off-time formula:**
```
off_time = airtime / DC - airtime = airtime × (1/DC - 1)

For RX1 (1% DC), DL SF9 (144ms):
off_time = 0.144 × (100 - 1) = 0.144 × 99 = 14.26 s

For RX2 (10% DC), DL SF12 (991ms):
off_time = 0.991 × (10 - 1) = 0.991 × 9 = 8.92 s
```

### 5.2 Per-Frequency Blocking (`dl_busy_until`)

```python
self.dl_busy_until = {}   # freq → tx_end_time
```

Set in `_on_rx1_open()` / `_on_rx2_open()` when a downlink is actually transmitted:
```python
best_gw.dl_busy_until[packet.freq] = time + dl_airtime
```

This prevents the gateway from receiving uplinks on the same frequency during DL transmission — the G8 half-duplex constraint (Section 4.2 above).

### 5.3 Network-Wide DL Tracking

```python
# In Network:
self._network_dl_busy = {}   # freq → tx_end_time
```

Set alongside `dl_busy_until`:
```python
self._network_dl_busy[packet.freq] = time + dl_airtime
```

Prevents two different gateways from simultaneously transmitting DL on the same frequency to different devices (DL-DL interference):
```python
if self._network_dl_busy.get(packet.freq, 0) > time:
    # DL-DL collision: return DL packet to queue
    self.ns.dl_scheduler.schedule(device.device_id, dl_packet)
```

---

## 6. Load Balancing

### 6.1 `balance_load(packet, device)`

```python
def balance_load(self, packet, device):
    closest_gw, _, _ = self.network.find_best_gateway(device, avoid_congestion=True)
    if closest_gw:
        closest_gw.received_packets.append(packet)
    else:
        self.received_packets.append(packet)   # local overflow
```

`avoid_congestion=True` excludes the current gateway and selects the alternative with the fewest `received_packets`:
```python
# In find_best_gateway(avoid_congestion=True):
other_gateways = [g for g in available_gateways if g.gw_id != device.current_gateway.gw_id]
return min(other_gateways, key=lambda x: len(x[0].received_packets))
```

**Limitation:** This is a soft redirection — the link budget (RSSI, SINR) was computed for the original gateway. The alternative gateway records the packet without recomputing RF metrics. It is primarily a queue management mechanism, not a physical handoff.

### 6.2 `dynamic_frequency_assignment()`

```python
def dynamic_frequency_assignment(self):
    available_frequencies = set(self.network.frequency_mhz)
    used_frequencies = {packet.freq for packet in self.received_packets}
    free_frequencies = available_frequencies - used_frequencies
    if free_frequencies:
        self.assigned_frequencies.add(random.choice(list(free_frequencies)))
```

Identifies frequencies not currently used by received packets and adds one to the gateway's assigned set. Called manually — not invoked automatically during simulation.

### 6.3 `clear_old_packets()`

```python
def clear_old_packets(self):
    if len(self.received_packets) > self.max_capacity:
        self.received_packets = self.received_packets[-self.max_capacity:]
```

Trims the `received_packets` buffer to the last `max_capacity` entries. Not called during normal simulation — available for memory management in long-running scenarios.

---

## 7. Multi-Gateway Diversity (MRC)

When `num_gateways > 1`, PyLoRaWAN implements Maximal Ratio Combining (MRC) of per-gateway SNR values (Sprint 9).

### 7.1 Trigger Condition

```python
if not device.use_lrfhss and len(self.gateways) > 1:
    mrc_receptions = self.ns.gateway_manager.process_uplink_from_all(
        packet, self.gateways, self)
```

### 7.2 `process_uplink_from_all(packet, gateways, network)`

For each gateway independently:

```python
distance = sqrt((dev.x - gw.x)² + (dev.y - gw.y)² + (ht - hr)²)
path_loss = network.pathloss(distance, freq, ...)
rssi      = tx_power + Ged + Ggw - path_loss
snr       = rssi - network.calculate_noise_floor(bw)
sensitivity = device.set_sensibilidade(device.sf, device.bw)

if rssi >= sensitivity:
    receptions.append({gateway, rssi, snr, distance})
```

**Note:** This uses `sensitivity_table` (SX1272, device-level) rather than `gw_sensitivity_table` (SX1301). This is slightly more conservative for secondary GWs — they must meet the device sensitivity threshold, not the superior GW threshold.

### 7.3 MRC Combining

```python
if len(mrc_receptions) > 1:
    snr_linear_sum = sum(10 ** (r['snr'] / 10.0) for r in mrc_receptions)
    packet.snr_mrc = 10.0 * log10(snr_linear_sum)
    packet.mrc_gw_count = len(mrc_receptions)
```

**MRC gain table (two equal-SNR gateways):**

| Single GW SNR | MRC SNR (2 GW) | Gain |
|--------------|---------------|------|
| −15.0 dB | −11.97 dB | +3.0 dB |
| −13.0 dB | −9.97 dB | +3.0 dB |
| −10.0 dB | −6.97 dB | +3.0 dB |
| 0 dB | +3.01 dB | +3.0 dB |

The 3 dB gain for two equal-SNR gateways is a fundamental property of MRC: SNR_MRC = 2 × SNR_single in linear scale.

### 7.4 `snr_mrc` Used in `evaluate_reception()`

```python
# In channel.evaluate_reception():
if getattr(packet, 'snr_mrc', None) is not None:
    snr = packet.snr_mrc
else:
    snr = packet.rssi - noise_floor
```

The MRC SNR is used instead of single-GW SNR for the threshold check. This can rescue packets that would otherwise fail:

```
Single GW: SNR = −13.5 dB < snr_min[9] = −12.5 dB → would fail
MRC (2 GW, equal SNR): SNR_MRC = −10.47 dB > −12.5 dB → passes
```

### 7.5 Multi-GW Device Registry Update

All secondary gateways that received the packet are recorded in `device_registry`:
```python
for r in mrc_receptions:
    gw_id = r['gateway'].gw_id
    if gw_id != best_gateway.gw_id:
        dev_status.gateways[gw_id] = {
            'rssi': r['rssi'], 'snr': r['snr'],
            'sinr': None, 'time': time
        }
```

This is critical for downlink: the NS now knows multiple GWs can reach this device and picks the best one for the DL response.

---

## 8. Gateway Manager (Network Server Side)

### 8.1 `GatewayStatus`

```python
class GatewayStatus:
    gateway: Gateway          # reference to the Gateway object
    last_tx_time: dict        # {"rx1": float, "rx2": float}
    total_dl_sent: int
    duty_cycle_usage: float
```

### 8.2 `GatewayManager` Attributes

```python
class GatewayManager:
    gateways: dict[int, GatewayStatus]   # gw_id → status
    rx1_dc: float = 0.01                 # 1% downlink DC for RX1
    rx2_dc: float = 0.10                 # 10% downlink DC for RX2
```

### 8.3 `select_best_for_downlink(device_id, device_registry)`

```python
def select_best_for_downlink(self, device_id, device_registry):
    status = device_registry.get_status(device_id)
    best_gw_id, best_snr = None, -999

    for gw_id, gw_info in status.gateways.items():
        snr = gw_info.get("snr") or -999
        if snr > best_snr:
            gw_status = self.gateways.get(gw_id)
            if gw_status is not None:
                best_gw_id = gw_id
                best_snr   = snr

    return self.gateways[best_gw_id].gateway if best_gw_id else None
```

Selects the gateway with the **highest uplink SNR** from the device — the best uplink path is also the best downlink path (channel reciprocity). This is equivalent to the ns-3 NetworkServer gateway selection strategy.

### 8.4 `can_send_downlink(gw_id, current_time, airtime, window)`

```python
def can_send_downlink(self, gw_id, current_time, airtime, window="rx1"):
    dc_limit = self.rx1_dc if window == "rx1" else self.rx2_dc
    off_time = airtime / dc_limit - airtime
    last_tx  = gw_status.last_tx_time.get(window, 0)
    return current_time >= last_tx + off_time
```

Returns `True` if the gateway has completed its mandatory off-time since the last downlink on this window type.

### 8.5 `record_downlink(gw_id, current_time, airtime, window)`

```python
def record_downlink(self, gw_id, current_time, airtime, window="rx1"):
    gw_status.last_tx_time[window] = current_time + airtime
    gw_status.total_dl_sent += 1
```

Called immediately after a downlink is delivered. The off-time timer starts from `current_time + airtime` (end of transmission), not `current_time` (start).

---

## 9. Gateway Initialization and Placement

### 9.1 `Network.initialize_gateways()`

Gateways are always placed on a regular grid, regardless of device deployment type:

```python
rows = ceil(sqrt(num_gateways))
cols = ceil(num_gateways / rows)
x_spacing = area_size / cols
y_spacing = area_size / rows

for i in range(num_gateways):
    r = i // cols
    c = i % cols
    x = c * x_spacing + x_spacing / (2 * cols)   # half-cell offset → center-aligned
    y = r * y_spacing + y_spacing / (2 * rows)
```

**Placement examples:**

| num_gateways | rows × cols | Positions (10km area) |
|-------------|------------|----------------------|
| 1 | 1×1 | (5000, 5000) |
| 2 | 1×2 | (2500, 5000), (7500, 5000) |
| 4 | 2×2 | (2500, 2500), (7500, 2500), (2500, 7500), (7500, 7500) |
| 9 | 3×3 | (1667, 1667), (5000, 1667), (8333, 1667), ... |

### 9.2 NS Registration

Each gateway is registered with the Network Server immediately after creation:

```python
gateway = Gateway(gw_id=i, x=x, y=y, network=self)
self.gateways.append(gateway)
self.positions["gateways"][i] = (x, y)
self.ns.register_gateway(gateway)    # GatewayManager.register(gateway)
```

`GatewayManager.register()` creates a `GatewayStatus` entry:
```python
self.gateways[gateway.gw_id] = GatewayStatus(gateway)
```

---

## 10. Forwarding to the Network Server

The "forwarding" step in PyLoRaWAN is a direct in-process function call — there is no physical backhaul model (no packet loss, no latency). This matches the typical assumption in LoRaWAN simulators where the GW-NS link is considered ideal.

```python
# In Network._on_device_send(), immediately after process_uplink():
mac_commands = self.ns.on_uplink_received(packet, best_gateway)
```

The gateway that performed reception is passed to the NS as a reference. The NS uses it to:
1. Update the device registry with per-gateway RF metrics
2. Select the best gateway for downlink delivery

### 10.1 NS Processing Result

`on_uplink_received()` returns a list of MAC commands. These are applied to the device in the same event cycle:

```python
if mac_commands:
    self._apply_mac_commands(device, mac_commands)
```

This immediate application simulates the RX window delivery: the device "receives" the MAC commands in its next RX window, which is modeled as instantaneous from the NS perspective.

### 10.2 What the NS Records per Gateway

From `DeviceRegistry.update(packet, gateway)`:

```python
status.gateways[gateway.gw_id] = {
    "rssi":  packet.rssi,
    "snr":   packet.snr,
    "sinr":  packet.sinr,
    "time":  packet.arrival_time,
}
```

This record is maintained per-gateway, not just per-device. `get_best_gateway()` returns the `gw_id` with the maximum SNR across all entries.

---

## 11. Downlink Selection for RX Windows

The downlink delivery process involves gateway selection at two decision points.

### 11.1 RX1 Window (`_on_rx1_open`)

```python
def _on_rx1_open(self, device, packet):
    device.radio_state = RadioState.RX1
    time = self.scheduler.now

    if self.ns.dl_scheduler.has_pending(device.device_id):
        best_gw = self.ns.gateway_manager.select_best_for_downlink(
            device.device_id, self.ns.device_registry)

        if best_gw is not None:
            dl_airtime = self._calc_dl_airtime(packet.sf, packet.bw)
            if self.ns.gateway_manager.can_send_downlink(
                    best_gw.gw_id, time, dl_airtime, "rx1"):

                dl_packet = self.ns.get_downlink(device.device_id)
                if dl_packet:
                    # DL-DL interference check
                    if self._network_dl_busy.get(packet.freq, 0) > time:
                        self.ns.dl_scheduler.schedule(device.device_id, dl_packet)
                    else:
                        self.ns.gateway_manager.record_downlink(
                            best_gw.gw_id, time, dl_airtime, "rx1")
                        best_gw.dl_busy_until[packet.freq] = time + dl_airtime
                        self._network_dl_busy[packet.freq] = time + dl_airtime
                        if dl_packet.packet_type == "ack" and not packet.collided:
                            packet.ack_received = True
                        if dl_packet.payload and hasattr(dl_packet.payload, 'cid'):
                            self._apply_mac_commands(device, [dl_packet.payload])
                        device.retransmission_attempts = 0
                        device.adr_ack_cnt = 0
                        device.radio_state = RadioState.IDLE
                        return   # DL delivered in RX1
```

If DL is successfully delivered in RX1, the device transitions directly to IDLE — no RX2 window is opened.

### 11.2 RX2 Window (`_on_rx2_open`)

If no DL was delivered in RX1 (or RX1 was skipped due to duty cycle/interference):

```python
def _on_rx2_open(self, device, packet):
    device.radio_state = RadioState.RX2

    rx2_freq = getattr(device, '_rx2_freq', None) or self.region.rx2_frequency
    rx2_sf, rx2_bw = self.region.dr_to_sf_bw(self.region.rx2_dr)
    # EU868 default: 869.525 MHz, SF12, BW=125kHz
```

The RX2 window uses regional parameters unless overridden by `RxParamSetupReq` (MAC command 0x05):
- `device._rx2_freq`: custom RX2 frequency
- `device._rx2_sf`, `device._rx2_bw`: custom RX2 data rate

### 11.3 DL Delivery Decision Flow

```
RX1/RX2 window opens
│
├─ [has_pending(device_id)?] ──No──→ skip to SLEEP/IDLE
│
├─ select_best_for_downlink() → best_gw (highest UL SNR)
│
├─ [best_gw found?] ──No──→ skip to SLEEP/IDLE
│
├─ can_send_downlink(gw_id, t, airtime, window)?
│   └─ No → skip to SLEEP/IDLE (GW duty cycle exhausted)
│
├─ get_downlink(device_id) → dl_packet (highest priority from heap)
│
├─ [_network_dl_busy[freq] > t?]
│   └─ Yes → re-queue dl_packet; skip (DL-DL interference)
│
└─ deliver:
    record_downlink(), dl_busy_until[freq], _network_dl_busy[freq]
    apply ACK / MAC commands
    device.retransmission_attempts = 0
    device.adr_ack_cnt = 0
    device.radio_state = IDLE (RX1) or SLEEP (RX2)
```

### 11.4 RX2 Parameters by Region

| Region | RX2 Frequency (MHz) | RX2 DR | RX2 SF | RX2 BW |
|--------|-------------------|--------|--------|--------|
| EU868 | 869.525 | DR0 | SF12 | 125 kHz |
| US915 | 923.3 | DR8 | SF12 | 500 kHz |
| AU915 | 923.3 | DR8 | SF12 | 500 kHz |
| AS923 | 923.2 | DR2 | SF10 | 125 kHz |

---

## 12. Complete Reception Sequence Diagram

```
[DEVICE_SEND fires at time t]
│
│   device.select_channel() → freq
│   find_best_gateway() → best_gw, distance, rssi_est
│   device.radio_state = TX
│   calculate_airtime() → airtime
│   Packet created (device_id, sf, tx_power, bw, freq, rssi_est, t, airtime)
│
│   ┌─────────────────────────────────────────────────────────┐
│   │           GATEWAY.process_uplink(packet)                │
│   │                                                         │
│   │   [dl_busy_until[freq] > t?] ─Yes─→ collided=True      │
│   │                                                         │
│   │   try_allocate_path() → [8 paths busy?] ─Yes─→ collided │
│   │   path[k].assign(packet, t+airtime)                     │
│   │                                                         │
│   │   distance = 3D Euclidean(device, gw)                   │
│   │   PL = pathloss(distance, freq, model, x, y)            │
│   │   BPL = get_building_penetration(device)                │
│   │   packet.rssi = Ptx + Ged + Ggw - PL - BPL             │
│   │                                                         │
│   │   for each (other_pkt, s, e) in channel.on_air:         │
│   │       if same_freq and overlap:                         │
│   │           other_rssi = compute link budget              │
│   │           I += other_rssi_linear × overlap_ratio        │
│   │   packet.interference_per_sf = {...}                    │
│   │                                                         │
│   │   NF = 10*log10(k*T*BW) + 30 + 6                       │
│   │   SINR = S/(I+N) → packet.sinr                         │
│   │   SNR  = S/N     → packet.snr, device.snr              │
│   │   SIR  = S/I     → packet.sir                          │
│   │                                                         │
│   │   received_packets.append(packet)                       │
│   └─────────────────────────────────────────────────────────┘
│
│   [multi-GW?] → process_uplink_from_all() → MRC
│       packet.snr_mrc = 10*log10(Σ 10^(snri/10))
│       register secondary GWs in device_registry
│
│   channel.add_transmission(packet, t, t+airtime)
│   packet_tracker.add_packet(packet)
│
│   ns.on_uplink_received(packet, best_gw):
│       verify_frame_mic() → packet.mic_valid
│       validate_frame_counter() → reject if replay
│       device_registry.update(packet, gw)
│           status.gateways[gw.gw_id] = {rssi, snr, sinr, time}
│       controller.on_new_packet() → [LinkAdrReq, ...]
│       [confirmed & !collided?] → schedule ACK
│
│   schedule PACKET_TX_END @ t+airtime
│   schedule DEVICE_SEND @ t+Exp(1/λ)
│
[PACKET_TX_END fires at t+airtime]
│
│   device.radio_state = WAIT_RX1
│   channel.evaluate_reception(packet, gw):
│       SNR check → [fail?] → collided=True
│       preamble_lock = t + 6×Tsym
│       filter late interferers
│       for each early interferer:
│           energy_ratio = RSSI_diff + 10*log10(T/T_overlap)
│           [energy_ratio < matrix_threshold?] → collided=True
│   gateway.release_paths(t+airtime)
│   channel.cleanup_expired(t+airtime-10)
│   schedule RX1_WINDOW_OPEN @ t+airtime+1s
│
[RX1_WINDOW_OPEN fires at t+airtime+1s]
│
│   device.radio_state = RX1
│   [pending DL?] → select_best_for_downlink()
│   [can_send_downlink()?]
│   [!DL-DL interference?]
│   → deliver ACK/MAC: packet.ack_received=True
│      record_downlink(), dl_busy_until, _network_dl_busy
│      device.radio_state = IDLE; return
│
│   [no DL delivered] → schedule RX2_WINDOW_OPEN @ t+airtime+2s
│
[RX2_WINDOW_OPEN fires at t+airtime+2s]
│
│   device.radio_state = RX2
│   rx2_freq = region.rx2_frequency or device._rx2_freq
│   rx2_sf/bw = region.dr_to_sf_bw(rx2_dr) or device._rx2_sf/bw
│   [pending DL?] → same selection + delivery logic as RX1
│
│   device.radio_state = SLEEP → IDLE
│   [packet.collided && retransmit_attempts < 10?]
│       → schedule retransmit with backoff
```

---

## 13. Numerical Examples

### Example 1 — Single GW Reception, No Interference

**Setup:** Device 5, SF7, 14 dBm, 800 m from GW (outdoor), EU868, `log_normal_shadowing`.

```
distance = sqrt(800² + (1.5-30)²) = sqrt(640000 + 812.25) = 800.5 m
PL = 7.7 + 10×3.76×log10(800.5) + N(0, 3.57) = 7.7 + 37.6×2.903 + 1.8 = 119.6 dB
BPL = 0 (outdoor)
packet.rssi = 14 + 0 + 3 - 119.6 = -102.6 dBm

noise_floor (BW=125kHz) = -116.9 dBm
packet.snr = -102.6 - (-116.9) = 14.3 dB
packet.sinr = 10*log10(10^(-10.26) / 10^(-11.69)) = 14.3 dB  (no interference)

SX1301 sensitivity SF7/125kHz = -130.0 dBm
-102.6 > -130.0 → coverage OK

active_paths before = 2/8
path[3] assigned, tx_end = t + 0.0566s
```

### Example 2 — GW Saturation

**Setup:** 9 active transmissions already in progress (SF12, long ToA).

```
active_paths_count = 8/8 (all busy)
try_allocate_path → returns False
saturation_events += 1
packet.collided = True
```

This is a GW-level collision that happens **before** any link budget or interference calculation. Even if the uplink signal is perfect, it cannot be received.

### Example 3 — Multi-GW MRC Recovery

**Setup:** 2 gateways, GW0 at (5000, 5000), GW1 at (9500, 5000). Device at (9000, 5000), SF9.

```
Distance to GW0: 4000 m → PL ≈ 145 dB → RSSI ≈ 14+3-145 = -128 dBm
Distance to GW1:  500 m → PL ≈ 117 dB → RSSI ≈ 14+3-117 = -100 dBm

SNR GW0: -128 - (-117) = -11 dB < snr_min[9]=-12.5 dB → fails single GW
SNR GW1: -100 - (-117) = +17 dB → passes easily

primary gateway = GW1 (best RSSI)
process_uplink runs on GW1

MRC: only GW1 above sensitivity → mrc_receptions = 1 entry
snr_mrc = SNR_GW1 = 17 dB (no combining needed)
```

**Alternate scenario** (device equidistant, both GWs give SNR=−13.5 dB):
```
SNR_MRC = 10×log10(10^(-1.35) + 10^(-1.35))
        = 10×log10(2 × 0.0447)
        = 10×log10(0.0894)
        = −10.5 dB > −12.5 dB → passes after MRC ✓
```

### Example 4 — Downlink Gateway Selection

**Setup:** Device received by 3 GWs with different SNR values.

```
device_registry.gateways = {
    0: {rssi: -120, snr: -3.1},    # GW0: poor SNR
    1: {rssi: -108, snr:  8.9},    # GW1: good SNR
    2: {rssi: -115, snr:  1.9},    # GW2: medium SNR
}

select_best_for_downlink(device_id):
    → best_gw_id = 1 (highest SNR = 8.9 dB)
    → return GatewayStatus[1].gateway
```

GW1 will transmit the downlink ACK/MAC command to the device.

### Example 5 — DL Duty Cycle Check

**Setup:** GW1 sent a DL in RX1 at t=1000s using SF9 (144ms airtime).

```
DL airtime = _calc_dl_airtime(sf=9, bw=125000) = 144ms
record_downlink(gw_id=1, t=1000, airtime=0.144, window="rx1")
→ last_tx_time["rx1"] = 1000 + 0.144 = 1000.144 s

Next device uplink at t=1005s:
can_send_downlink(gw_id=1, t=1005, airtime=0.144, window="rx1"):
    dc_limit = 0.01
    off_time = 0.144/0.01 - 0.144 = 14.4 - 0.144 = 14.256 s
    return 1005 >= 1000.144 + 14.256 = 1014.4 → False → blocked

Next uplink at t=1015s:
    return 1015 >= 1014.4 → True → DL allowed
```

---

## 14. Comparison with ns-3 and FLoRa

| Feature | PyLoRaWAN | ns-3 LoRaWAN | FLoRa |
|---------|----------|-------------|-------|
| **GW hardware model** | SX1301 (8 paths) | SX1301 (8 paths) | Infinite capacity |
| **Path allocation** | First-free of 8 | First-free of 8 | Not modeled |
| **Saturation tracking** | `saturation_events` counter | Logged | N/A |
| **Link budget** | `Ptx + Ged + Ggw - PL - BPL` | Same | `Ptx - PL` |
| **Antenna gains** | Separate ED (0 dBi) + GW (3 dBi) | Yes | Not modeled |
| **3D distance** | Yes (height difference included) | Yes | 2D only |
| **Building penetration** | Lognormal BPL cache (G3) | ITU-R model | Not modeled |
| **SINR computation** | Energy-weighted per-interferer | Same model | Not computed |
| **DL-UL blocking (G8)** | Per-frequency `dl_busy_until` | Yes | Not modeled |
| **DL-DL network block** | `_network_dl_busy` | Yes (backhaul model) | Not modeled |
| **GW DC tracking** | Per window (RX1/RX2) | Per window | Not modeled |
| **RX2 parameters** | Regional + per-device override | Regional only | Fixed (869.525 MHz) |
| **Multi-GW MRC** | Linear SNR sum (Sprint 9) | `LoraInterferenceHelper` | Not modeled |
| **DL GW selection** | Best uplink SNR | Best uplink SNR | Closest GW |
| **Load balancing** | Redirect to least-loaded GW | Not modeled | Not modeled |
| **Backhaul model** | Ideal (in-process call) | Ideal | Ideal |
| **Gateway placement** | Configurable (always grid) | `GridPositionAllocator` | Random / grid |
