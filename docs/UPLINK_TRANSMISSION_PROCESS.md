# Uplink Transmission Process

**PyLoRaWAN Simulator — Technical Reference**

---

## Table of Contents

1. [Overview](#1-overview)
2. [Phase 1 — Pre-Transmission Checks](#2-phase-1--pre-transmission-checks)
3. [Phase 2 — PHY Preparation](#3-phase-2--phy-preparation)
4. [Phase 3 — Packet Construction](#4-phase-3--packet-construction)
5. [Phase 4 — Gateway Processing](#5-phase-4--gateway-processing)
6. [Phase 5 — MRC Multi-Gateway Combining](#6-phase-5--mrc-multi-gateway-combining)
7. [Phase 6 — Channel Model Registration](#7-phase-6--channel-model-registration)
8. [Phase 7 — Network Server Pipeline](#8-phase-7--network-server-pipeline)
9. [Phase 8 — TX_END and Channel Evaluation](#9-phase-8--tx_end-and-channel-evaluation)
10. [LR-FHSS Uplink Path](#10-lr-fhss-uplink-path)
11. [Complete Uplink Sequence Diagram](#11-complete-uplink-sequence-diagram)
12. [Decision Points and Failure Modes](#12-decision-points-and-failure-modes)
13. [Numerical Example — End-to-End](#13-numerical-example--end-to-end)
14. [Comparison with ns-3 and FLoRa](#14-comparison-with-ns-3-and-flora)

---

## 1. Overview

A LoRaWAN uplink transmission in PyLoRaWAN spans **8 simulation phases**, triggered by a `DEVICE_SEND` event and concluding at `TX_END` with a pass/fail decision from the channel model. The same packet object is populated incrementally as it traverses each layer.

### Files Involved

| Phase | File | Key Function |
|-------|------|-------------|
| 1–3 | `network.py` | `_on_device_send()` |
| 4 | `gateway.py` | `process_uplink()` |
| 5 | `network_server/gateway_manager.py` | `process_uplink_from_all()` |
| 6 | `channel.py` | `add_transmission()` |
| 7 | `network_server/server.py` | `on_uplink_received()` |
| 7a | `network_server/controller.py` | `on_new_packet()` |
| 7b | `network_server/components/adr.py` | `on_packet()` |
| 7c | `network_server/device_registry.py` | `update()`, `validate_frame_counter()` |
| 8 | `network.py` | `_on_tx_end()` |
| 8a | `channel.py` | `evaluate_reception()` |

### CSS vs LR-FHSS Paths

The uplink process forks at Phase 3 depending on `device.use_lrfhss`:

```
DEVICE_SEND
    │
    ├─ device.use_lrfhss = False  →  CSS (LoRa) path  ← this document's main focus
    │
    └─ device.use_lrfhss = True   →  LR-FHSS path  ← see Section 10
```

---

## 2. Phase 1 — Pre-Transmission Checks

**Handler:** `Network._on_device_send(device)`
**Triggered by:** `EventType.DEVICE_SEND` event

Before any transmission occurs, a series of guards are evaluated in strict order. Any failed check **defers** the transmission rather than dropping it.

### Check 1: Battery Depletion

```python
if device.battery is not None and device.battery.depleted:
    return  # device silently drops: no reschedule
```

A depleted device stops all activity permanently. No new `DEVICE_SEND` is scheduled — the device is effectively dead for the rest of the simulation.

---

### Check 2: Duty Cycle Constraint

```python
if time < device.dc_release_time:
    delay = device.dc_release_time - time + 0.001
    self.scheduler.schedule(delay, EventType.DEVICE_SEND,
                            self._on_device_send, device)
    return
```

`dc_release_time` is the earliest time the device may transmit again on its current sub-band (set at the end of the previous transmission). The `+0.001 s` margin prevents floating-point edge cases where `time == dc_release_time`.

The retry is scheduled **deterministically** at `dc_release_time`, not as a new Poisson draw.

---

### Check 3: Channel Selection

```python
device.select_channel()
# → device.freq = random.choice(device._available_channels)
```

The device picks a random channel from its list. For EU868 default: `[868.1, 868.3, 868.5]` MHz. This happens **before** the duty cycle check in logic order, but since `dc_release_time` is per-device (not per-channel), a channel hop does not bypass the duty cycle.

---

### Check 4: LinkCheckReq Injection (10%)

```python
if random.random() < 0.10:
    self.ns.link_check.request_link_check(device.device_id)
```

Stochastically injects a `LinkCheckReq` MAC command — the NS will respond with `LinkCheckAns` (SNR margin + GW count) in the next downlink window.

---

### Check 5: Payload Validation

```python
effective_pl = device.validate_payload_size(self.region)
# → min(device.pl, region.get_max_payload(sf, bw))
```

Payload is silently truncated if it exceeds the regional DR limit. EU868 DR0 (SF12/BW125) allows max 51 bytes; DR5 (SF7/BW125) allows 222 bytes.

---

### Check 6: Dwell Time Check

```python
if not device.check_dwell_time(self.region):
    next_delay = np.random.exponential(1.0 / device.lambda_rate)
    self.scheduler.schedule(next_delay, EventType.DEVICE_SEND, ...)
    return
```

Relevant for AS923 and US915 where dwell time ≤ 400 ms is mandated. For EU868, this check always passes. On failure, a new Poisson inter-arrival is scheduled.

---

### Check 7: Coverage Check

```python
best_gateway, _, best_rssi = self.find_best_gateway(device)
if best_gateway is None or not device.coverage_status:
    next_delay = np.random.exponential(1.0 / device.lambda_rate)
    self.scheduler.schedule(next_delay, EventType.DEVICE_SEND, ...)
    return
```

`find_best_gateway()` evaluates the link budget to every gateway and returns the one with the highest RSSI above the SX1301 sensitivity threshold. If none is reachable, the device is considered out-of-coverage and the transmission is deferred.

**Link budget for gateway selection:**
```
distance = sqrt((dev.x - gw.x)² + (dev.y - gw.y)² + (ht - hr)²)
PL = pathloss(distance, freq, model)
BPL = get_building_penetration(device)   # 0 if outdoor
RSSI = tx_power + ed_antenna_gain + gw_antenna_gain - PL - BPL
→ include GW if RSSI > gw_sensitivity_table[(sf, bw)]
```

SX1301 sensitivity (125 kHz): −130.0 dBm (SF7) to −142.5 dBm (SF12) — ~6 dB better than SX1272.

On success:
```python
device.current_gateway = best_gateway
device.rssi = best_rssi
```

---

### Pre-TX Check Summary

| Check | On Fail | Reschedule |
|-------|---------|-----------|
| Battery depleted | Silent drop | None |
| Duty cycle blocked | Defer to `dc_release_time` | Deterministic |
| No coverage | Skip | Poisson `Exp(1/λ)` |
| Dwell time exceeded | Skip | Poisson `Exp(1/λ)` |
| Payload too large | Truncate & continue | — |

---

## 3. Phase 2 — PHY Preparation

### 3.1 FSM Transition to TX

```python
device.radio_state = RadioState.TX
```

The device enters the TX state in the Class A FSM. This blocks RX window eligibility for the duration of the transmission.

### 3.2 Time-on-Air (Airtime) Calculation

**CSS path (SX1272 formula):**

```python
def calculate_airtime(self):
    Tsym = (2.0 ** self.sf) / self.bw
    Tpream = (8 + 4.25) * Tsym
    DE = 1 if Tsym > 0.016 else 0       # Low Data Rate Optimize (SF11/SF12 @ 125kHz)
    H, CRC = 0, 1                        # explicit header, CRC enabled
    numerator   = 8*pl - 4*sf + 28 + 16*CRC - 20*H
    denominator = 4*(sf - 2*DE)
    payloadSymbNB = 8 + max(ceil(numerator / denominator) * (cr + 4), 0)
    return Tpream + payloadSymbNB * Tsym
```

**Airtime reference table (BW=125kHz, PL=20B, CR=4/5):**

| SF | Tsym (ms) | DE | ToA (ms) |
|----|----------|----|---------|
| 7  | 1.024    | 0  | 56.58   |
| 8  | 2.048    | 0  | 102.40  |
| 9  | 4.096    | 0  | 184.32  |
| 10 | 8.192    | 0  | 329.73  |
| 11 | 16.384   | 1  | 659.46  |
| 12 | 32.768   | 1  | 1318.91 |

**LR-FHSS path:**
```python
airtime = self.lrfhss_phy.calculate_toa(device.pl)
```

### 3.3 Energy Model Update

```python
device.energy_model.update_energy(device=device, airtime=airtime, sim_time=time)
```

Charges the full Class A energy cycle: TX + WAIT_RX1 + RX1 + WAIT_RX2 + RX2 + SLEEP. The `sim_time` parameter enables accurate sleep accounting since the last TX.

### 3.4 Duty Cycle Release Time

```python
dc_limit = self.region.get_duty_cycle_limit(device.freq) or (ed_dc_limit_percent / 100.0)
device.dc_release_time = time + airtime / dc_limit
```

**Example — EU868 G1 sub-band (1% DC), SF9:**
```
airtime = 184.32 ms
dc_release_time = t + 0.18432 / 0.01 = t + 18.432 s
```

The device must wait 18.432 s before transmitting again on the G1 sub-band.

---

## 4. Phase 3 — Packet Construction

### 4.1 Frame Counter Increment

```python
device.frame_counter_up += 1
```

FCntUp is incremented before MIC computation, matching LoRaWAN 1.0.x spec section 4.3.3. The frame counter is 32-bit internally but only the lower 16 bits are transmitted over-the-air.

### 4.2 MIC Computation (OTAA only)

```python
if device.nwk_skey is not None and isinstance(device.dev_addr, int):
    _payload_sim = bytes(device.pl)      # zero-filled simulated payload
    _frame_mic = compute_frame_mic(
        device.nwk_skey,
        device.dev_addr,
        device.frame_counter_up,
        0,                               # direction = uplink
        _payload_sim,
    )
```

MIC algorithm (LoRaWAN 1.0.x spec 4.4):
```
B0 block = 0x49 | 0x00000000 | dir | DevAddr | FCntUp | 0x00 | len(msg)
MIC = AES-128-CMAC(NwkSKey, B0 | MHDR | FHDR | FPort | FRMPayload)[0:4]
```

Devices that joined via ABP (`nwk_skey=None`) skip MIC computation — `packet.mic = None`.

### 4.3 Packet Object Creation

```python
packet = Packet(device.device_id, device.sf, device.tx_power, device.bw,
                device.freq, device.rssi, time, airtime)
packet.frame_counter = device.frame_counter_up
packet.mic = _frame_mic
packet.confirmed = (random.random() < device.confirmed_ratio)  # 30%
device.last_transmissions.append(packet)
```

At this point `packet.rssi` is the **predicted** RSSI from `find_best_gateway()`. The actual RSSI (with full path loss + shadowing + BPL) is recomputed inside `gateway.process_uplink()`.

---

## 5. Phase 4 — Gateway Processing

**Function:** `Gateway.process_uplink(packet)`

### 5.1 DL-UL Interference Check (G8)

```python
if self.dl_busy_until.get(packet.freq, 0) > current_time:
    packet.collided = True
    return
```

The SX1301 is half-duplex per frequency. If the gateway is currently transmitting a downlink on `packet.freq`, the uplink reception is physically impossible. `dl_busy_until[freq]` is set in `_on_rx1_open()` / `_on_rx2_open()` when a DL is scheduled.

---

### 5.2 SX1301 Reception Path Allocation

```python
if not self.try_allocate_path(packet, current_time):
    packet.collided = True   # GW saturated
    return
```

The SX1301 has 8 parallel demodulation paths. If all 8 are occupied by concurrent transmissions, the packet is lost:

```python
def try_allocate_path(self, packet, current_time):
    tx_end = current_time + packet.rectime
    for path in self.reception_paths:
        if path.is_free(current_time):
            path.assign(packet, tx_end)   # mark busy until tx_end
            return True
    self.saturation_events += 1
    return False
```

`ReceptionPath.is_free(t)` returns `True` if `not busy OR tx_end <= t`. This ensures a path is reusable as soon as its current packet finishes transmitting.

---

### 5.3 Link Budget Computation

```python
distance = max(sqrt((dev.x - gw.x)² + (dev.y - gw.y)² + (ht - hr)²), 1.0)
path_loss = network.pathloss(distance, freq, model, device_x=dev.x, device_y=dev.y)
path_loss += network.get_building_penetration(device)   # BPL if indoor
packet.rssi = device.tx_power + ed_antenna_gain + gw_antenna_gain - path_loss
```

**Full link budget:**
```
RSSI [dBm] = Ptx [dBm] + Ged [dBi] + Ggw [dBi] - PL [dB] - BPL [dB]

Default values:
  Ptx = 14 dBm (EU868 max)
  Ged = 0 dBi  (chip antenna)
  Ggw = 3 dBi  (omnidirectional)
  BPL = lognormal(μ=ln(20), σ=0.5) dB if indoor, 0 otherwise
```

Note: This RSSI overwrites the estimate from Phase 1 with the fully computed value including all propagation effects.

---

### 5.4 Interference Accumulation (G14 — Energy-Based)

The gateway iterates all packets currently on-air on the same frequency and accumulates interference energy weighted by temporal overlap:

```python
for (other_pkt, tx_start, tx_end) in self.network.channel.on_air:
    if other_pkt.packet_id == packet.packet_id: continue
    if other_pkt.freq != packet.freq: continue
    # Temporal overlap check
    if tx_end <= current_time or tx_start >= current_time + packet.rectime: continue

    overlap_start = max(current_time, tx_start)
    overlap_end   = min(current_time + packet.rectime, tx_end)
    overlap_duration = overlap_end - overlap_start
    overlap_ratio = overlap_duration / packet.rectime

    # Interferer RSSI at this gateway
    other_path_loss = network.pathloss(other_distance, ...)
    other_rssi_dbm  = other_pkt.tx_power + Ged + Ggw - other_path_loss
    other_rssi_linear = 10 ** (other_rssi_dbm / 10)

    # Weighted interference energy
    interference_power_linear += other_rssi_linear * overlap_ratio
    interference_per_sf[other_pkt.sf] += other_rssi_linear * overlap_ratio
```

**Rationale:** This follows the ns-3 `LoraInterferenceHelper` energy model. A partial overlap contributes only proportionally — a 10% overlap contributes 10% of the interferer's power. This is more accurate than binary overlap detection (which treats a 1 ms overlap over a 2 s packet the same as a full collision).

---

### 5.5 SINR Computation

```python
packet.noise_floor = network.calculate_noise_floor(device.bw)
# noise_floor = 10*log10(k * T * BW) + 30 + NF
# = 10*log10(1.38e-23 * 294.15 * BW) + 30 + 6  [dBm]

noise_linear  = 10 ** (packet.noise_floor / 10)
signal_linear = 10 ** (packet.rssi / 10)

sinr_linear = signal_linear / (interference_power_linear + noise_linear)
packet.sinr = 10 * log10(sinr_linear)

# Derived metrics
packet.snr = packet.rssi - packet.noise_floor        # SNR (no interference)
packet.sir = packet.rssi - 10*log10(interference)    # SIR (no noise)
device.snr = packet.snr                              # update device state
```

**Noise floor reference (BW=125kHz, NF=6dB, T=294.15K):**
```
NF = 10*log10(1.38e-23 × 294.15 × 125000) + 30 + 6
   = 10*log10(5.077e-16) + 30 + 6
   = -152.94 + 30 + 6 = -116.94 dBm ≈ -117 dBm
```

---

### 5.6 Gateway Saturation and Load Balancing

If `len(received_packets) >= max_capacity` (100):

```python
def balance_load(self, packet, device):
    closest_gw, _, _ = network.find_best_gateway(device, avoid_congestion=True)
    if closest_gw:
        closest_gw.received_packets.append(packet)
    else:
        self.received_packets.append(packet)  # local overflow
```

`avoid_congestion=True` excludes the current gateway and selects the one with the fewest `received_packets`. This is a soft load balancing mechanism — the packet's link budget was computed for the original gateway.

---

### Phase 4 Summary

| Step | Action | Failure mode |
|------|--------|-------------|
| DL-UL check | Block if GW transmitting DL on same freq | `packet.collided = True` |
| Path allocation | Assign one of 8 SX1301 demodulators | `packet.collided = True` (saturation) |
| Link budget | Compute RSSI with full propagation + BPL | — |
| Interference accumulation | Energy-weighted sum over overlapping TX | — |
| SINR computation | S / (I + N) | — |
| GW buffer | Append to `received_packets` | Load balance to alt GW |

---

## 6. Phase 5 — MRC Multi-Gateway Combining

**Applicable only when `num_gateways > 1` and CSS path.**

```python
mrc_receptions = self.ns.gateway_manager.process_uplink_from_all(
    packet, self.gateways, self)
```

### 6.1 `process_uplink_from_all()`

For each gateway, independently computes:

```python
distance = sqrt((dev.x - gw.x)² + (dev.y - gw.y)² + (ht - hr)²)
path_loss = network.pathloss(distance, freq, ...)
rssi = tx_power + Ged + Ggw - path_loss
noise_floor = network.calculate_noise_floor(bw)
snr = rssi - noise_floor

if rssi >= sensitivity_table[(sf, bw)]:
    receptions.append({gateway, rssi, snr, distance})
```

Returns list of all gateways that could receive the packet above their sensitivity floor.

### 6.2 MRC SNR Combination

If 2+ gateways receive the packet:

```python
snr_linear_sum = sum(10 ** (r['snr'] / 10.0) for r in mrc_receptions)
packet.snr_mrc = 10.0 * log10(snr_linear_sum)
packet.mrc_gw_count = len(mrc_receptions)
```

**MRC formula:**
```
SNR_MRC = 10 * log10( Σ_i 10^(SNR_i/10) )
```

This is the ideal MRC gain for independent Gaussian noise — equivalent to adding the SNR contributions linearly. With 2 equal-SNR gateways, the gain is `10*log10(2) = 3 dB`.

### 6.3 Device Registry Update for Secondary GWs

```python
for r in mrc_receptions:
    gw_id = r['gateway'].gw_id
    if gw_id != best_gateway.gw_id:
        dev_status.gateways[gw_id] = {
            'rssi': r['rssi'], 'snr': r['snr'],
            'sinr': None, 'time': time
        }
```

The NS now knows all gateways that saw this packet — used later for optimal downlink gateway selection.

---

## 7. Phase 6 — Channel Model Registration

```python
if not device.use_lrfhss:
    self.channel.add_transmission(packet, time, time + airtime)
```

```python
def add_transmission(self, packet, tx_start, tx_end):
    self.on_air.append((packet, tx_start, tx_end))
```

The packet is registered in `channel.on_air` — a list of `(packet, tx_start, tx_end)` tuples. This list is consulted by `gateway.process_uplink()` (Phase 4) for interference accumulation and by `channel.evaluate_reception()` (Phase 8) for capture effect evaluation.

**Timing note:** The packet is added to `on_air` **after** `process_uplink()` has already scanned the list. This means the current packet does not interfere with itself. Packets already on-air when this packet starts will be in the list from their own previous `add_transmission()` calls.

---

## 8. Phase 7 — Network Server Pipeline

### 8.1 PacketTracker

```python
self.packet_tracker.add_packet(packet)
# → packets.append(packet)
# → unique_packet_count += 1
# → packet_history[device_id].append(packet)
```

Packet is registered before NS processing so statistics are always captured, even if the NS rejects it.

---

### 8.2 `NetworkServer.on_uplink_received(packet, gateway)`

#### Step 1: MIC Validation

```python
if packet.mic is not None:
    mic_ok = verify_frame_mic(
        status.nwk_skey, status.dev_addr,
        packet.frame_counter, 0,  # uplink
        payload_sim, packet.mic
    )
    packet.mic_valid = mic_ok
```

Reconstructs the MIC using the NS copy of NwkSKey (derived during OTAA join). A mismatch sets `packet.mic_valid = False` but does **not** drop the packet — it continues through the pipeline. This models the behavior where the simulator wants to observe the packet regardless of security state.

#### Step 2: Frame Counter Validation

```python
if not device_registry.validate_frame_counter(packet.device_id, packet.frame_counter):
    return []    # reject: replay or duplicate
```

```python
def validate_frame_counter(self, device_id, frame_counter):
    if frame_counter <= status.frame_counter_up:
        return False            # replay attack or retransmit with same FC
    status.frame_counter_up = frame_counter
    return True
```

A frame counter that is not strictly greater than the last known value is rejected (anti-replay). This means the NS does not process retransmissions as new uplinks — they arrive with a fresh `frame_counter_up` (incremented in `_on_device_send`), so they pass.

#### Step 3: Device Registry Update

```python
status = device_registry.update(packet, gateway)
```

```python
def update(self, packet, gateway):
    status.last_packet = packet
    status.last_seen_time = packet.arrival_time
    status.gateways[gateway.gw_id] = {
        "rssi": packet.rssi, "snr": packet.snr,
        "sinr": packet.sinr, "time": packet.arrival_time,
    }
    return status
```

Per-gateway RSSI/SNR/SINR records are updated. The `gateways` dict is used for downlink gateway selection (`select_best_for_downlink()` picks the GW with highest SNR).

---

### 8.3 NetworkController Pipeline

```python
mac_commands = self.controller.on_new_packet(packet, status)
```

The `NetworkController` iterates all registered components and collects their MAC command responses:

```python
def on_new_packet(self, packet, device_status):
    mac_commands = []
    for component in self.components:
        cmds = component.on_packet(packet, device_status)
        if cmds:
            mac_commands.extend(cmds)
    return mac_commands
```

**Registered components (in order):**

| Component | File | MAC Command Generated |
|-----------|------|-----------------------|
| `ADRComponent` | `components/adr.py` | `LinkAdrReq` (SF + TX power) |
| `DutyCycleComponent` | `components/duty_cycle.py` | `DutyCycleReq` |
| `LinkCheckComponent` | `components/link_check.py` | `LinkCheckAns` (SNR margin, GW count) |
| `DevStatusComponent` | `components/dev_status.py` | `DevStatusReq` |
| `NewChannelComponent` | `components/new_channel.py` | `NewChannelReq` |

---

### 8.4 ADR Component Deep Dive

The ADR component is the most important NS component for uplink optimization:

```python
def on_packet(self, packet, device_status):
    # 1. Append SNR to history (deque maxlen=20)
    self.device_histories[device_id].append(packet.snr)
    if len(history) < self.history_size:
        return []       # wait for 20 samples

    # 2. Compute SNR metric (method = average/maximum/minimum/percentile/ewma)
    snr_metric = self.compute_snr_metric(history)

    # 3. Compute margin
    required_snr = snr_min_per_sf[packet.sf]   # {12:-20, 11:-17.5, ..., 7:-7.5}
    margin = snr_metric - required_snr - margin_db   # margin_db = 10 dB default

    # 4. Convert to steps (ADR_STEP_DB = 3 dB per step)
    n_steps = int(margin / 3)

    # 5. Apply in priority order:
    # +steps: reduce SF first, then reduce TX power
    while n_steps > 0 and new_sf > 7:   new_sf -= 1; n_steps -= 1
    while n_steps > 0 and new_tx > 2:   new_tx -= 2; n_steps -= 1
    # -steps: increase TX power first, then increase SF
    while n_steps < 0 and new_tx < 14:  new_tx += 2; n_steps += 1
    while n_steps < 0 and new_sf < 12:  new_sf += 1; n_steps += 1

    # 6. Generate LinkAdrReq if change needed
    if new_sf != packet.sf or new_tx != packet.tx_power:
        dr = 12 - new_sf   # DR = 12 - SF for EU868
        return [LinkAdrReq(data_rate=dr, tx_power=new_tx, ch_mask=0xFFFF)]
    return []
```

**SNR metric policies:**

| Method | Formula | Behavior |
|--------|---------|----------|
| `average` | `mean(SNR[-20:])` | Balanced, stable (ns-3 default) |
| `maximum` | `max(SNR[-20:])` | Aggressive SF reduction |
| `minimum` | `min(SNR[-20:])` | Conservative, highest reliability |
| `percentile` | `P90(SNR[-20:])` | Near-maximum, outlier-resistant |
| `ewma` | `α=0.3 EWMA` | Tracks recent trend |

**ADR Backoff:** If a device has not received a downlink for `ADR_ACK_LIMIT=64` frames (device increments `adr_ack_cnt` per TX and NS resets it on DL delivery), the `check_backoff()` method increases TX power by 2 dBm per `ADR_ACK_DELAY=32` extra frames, then SF if at max power.

---

### 8.5 ACK and Downlink Scheduling

```python
# Schedule ACK if confirmed and not collided
if packet.confirmed and not packet.collided:
    ack = DownlinkPacket(packet.device_id, packet_type="ack")
    self.dl_scheduler.schedule(packet.device_id, ack)
    status.needs_ack = True

# Schedule MAC commands as downlink payloads
if mac_commands:
    for cmd in mac_commands:
        mc_pkt = DownlinkPacket(packet.device_id, payload=cmd, packet_type="mac_command")
        self.dl_scheduler.schedule(packet.device_id, mc_pkt)
```

The `DownlinkScheduler` is a priority heap: ACK (priority 0) > MAC command (priority 1) > App data (priority 2). The ACK is only scheduled if the packet was not lost — an undelivered packet cannot be acknowledged.

---

### 8.6 MAC Command Application (Device Side)

```python
mac_commands = self.ns.on_uplink_received(packet, best_gateway)
if mac_commands:
    self._apply_mac_commands(device, mac_commands)
```

```python
def _apply_mac_commands(self, device, mac_commands):
    processor = MACCommandProcessor(network_server=self.ns)
    responses = processor.process_downlink_commands(device, mac_commands)
    if responses:
        device.pending_mac_commands.extend(responses)
```

MAC commands from the NS are applied immediately to the device (simulating the RX window delivery). `process_downlink_commands()` updates device parameters (SF, TX power, duty cycle) and generates response commands (e.g., `LinkAdrAns`) to be sent in the next uplink.

**Note on `server_side_adr`:** If `server_side_adr=True` (default), device-side ADR (`lorawan_mac.process_mac_commands()`) is disabled. Only the NS generates `LinkAdrReq`:

```python
if self.adr_enabled and not self.server_side_adr:
    self.lorawan_mac.process_mac_commands(device)   # legacy mode
```

---

### 8.7 Event Scheduling After Send

```python
# ADR backoff counter
device.adr_ack_cnt += 1

# TX_END fires at airtime from now
self.scheduler.schedule(airtime, EventType.PACKET_TX_END,
                        self._on_tx_end, device, packet)

# Next regular uplink
next_delay = np.random.exponential(1.0 / device.lambda_rate)
self.scheduler.schedule(next_delay, EventType.DEVICE_SEND,
                        self._on_device_send, device)
```

Both events are scheduled simultaneously: the `TX_END` completes the current transmission, and the `DEVICE_SEND` queues the next independent one. The two are independent — the next uplink may fire before RX windows close (for slow-SF devices with high λ), but duty cycle will block it if needed.

---

## 9. Phase 8 — TX_END and Channel Evaluation

**Handler:** `Network._on_tx_end(device, packet)`
**Triggered by:** `EventType.PACKET_TX_END` (scheduled at `t + airtime`)

### 9.1 FSM Transition

```python
device.radio_state = RadioState.WAIT_RX1
```

### 9.2 Channel Reception Evaluation

For CSS packets:
```python
received = self.channel.evaluate_reception(packet, device.current_gateway)
packet.received = received
if not received:
    packet.collided = True
```

#### `ChannelModel.evaluate_reception()` — Step by Step

**Step 1: SNR threshold check**

```python
noise_floor = network.calculate_noise_floor(packet.bw)
snr = packet.snr_mrc if packet.snr_mrc is not None else (packet.rssi - noise_floor)
packet.snr = snr
snr_required = snr_min_per_sf[packet.sf]
if snr < snr_required:
    packet.collided = True
    return False
```

SNR thresholds (dB): SF7=−7.5, SF8=−10, SF9=−12.5, SF10=−15, SF11=−17.5, SF12=−20.

If MRC SNR is available (multi-GW), it is used — this can rescue packets that would fail on single-GW SNR.

---

**Step 2: Preamble lock window (G13)**

```python
Tsym = (2.0 ** packet.sf) / packet.bw
preamble_lock_time = tx_start + 6 * Tsym
```

| SF | Tsym (ms) | Lock window (6×Tsym) |
|----|----------|---------------------|
| 7  | 1.024    | 6.144 ms |
| 8  | 2.048    | 12.288 ms |
| 9  | 4.096    | 24.576 ms |
| 10 | 8.192    | 49.152 ms |
| 11 | 16.384   | 98.304 ms |
| 12 | 32.768   | 196.608 ms |

Any interferer that starts **after** `preamble_lock_time` is filtered out:
```python
if other_start >= preamble_lock_time:
    continue    # late interferer: does not affect demodulation
```

This models the physical behavior of the SX1272 correlator: once the preamble is locked (after 6 symbols), the demodulator is locked to the target signal. Late-arriving interferers cannot disrupt preamble synchronization, though they still affect the payload SINR.

---

**Step 3: Interference matrix selection**

```python
matrix = (interference_matrix_goursaud
          if parametors.interference_model == "goursaud"
          else interference_matrix)
```

| Model | Co-SF threshold | Source |
|-------|----------------|--------|
| Semtech (default) | +1 dB | AN1200.18 |
| Goursaud | +6 dB | ns-3 default |

---

**Step 4: Capture effect — energy ratio (G14)**

For each interferer that arrived before `preamble_lock_time`:

```python
energy_correction_db = 10.0 * log10(packet_duration / max(overlap_duration, 1e-9))
energy_ratio_db = (packet.rssi - interferer.rssi) + energy_correction_db

if energy_ratio_db < threshold_db:
    survived = False
    break    # one failure is enough: packet is lost
```

**Energy ratio formula:**
```
E_signal    = P_signal    × T_packet
E_interferer = P_interferer × T_overlap

energy_ratio = E_signal / E_interferer
             = (P_signal / P_interferer) × (T_packet / T_overlap)

In dB:
energy_ratio_db = (RSSI_signal - RSSI_interferer) + 10*log10(T_packet / T_overlap)
```

**Correction factor interpretation:**

| Overlap fraction | T_pkt/T_overlap | Correction (dB) |
|-----------------|----------------|-----------------|
| 100% | 1.0 | 0 dB |
| 50% | 2.0 | +3 dB |
| 10% | 10.0 | +10 dB |
| 1% | 100.0 | +20 dB |

A 1% temporal overlap requires the interferer to be only 1 dB stronger than the signal MINUS the threshold (≈ −8 dB for co-SF Semtech) to cause a collision — very unlikely for short overlaps.

---

### 9.3 Post-Evaluation Cleanup

```python
device.current_gateway.release_paths(time)
# → for each path: if tx_end <= time: busy=False, packet=None

if not device.use_lrfhss:
    self.channel.cleanup_expired(time - 10.0)
# → on_air = [(p,s,e) for ... if e > time - 10.0]
```

The 10-second window in `cleanup_expired` retains recently finished packets as potential interferers for in-progress transmissions (SF12 ToA < 1.32 s, so 10 s is ample).

---

### 9.4 RX Window Scheduling

```python
self.scheduler.schedule(receive_delay1, EventType.RX1_WINDOW_OPEN,
                        self._on_rx1_open, device, packet)
```

`receive_delay1 = 1.0 s` (EU868 default). The RX1 window opens 1 second after TX_END, regardless of whether the packet was received or lost.

---

## 10. LR-FHSS Uplink Path

When `device.use_lrfhss = True`, the uplink diverges after packet creation:

### Fragment Creation and Registration

```python
packet.phy_type = "LR-FHSS"
fragments = self.lrfhss_phy.create_fragments(str(packet.packet_id), device.pl, time)
packet.lrfhss_fragments = fragments
self.lrfhss_channel.add_fragments(fragments)
self.acrda.register_packet(str(packet.packet_id), fragments)
best_gateway.process_uplink(packet)   # for DL duty cycle tracking only
```

`create_fragments()` produces:
- `n_headers` fragments (default 3) — redundant copies of metadata
- `n_payloads` fragments — payload split across OBW hopping channels

Each fragment hops to a different sub-channel from the 35 OBW slots, distributed pseudo-randomly.

### TX_END Evaluation for LR-FHSS

```python
self.lrfhss_channel.check_fragment_collisions(packet.lrfhss_fragments)
_, n_payloads, threshold = self.lrfhss_phy.fragment_packet(device.pl)
received = self.lrfhss_channel.evaluate_packet(packet.lrfhss_fragments, threshold)
```

`evaluate_packet()` uses partial decoding: the packet is received if at least `threshold` payload fragments survive collision. This is the key advantage of LR-FHSS — even with some fragment collisions, the packet can still be decoded.

### ACRDA Post-Simulation Processing

After `simulate_transmissions()` completes, ACRDA SIC runs:

```python
decoded_sic, sic_iters = self.acrda.process_window(self.lrfhss_channel)
```

Iterative SIC removes decoded packets' interference and re-evaluates previously collided packets. LR-FHSS packets that collided during the simulation may be retroactively decoded.

---

## 11. Complete Uplink Sequence Diagram

```
t=0s  DEVICE_SEND fires
│
├─ [Battery OK?] ─No──→ silent drop
├─ [DC OK?] ──────No──→ defer to dc_release_time (deterministic)
├─ select_channel()     → device.freq = random([868.1, 868.3, 868.5])
├─ [Dwell time OK?] ──No→ defer Exp(1/λ)
├─ [Coverage?] ───────No→ defer Exp(1/λ)
│    └─ find_best_gateway() → RSSI > gw_sensitivity → best_gateway
│
├─ device.radio_state = TX
├─ airtime = calculate_airtime()   (SX1272 formula)
├─ energy_model.update_energy(airtime, t)
├─ dc_release_time = t + airtime/dc_limit
│
├─ frame_counter_up += 1
├─ MIC = compute_frame_mic(nwk_skey, dev_addr, fc, ↑, payload)
├─ Packet created: (device_id, sf, tx_power, bw, freq, rssi, t, airtime)
│
├─ gateway.process_uplink(packet):
│   ├─ [DL busy on freq?] → packet.collided = True; return
│   ├─ try_allocate_path() → [All 8 paths busy?] → collided; return
│   ├─ RSSI = Ptx + Ged + Ggw - PL(dist) - BPL
│   ├─ Σ interference from channel.on_air (energy-weighted overlap)
│   ├─ SINR = S/(I+N), SNR = S/N, SIR = S/I
│   └─ received_packets.append(packet)
│
├─ [multi-GW & CSS?]
│   └─ MRC: snr_mrc = 10*log10(Σ 10^(snri/10))
│
├─ channel.add_transmission(packet, t, t+airtime)
├─ packet_tracker.add_packet(packet)
│
├─ ns.on_uplink_received(packet, gw):
│   ├─ verify_frame_mic() → packet.mic_valid
│   ├─ validate_frame_counter() → reject if replay
│   ├─ device_registry.update(packet, gw)
│   ├─ controller.on_new_packet():
│   │   ├─ ADRComponent → [LinkAdrReq?]
│   │   ├─ DutyCycleComponent → [DutyCycleReq?]
│   │   ├─ LinkCheckComponent → [LinkCheckAns?]
│   │   └─ DevStatusComponent → [DevStatusReq?]
│   ├─ [confirmed & !collided?] → schedule ACK in dl_scheduler
│   └─ schedule MAC commands in dl_scheduler
│
├─ _apply_mac_commands(device, cmds)   (immediate device update)
├─ device.adr_ack_cnt += 1
├─ schedule PACKET_TX_END @ t+airtime
└─ schedule DEVICE_SEND @ t+Exp(1/λ)

t=airtime  PACKET_TX_END fires
│
├─ device.radio_state = WAIT_RX1
├─ channel.evaluate_reception(packet, gw):
│   ├─ [SNR < snr_min_per_sf?] → collided=True; return False
│   ├─ preamble_lock_time = t + 6*Tsym
│   ├─ filter interferers arriving after lock
│   ├─ for each pre-lock interferer:
│   │   energy_ratio = RSSI_diff + 10*log10(T_pkt/T_overlap)
│   │   [energy_ratio < matrix_threshold?] → collided=True; return False
│   └─ return True
├─ gateway.release_paths(t+airtime)
├─ channel.cleanup_expired(t+airtime-10)
└─ schedule RX1_WINDOW_OPEN @ t+airtime+1s

t+1s  RX1_WINDOW_OPEN fires → (downlink process begins)
```

---

## 12. Decision Points and Failure Modes

| Stage | Decision | Failure → |
|-------|---------|-----------|
| Battery | `depleted` | Permanent silence |
| Duty cycle | `t < dc_release_time` | Defer (deterministic) |
| Coverage | `RSSI < sensitivity` | Defer (Poisson) |
| DL-UL (G8) | `dl_busy_until[f] > t` | `collided=True` |
| Path saturation | All 8 SX1301 paths busy | `collided=True` |
| SNR threshold | `SNR < snr_min[SF]` | `collided=True` |
| Preamble + capture | `energy_ratio < matrix[SF_t][SF_i]` | `collided=True` |
| Frame counter | `FC ≤ last_FC_up` | NS rejects (no MAC commands) |

**Collision vs deferred:** A `collided=True` packet is fully processed (consumes a GW path, generates interference for others, is tracked in PacketTracker) but marked as not received. A deferred packet is not transmitted at all in that cycle.

---

## 13. Numerical Example — End-to-End

**Scenario:** Device 7, SF9, 14 dBm, 1500 m from GW, outdoor, EU868, t=125.4 s.

### Phase 1 — Checks
```
dc_release_time = 112.0 s → 125.4 > 112.0 → OK
freq = 868.3 MHz (G1 sub-band)
PL = 7.7 + 10*3.76*log10(1500) + N(0, 3.57) = 7.7 + 37.6*3.176 + 1.2 = 128.3 dB
RSSI_est = 14 + 0 + 3 - 128.3 = -111.3 dBm
gw_sensitivity SF9 = -135.0 dBm → -111.3 > -135.0 → coverage OK
```

### Phase 2 — Airtime
```
Tsym = 512/125000 = 4.096 ms
DE = 0 (Tsym < 16 ms)
Tpream = 12.25 * 4.096 = 50.2 ms
payloadSymbNB = 8 + ceil((160-36+28+16)/36) * 5 = 8 + ceil(4.67)*5 = 8 + 25 = 33
ToA = 50.2 + 33*4.096 = 50.2 + 135.2 = 185.4 ms
dc_release_time = 125.4 + 0.1854/0.01 = 125.4 + 18.54 = 143.94 s
```

### Phase 3 — Packet
```
frame_counter_up = 1254
MIC = compute_frame_mic(nwk_skey, dev_addr=0x1A2B3C4D, 1254, ↑, payload)
     = first 4 bytes of AES-128-CMAC(nwk_skey, B0 | msg)
packet.confirmed = (0.41 < 0.3) = False
```

### Phase 4 — Gateway
```
dl_busy_until[868.3] = 120.0 s < 125.4 s → not blocked
active paths = 3/8 → path 4 assigned
distance = sqrt(1500² + (1.5-30)²) ≈ 1500.3 m
PL = 128.3 dB (same model, same position)
BPL = 0 (outdoor)
packet.rssi = 14 + 0 + 3 - 128.3 = -111.3 dBm
noise_floor = -116.94 dBm
```

One interferer on air: Device 23, SF9, −113.5 dBm, overlap 40% of ToA:
```
overlap_ratio = 0.40
other_rssi_linear = 10^(-113.5/10) = 4.467e-12
interference_linear = 4.467e-12 * 0.40 = 1.787e-12

signal_linear  = 10^(-111.3/10) = 7.413e-12
noise_linear   = 10^(-116.94/10) = 2.023e-12

sinr_linear = 7.413e-12 / (1.787e-12 + 2.023e-12) = 7.413 / 3.810 = 1.946
packet.sinr = 10*log10(1.946) = 2.89 dB
packet.snr  = -111.3 - (-116.94) = 5.64 dB
```

### Phase 8 — Reception Evaluation
```
snr = 5.64 dB > snr_min[9] = -12.5 dB → SNR check PASSES

preamble_lock_time = 125.4 + 6*4.096ms = 125.4 + 0.0246 = 125.425 s

Interferer (Device 23) tx_start = 125.38 s < 125.425 s → arrives before lock → valid interferer

energy_correction = 10*log10(0.1854 / (0.1854*0.40)) = 10*log10(1/0.40) = 3.98 dB
energy_ratio = (-111.3 - (-113.5)) + 3.98 = 2.2 + 3.98 = 6.18 dB

threshold_db = interference_matrix[3][3] = +1 dB   (SF9 target, SF9 interferer, Semtech)

6.18 dB > 1 dB → survived = True → packet.collided = False ✓
```

**Result:** Packet received successfully despite a 40% temporal overlap with a co-SF interferer, because the signal energy dominates by 6.18 dB (threshold = 1 dB).

---

## 14. Comparison with ns-3 and FLoRa

| Feature | PyLoRaWAN | ns-3 LoRaWAN | FLoRa |
|---------|----------|-------------|-------|
| **Pre-TX duty cycle** | Per sub-band (G/G1/G2/G3) | Per channel | Global 1% |
| **Coverage check** | Every TX via `find_best_gateway()` | Continuous (mobility) | Per TX |
| **GW path model** | 8 SX1301 demodulators | 8 paths (LorawanMac) | Not modeled |
| **DL-UL interference (G8)** | Per-frequency `dl_busy_until` | Yes (full duplex disabled) | Not modeled |
| **Link budget** | `Ptx + Ged + Ggw - PL - BPL` | Same (antenna gains) | `Ptx - PL` |
| **Interference model** | Energy-based weighted overlap (G14) | Energy-based (LoraInterferenceHelper) | Semtech matrix (binary) |
| **Preamble locking (G13)** | 6×Tsym lock window | Yes (ns-3 preamble detection) | Not modeled |
| **Capture effect** | Energy ratio vs matrix threshold | Same | RSSI diff vs matrix threshold |
| **Interference matrices** | Semtech + Goursaud (configurable) | Goursaud (default) | Semtech only |
| **MRC** | Linear SNR sum (Sprint 9) | `LoraInterferenceHelper` diversity | Not modeled |
| **NS pipeline** | ADR + DC + LinkCheck + DevStatus | AdrComponent + NetworkController | ADR only |
| **Frame counter validation** | Strict monotone check | Yes | Not implemented |
| **MIC validation** | AES-128-CMAC (OTAA) | Full AES | Not implemented |
| **ADR policies** | 5 (avg/max/min/pctile/ewma) | 1 (average) | 1 (average) |
| **LR-FHSS** | Fragments + ACRDA/SIC | Not available | Not available |
