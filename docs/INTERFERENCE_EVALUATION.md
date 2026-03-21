# Interference Evaluation

**PyLoRaWAN Simulator — Technical Reference**

---

## Table of Contents

1. [Overview](#1-overview)
2. [Interference Taxonomy](#2-interference-taxonomy)
3. [Interference Matrices](#3-interference-matrices)
4. [Phase 1 — Gateway: Energy Accumulation](#4-phase-1--gateway-energy-accumulation)
5. [Phase 2 — Channel Model: Capture Effect](#5-phase-2--channel-model-capture-effect)
6. [DL-UL Interference (G8)](#6-dl-ul-interference-g8)
7. [Preamble Locking (G13)](#7-preamble-locking-g13)
8. [LR-FHSS Fragment Collision](#8-lr-fhss-fragment-collision)
9. [ACRDA — Successive Interference Cancellation](#9-acrda--successive-interference-cancellation)
10. [Legacy Collision Detection](#10-legacy-collision-detection)
11. [Interaction Between Sources](#11-interaction-between-sources)
12. [Numerical Examples](#12-numerical-examples)
13. [Comparison with ns-3 and FLoRa](#13-comparison-with-ns-3-and-flora)

---

## 1. Overview

Interference evaluation in PyLoRaWAN is a **two-phase, layered pipeline** executed for every CSS uplink:

| Phase | Where | What is decided |
|-------|-------|----------------|
| **Phase 1** | `gateway.process_uplink()` | SINR computed from energy-weighted interference accumulation |
| **Phase 2** | `channel.evaluate_reception()` | Pass/fail based on capture effect + preamble locking |

Both phases run before the `PACKET_TX_END` event delivers a final `packet.collided` value.

Additionally, three orthogonal interference sources are evaluated independently:

- **DL-UL (G8):** downlink blocking uplink reception on the same frequency
- **GW saturation:** all 8 SX1301 demodulators occupied
- **LR-FHSS fragment collisions:** per-channel, per-fragment collision check + ACRDA/SIC

### Files Involved

| File | Class / Function | Role |
|------|-----------------|------|
| `gateway.py` | `Gateway.process_uplink()` | Phase 1: SINR computation |
| `channel.py` | `ChannelModel.evaluate_reception()` | Phase 2: capture effect decision |
| `parametors.py` | `interference_matrix`, `interference_matrix_goursaud` | Threshold tables |
| `lrfhss.py` | `LRFHSS_Channel`, `ACRDA` | LR-FHSS collision + SIC |
| `network.py` | `detect_collisions_and_interference()` | Legacy post-hoc detection |

---

## 2. Interference Taxonomy

### 2.1 Types by Origin

| Type | Cause | Models |
|------|-------|--------|
| **Co-SF** | Two devices transmit same SF, same freq, at the same time | Semtech (+1 dB), Goursaud (+6 dB) |
| **Cross-SF** | Two devices transmit different SFs on same freq with overlap | Semtech matrix (row ≠ col), Goursaud matrix |
| **DL-UL** | GW transmitting downlink on freq while receiving uplink (G8) | Binary block |
| **GW saturation** | > 8 simultaneous receptions (SX1301 limit) | Binary block |
| **LR-FHSS fragment** | Two fragments land on the same OBW channel at same time | Binary per fragment |

### 2.2 Types by Timing Relationship

```
Packet under reception:  [==========TX=========]
                          tx_start             tx_end

Scenario A — Full overlap:
Interferer:              [======OTHER=======]
→ Maximum interference energy

Scenario B — Partial overlap (start):
Interferer:       [=OTHER=]
→ Reduced energy (overlap_ratio < 1), may survive via capture

Scenario C — Partial overlap (after preamble lock):
Interferer:                    [=OTHER=]
                     lock_time↑
→ Filtered out by G13: does not contribute to interference

Scenario D — No overlap:
Interferer: [=OTHER=]
→ Zero contribution
```

### 2.3 Evaluation Order

```
DEVICE_SEND → gateway.process_uplink()
  1. DL-UL block? → collided=True, return immediately
  2. GW path saturation? → collided=True, return immediately
  3. Compute SINR (accumulate interference from channel.on_air)

PACKET_TX_END → channel.evaluate_reception()
  4. SNR < threshold? → collided=True, return False
  5. Preamble-locked interferers filtered
  6. Energy ratio vs matrix threshold? → collided=True/False
```

---

## 3. Interference Matrices

### 3.1 Purpose

The interference matrix defines, for each (target SF, interferer SF) pair, the **minimum signal-to-interference energy ratio** (dB) that allows the target packet to survive. Below this threshold, the target packet is lost (capture fails).

```
matrix[SF_target_idx][SF_interferer_idx] = threshold_db

SF index mapping: idx = 12 - SF
  SF12 → idx 0
  SF11 → idx 1
  SF10 → idx 2
  SF9  → idx 3
  SF8  → idx 4
  SF7  → idx 5
```

### 3.2 Semtech Matrix (`interference_matrix`) — Default

Source: Semtech Application Note AN1200.18 — also used by LoRaWANSim.

```
                  Interferer SF:
Target SF:  SF12  SF11  SF10  SF9   SF8   SF7
  SF12   [  +1,  -23,  -24,  -25,  -25,  -25 ]
  SF11   [ -20,   +1,  -20,  -21,  -22,  -22 ]
  SF10   [ -18,  -17,   +1,  -17,  -18,  -19 ]
  SF9    [ -15,  -14,  -13,   +1,  -13,  -15 ]
  SF8    [ -13,  -13,  -12,  -11,   +1,  -11 ]
  SF7    [  -9,   -9,   -9,   -9,   -8,   +1 ]
```

**Reading example:** `matrix[5][5] = +1` means an SF7 packet survives a co-SF7 interferer if the energy ratio ≥ +1 dB (signal energy must exceed interferer energy by at least 1 dB).

**Key properties:**
- **Co-SF diagonal:** all `+1 dB` — low capture threshold, CSS orthogonality is partial
- **Cross-SF (off-diagonal):** large negative values (−8 to −25 dB) — the signal needs to be barely above the noise to survive cross-SF interference; effectively, different SFs do not interfere with each other unless RSSI differences are extreme

### 3.3 Goursaud Matrix (`interference_matrix_goursaud`) — ns-3 Default

Source: Goursaud & Gorce, "Dedicated networks for IoT: PHY / MAC state of the art and challenges," 2015.

```
                  Interferer SF:
Target SF:  SF12  SF11  SF10  SF9   SF8   SF7
  SF12   [  +6,  -36,  -36,  -36,  -36,  -36 ]
  SF11   [ -29,   +6,  -33,  -33,  -33,  -33 ]
  SF10   [ -28,  -26,   +6,  -30,  -30,  -30 ]
  SF9    [ -27,  -25,  -23,   +6,  -27,  -27 ]
  SF8    [ -24,  -22,  -20,  -19,   +6,  -24 ]
  SF7    [ -20,  -16,  -18,  -19,  -16,   +6 ]
```

**Key difference from Semtech:**
- **Co-SF diagonal: +6 dB** — much stricter. The signal must dominate the interferer by 6 dB (4× power ratio) to survive a co-SF collision. This models the physical reality more conservatively.
- **Cross-SF:** even more negative (−16 to −36 dB) — stronger cross-SF isolation.

### 3.4 Matrix Selection

```python
matrix = (interference_matrix_goursaud
          if parametors.interference_model == "goursaud"
          else interference_matrix)
```

Set via `parametors.py`:
```python
interference_model = "semtech"   # or "goursaud"
```

The selection is read **at runtime** (not cached at import), enabling test overrides via `parametors.interference_model = "goursaud"`.

### 3.5 Co-SF Impact Comparison

| Scenario | Semtech threshold | Goursaud threshold | Signal margin needed |
|----------|------------------|--------------------|---------------------|
| SF7 vs SF7 | +1 dB | +6 dB | +1 dB vs +6 dB |
| SF9 vs SF9 | +1 dB | +6 dB | +1 dB vs +6 dB |
| SF12 vs SF7 | −25 dB | −36 dB | nearly anything survives |
| SF7 vs SF12 | −9 dB | −20 dB | SF7 target: signal needs ≥ −9/−20 dB over SF12 interferer |

---

## 4. Phase 1 — Gateway: Energy Accumulation

**Function:** `Gateway.process_uplink(packet)`

This phase computes the **aggregate interference power** seen at the gateway, using an energy-based model equivalent to ns-3's `LoraInterferenceHelper`.

### 4.1 On-Air Scan

For every packet currently registered in `channel.on_air`:

```python
for (other_pkt, tx_start, tx_end) in self.network.channel.on_air:
    if other_pkt.packet_id == packet.packet_id: continue   # skip self
    if other_pkt.freq != packet.freq: continue             # different channel
    if tx_end <= current_time: continue                    # finished before us
    if tx_start >= current_time + packet.rectime: continue # starts after us
```

Only packets on the **same frequency** with **temporal overlap** contribute.

### 4.2 Overlap Ratio

```python
overlap_start    = max(current_time, tx_start)
overlap_end      = min(current_time + packet.rectime, tx_end)
overlap_duration = overlap_end - overlap_start
overlap_ratio    = overlap_duration / packet.rectime
```

`overlap_ratio` is the fraction of the **target packet's** duration that is overlapped by the interferer. It ranges from just above 0 (brief overlap) to 1.0 (complete overlap).

### 4.3 Interferer RSSI

The interferer's RSSI at this gateway is computed independently:

```python
other_distance = sqrt((other_dev.x - gw.x)² + (other_dev.y - gw.y)² + (ht-hr)²)
other_path_loss = network.pathloss(other_distance, ...)
other_path_loss += network.get_building_penetration(other_device)
other_rssi_dbm  = other_pkt.tx_power + Ged + Ggw - other_path_loss
other_rssi_linear = 10 ** (other_rssi_dbm / 10)
```

### 4.4 Energy-Weighted Accumulation

```python
interference_power_linear += other_rssi_linear * overlap_ratio
interference_per_sf[other_pkt.sf] += other_rssi_linear * overlap_ratio
```

**Physical interpretation:**
```
E_interferer = P_interferer × T_overlap
             = P_interferer × (overlap_ratio × T_packet)

Accumulated interference power ≡ Σ_i [ P_i × overlap_ratio_i ]
                                = Σ_i [ E_i / T_packet ]
```

This is the equivalent constant power that would produce the same energy as all partial interferers. It scales correctly: a 10% overlap contributes 10% of the interferer's power.

**Per-SF tracking:**
```python
packet.interference_per_sf = interference_per_sf
# e.g., {7: 1.2e-12, 9: 3.4e-13} — total interference power per SF in linear scale
```

This is stored on the packet for detailed post-simulation analysis.

### 4.5 SINR Computation

```python
packet.noise_floor = network.calculate_noise_floor(device.bw)
noise_linear  = 10 ** (packet.noise_floor / 10)
signal_linear = 10 ** (packet.rssi / 10)

sinr_linear = signal_linear / (interference_power_linear + noise_linear)
packet.sinr = 10 * log10(sinr_linear)
```

**Full equation:**
```
SINR [dB] = 10 × log₁₀( P_signal / (Σ_i(P_i × r_i) + P_noise) )

where:
  P_signal = 10^(RSSI_signal/10)         [mW linear]
  P_i      = 10^(RSSI_interferer_i/10)  [mW linear]
  r_i      = overlap_ratio_i             [0..1]
  P_noise  = 10^(noise_floor/10)         [mW linear]
```

**Derived metrics also computed:**
```python
packet.snr = packet.rssi - packet.noise_floor   # ignores interference
packet.sir = packet.rssi - 10*log10(Σ P_i×r_i) if interference > 0 else +inf
device.snr = packet.snr                         # updates device state for ADR history
```

---

## 5. Phase 2 — Channel Model: Capture Effect

**Function:** `ChannelModel.evaluate_reception(packet, gateway)`

This phase makes the **binary pass/fail decision** using the interference matrix and preamble locking.

### 5.1 Step 1 — SNR Threshold

```python
noise_floor = network.calculate_noise_floor(packet.bw)
snr = packet.snr_mrc if packet.snr_mrc is not None else (packet.rssi - noise_floor)
packet.snr = snr

snr_required = snr_min_per_sf.get(packet.sf, -20.0)
if snr < snr_required:
    packet.collided = True
    self.total_collisions += 1
    return False
```

SNR minimum thresholds (dB) — from LoRa Design Guide / ns-3 / FLoRa:

| SF | SNR_min (dB) | Physical meaning |
|----|-------------|-----------------|
| 7  | −7.5  | Minimum demodulable signal, 125 kHz BW |
| 8  | −10.0 |  |
| 9  | −12.5 |  |
| 10 | −15.0 |  |
| 11 | −17.5 |  |
| 12 | −20.0 | Maximum sensitivity (longest symbol time) |

**MRC SNR:** If `packet.snr_mrc` is available (multi-gateway combining, Sprint 9), it is used instead of single-GW SNR. This can rescue packets that would fail the single-GW threshold:

```
SNR_MRC = 10 × log₁₀( Σ_i 10^(SNR_i/10) )

Two GWs, each SNR = −13 dB (below −12.5 threshold for SF9):
SNR_MRC = 10 × log₁₀( 10^(-1.3) + 10^(-1.3) )
        = 10 × log₁₀( 2 × 0.0501 )
        = 10 × log₁₀( 0.1002 )
        = 10 × (−1.0)
        = −10.0 dB > −12.5 dB → PASSES
```

---

### 5.2 Step 2 — Preamble Lock Window (G13)

```python
Tsym = (2.0 ** packet.sf) / packet.bw
preamble_lock_time = tx_start + 6 * Tsym
```

Any interferer that **starts at or after** `preamble_lock_time` is excluded:

```python
for (other_pkt, other_start, other_end) in self.on_air:
    ...
    if other_start >= preamble_lock_time:
        continue    # G13: arrived after lock, no effect
```

**Physical basis:** The SX1272 preamble consists of 12.25 symbols. After approximately 6 symbols, the correlator is locked to the target chirp sequence. Late-arriving interferers cannot disrupt preamble synchronization because the receiver has already acquired timing and frequency.

**Lock window by SF:**

| SF | Tsym (ms) | Lock window (6×Tsym ms) | Fraction of ToA |
|----|----------|------------------------|----------------|
| 7  | 1.024    | 6.144                  | 10.9% |
| 8  | 2.048    | 12.288                 | 12.0% |
| 9  | 4.096    | 24.576                 | 13.3% |
| 10 | 8.192    | 49.152                 | 14.9% |
| 11 | 16.384   | 98.304                 | 14.9% |
| 12 | 32.768   | 196.608                | 14.9% |

For SF12 (ToA ≈ 1319 ms), an interferer that starts more than 197 ms after the target's preamble begins has **no effect**, regardless of its power. This gives SF12 significant tolerance against late-arriving interferers.

---

### 5.3 Step 3 — Energy Ratio (G14 Capture Effect)

For each interferer that passes the preamble lock filter:

```python
sf_target_idx = 12 - packet.sf
sf_inter_idx  = 12 - interferer.sf
threshold_db  = matrix[sf_target_idx][sf_inter_idx]

if packet.rssi is not None and interferer.rssi is not None:
    energy_correction_db = 10.0 * math.log10(
        packet_duration / max(overlap_duration, 1e-9))
    energy_ratio_db = (packet.rssi - interferer.rssi) + energy_correction_db

    if energy_ratio_db < threshold_db:
        survived = False
        break
```

**Energy ratio formula:**
```
E_signal     = P_signal     × T_packet
E_interferer = P_interferer × T_overlap

energy_ratio = E_signal / E_interferer
             = (P_signal / P_interferer) × (T_packet / T_overlap)

In dB:
energy_ratio_db = (RSSI_signal − RSSI_interferer) + 10×log₁₀(T_packet / T_overlap)
```

**Why energy instead of power?** A partial interferer deposits less energy into the demodulator's correlator integration window. A 10% temporal overlap contributes only 10% of the interferer's energy, regardless of its instantaneous power. The correction term `10×log₁₀(T_packet/T_overlap)` boosts the effective signal-to-interference ratio accordingly.

**Correction factor table:**

| Overlap % | T_pkt/T_overlap | Correction (dB) |
|-----------|----------------|----------------|
| 100% | 1.00 | 0.0 |
| 75%  | 1.33 | +1.2 |
| 50%  | 2.00 | +3.0 |
| 25%  | 4.00 | +6.0 |
| 10%  | 10.0 | +10.0 |
| 5%   | 20.0 | +13.0 |
| 1%   | 100.0| +20.0 |

### 5.4 Failure Condition

The packet fails if **any single interferer** violates the threshold:

```python
for interferer, overlap_duration, _ in interferers:
    ...
    if energy_ratio_db < threshold_db:
        survived = False
        break   # one failure ends evaluation
```

The `break` reflects the physical reality: a single dominant interferer that destroys demodulation cannot be compensated by other interferers being weak. Multiple interferers **accumulate** in Phase 1 (SINR) but are evaluated **individually** in Phase 2 (capture).

This asymmetry means:
- Phase 1 SINR accounts for aggregate load (important for ADR SNR history)
- Phase 2 capture checks each interferer's energy ratio individually (important for collision decision)

---

## 6. DL-UL Interference (G8)

**Location:** `Gateway.process_uplink()`, checked before any other processing.

```python
if self.dl_busy_until.get(packet.freq, 0) > current_time:
    packet.collided = True
    return
```

### 6.1 Mechanism

The SX1301 gateway chip is half-duplex: it **cannot receive on a frequency while transmitting on that same frequency**. When the NS schedules a downlink:

```python
# In _on_rx1_open() / _on_rx2_open():
best_gw.dl_busy_until[packet.freq] = time + dl_airtime
```

Any uplink arriving while `dl_busy_until[freq] > current_time` is immediately marked as collided.

### 6.2 DL-DL Network-Wide Interference

A second level of DL-UL protection tracks network-wide downlink activity:

```python
# In network.py, also set when DL is delivered:
self._network_dl_busy[packet.freq] = time + dl_airtime
```

If another gateway tries to deliver a downlink on the same frequency while the network-wide timer is active, it is also blocked (preventing two gateways from simultaneously transmitting DL on the same channel to different devices).

### 6.3 Off-time Calculation

Downlink airtime uses the same SX1272 ToA formula but with a minimal 13-byte payload:

```python
def _calc_dl_airtime(self, sf, bw):
    pl = 13  # MHDR + minimum MACPayload (ACK + MAC commands)
    # SX1272 ToA formula with this payload
```

Downlink in RX1 uses the device's SF; RX2 always uses SF12.

| DL window | SF | Approx DL airtime |
|-----------|----|--------------------|
| RX1       | 7  | ~41 ms |
| RX1       | 9  | ~144 ms |
| RX2       | 12 | ~991 ms |

The GW duty cycle (1% for RX1, 10% for RX2) is also checked independently via `GatewayManager.can_send_downlink()`.

---

## 7. Preamble Locking (G13)

Implemented in `channel.evaluate_reception()` as described in Section 5.2. Key additional details:

### 7.1 Lock Point Position

The lock point is placed at `tx_start + 6 × Tsym`. The LoRaWAN preamble is 12.25 symbols long (`8 + 4.25`), so the 6-symbol lock point corresponds to roughly **49% of the preamble duration**:

```
Preamble duration = Tpream = (8 + 4.25) × Tsym = 12.25 × Tsym
Lock time         = 6 × Tsym
Lock / Preamble   = 6 / 12.25 = 48.98%
```

### 7.2 Interaction with G14

G13 filters interferers from the G14 energy ratio calculation. An interferer arriving after the lock time:
- **Is not included** in Phase 2 (capture effect) interferers list
- **Is included** in Phase 1 (SINR accumulation) since it still adds RF energy to the receiver

This is the correct physical behavior: post-lock interference degrades SNR (Phase 1) but cannot cause a preamble synchronization failure (Phase 2).

### 7.3 Edge Case: Interferer Starts Before Lock, Ends Before Packet

```
Packet:          [====================================]
                  tx_start          lock_time   tx_end
Interferer:  [=====]
             other_start < lock_time, other_end < lock_time
```

This interferer is included in the Phase 2 check because `other_start < preamble_lock_time`. Its overlap duration = `other_end - tx_start` — the portion that overlaps the target's early reception window.

---

## 8. LR-FHSS Fragment Collision

### 8.1 Architecture

LR-FHSS uses a completely separate interference model from CSS. The key insight is that packets are **fragmented across many sub-channels** (up to 35 OBW channels), so a collision on one channel does not necessarily destroy the entire packet.

```python
class LRFHSS_Channel:
    active_fragments: list[(LRFHSSFragment, tx_start, tx_end)]
```

### 8.2 Fragment Collision Check

```python
def check_fragment_collisions(self, fragments):
    for frag in fragments:
        for (other, other_start, other_end) in self.active_fragments:
            if other.packet_id == frag.packet_id: continue   # skip own frags
            if other.channel != frag.channel: continue        # different OBW channel
            # Temporal overlap?
            if frag.tx_end <= other_start: continue
            if frag.tx_start >= other_end: continue
            frag.collided = True
            break
```

A fragment collides if:
1. Another fragment from a **different packet** is on the **same OBW channel** (0–34)
2. Their **time intervals overlap** (`tx_start`, `tx_end`)

LR-FHSS does **not** use RSSI or SINR for fragment collision — it is binary. Two fragments on the same channel at the same time always collide (no capture effect for LR-FHSS).

### 8.3 Partial Decoding Evaluation

```python
def evaluate_packet(self, fragments, threshold):
    h_success = sum(1 for f in fragments if f.frag_type == 'header' and not f.collided)
    p_success = sum(1 for f in fragments if f.frag_type == 'payload' and not f.collided)

    success = (h_success >= 1) and (p_success >= threshold)
```

**Decoding condition:**
- At least **1 header** fragment must survive (provides frame synchronization and metadata)
- At least **`threshold`** payload fragments must survive (provides sufficient coding redundancy)

**Threshold by code rate:**

| Code rate | `threshold_div` | `threshold = ceil(n_payloads / threshold_div)` |
|-----------|----------------|----------------------------------------------|
| 1/3 | 3 | Must receive ≥ 1/3 of payloads |
| 1/2 | 2 | Must receive ≥ 1/2 of payloads |
| 2/3 | 3 | Same as 1/3 (same `threshold_div`) |

**Example — code rate 1/3, 20B payload:**
```
n_payloads = ceil((20 + 3) / 2) = ceil(11.5) = 12
threshold  = ceil(12 / 3) = 4

Packet survives if: ≥1 header + ≥4 of 12 payload fragments not collided
```

### 8.4 OBW Hopping Isolation

With 35 OBW channels, the probability that two random fragments land on the same channel is `1/35 ≈ 2.9%` per fragment pair. With 12 payload fragments and `n` competing packets, the collision probability per fragment is approximately:

```
P(collision on one fragment) ≈ 1 - (1 - 1/35)^n_competing_fragments_at_same_time
```

For low to moderate network density, most fragments survive on their unique channels — explaining the high LR-FHSS PDR (≈100%) observed in simulation with 35 OBW channels.

---

## 9. ACRDA — Successive Interference Cancellation

### 9.1 Overview

ACRDA (Asynchronous Coded Random Access with Diversity) applies iterative SIC after the simulation completes. It can recover packets that were lost during the simulation due to fragment collisions.

```python
class ACRDA:
    window_size: int              # temporal window (multiples of TX period)
    decoded_packets: set          # packet_ids successfully decoded
    packet_fragments: dict        # packet_id → list[LRFHSSFragment]
```

### 9.2 `process_window(channel)` — SIC Algorithm

```python
def process_window(self, channel):
    new_recovery = True
    iterations = 0

    while new_recovery:
        new_recovery = False
        iterations += 1

        for packet_id, fragments in list(self.packet_fragments.items()):
            if packet_id in self.decoded_packets: continue

            h_ok = sum(1 for f in fragments if f.frag_type=='header' and not f.collided)
            p_ok = sum(1 for f in fragments if f.frag_type=='payload' and not f.collided)
            n_payloads = sum(1 for f in fragments if f.frag_type=='payload')
            threshold = max(1, n_payloads // 3)

            if h_ok >= 1 and p_ok >= threshold:
                self.decoded_packets.add(packet_id)
                self._cancel_interference(packet_id, channel)
                new_recovery = True   # trigger another iteration

    return len(self.decoded_packets), iterations
```

**SIC iteration:**
1. Find all packets that meet the decoding threshold in their **current state**
2. For each newly decoded packet, call `_cancel_interference()` to remove its contribution
3. Repeat until no new packets can be decoded

### 9.3 `_cancel_interference(packet_id, channel)`

```python
def _cancel_interference(self, packet_id, channel):
    decoded_frags = self.packet_fragments[packet_id]

    for dec_frag in decoded_frags:
        for other_id, other_frags in self.packet_fragments.items():
            if other_id == packet_id or other_id in self.decoded_packets: continue
            for other_frag in other_frags:
                if not other_frag.collided: continue
                if other_frag.channel != dec_frag.channel: continue
                if dec_frag.tx_end <= other_frag.tx_start: continue
                if dec_frag.tx_start >= other_frag.tx_end: continue
                other_frag.collided = False   # interference cancelled
```

Once a packet is decoded (its content is known), its interference on other fragments can be subtracted. The model uses a simplified binary cancellation: if the decoded fragment's channel and time overlap with a collided fragment, the collision flag is cleared.

### 9.4 Convergence

The while loop terminates because:
- `decoded_packets` grows monotonically (only adds)
- `packet_fragments` is finite and fixed
- Eventually no new packets meet the threshold → `new_recovery = False`

**Typical convergence (reference scenario, 20 LR-FHSS devices):**

| Iteration | Newly decoded | Total decoded |
|-----------|--------------|---------------|
| 1 | 14 | 14 |
| 2 | 4 | 18 |
| 3 | 1 | 19 |
| 4 | 0 | 19 (converged) |

### 9.5 ACRDA Registration

```python
# In _on_device_send():
self.acrda.register_packet(str(packet.packet_id), fragments)
```

```python
# After simulate_transmissions():
decoded_sic, sic_iters = self.acrda.process_window(self.lrfhss_channel)
```

The ACRDA runs **once** after all events complete. Packets decoded by SIC that were originally marked `collided=True` during the simulation do **not** have their `packet.collided` flag updated retroactively — this is a known limitation. The `decoded_sic` count is logged but not reflected in `PacketTracker` statistics.

---

## 10. Legacy Collision Detection

**Function:** `Network.detect_collisions_and_interference()`

This O(N²) post-hoc algorithm is retained for validation and debugging. It is **not** the primary interference mechanism — the primary mechanism is the two-phase pipeline described in Sections 4–5.

```python
def detect_collisions_and_interference(self):
    packets = sorted(self.packet_tracker.packets, key=lambda p: p.arrival_time)

    for i, p1 in enumerate(packets):
        for j in range(i + 1, len(packets)):
            p2 = packets[j]
            if p2.arrival_time >= p1.arrival_time + p1.rectime:
                break                          # early exit: sorted → no more overlaps
            if p1.device_id == p2.device_id: continue
            if p1.freq != p2.freq: continue    # only same channel

            # Overlap check
            overlap_start = max(p1.arrival_time, p2.arrival_time)
            overlap_end   = min(p1.tx_end, p2.tx_end)
            if overlap_end <= overlap_start: continue

            # Apply interference matrix (RSSI difference, not energy ratio)
            sf1_idx = 12 - p1.sf
            sf2_idx = 12 - p2.sf
            if p1.rssi - p2.rssi < interference_matrix[sf1_idx][sf2_idx]:
                p1.collided = True
            if p2.rssi - p1.rssi < interference_matrix[sf2_idx][sf1_idx]:
                p2.collided = True
```

### Differences vs Primary Model

| Feature | Primary (Phases 1–2) | Legacy |
|---------|---------------------|--------|
| Interference model | Energy-based (overlap_ratio × power) | Power-only (no overlap weighting) |
| Preamble locking | Yes (G13: 6×Tsym filter) | No |
| MRC SNR | Yes (Sprint 9) | No |
| DL-UL blocking | Yes (G8) | No |
| GW path saturation | Yes (8 paths) | No |
| Timing | Real-time during simulation | Post-hoc after simulation |
| Complexity | O(N × K) per packet | O(N²) total |

The early-break optimization makes the legacy algorithm approximately O(N × K) in practice, where K is the maximum number of concurrent transmissions at time t.

---

## 11. Interaction Between Sources

### 11.1 Evaluation Order and Priority

```
1. DL-UL block (G8)           → immediate collided=True, no further processing
2. GW path saturation         → immediate collided=True, no further processing
3. SNR < snr_min_per_sf       → collided=True in evaluate_reception(), return False
4. Preamble + energy ratio    → collided=True if any interferer passes the threshold test
```

Higher-priority failures (1–2) bypass all subsequent checks — a DL-blocked packet is not evaluated for interference matrix effects.

### 11.2 SINR vs Capture Effect Relationship

Phase 1 (SINR) and Phase 2 (capture) are **complementary, not redundant**:

- **Phase 1 SINR** affects:
  - ADR SNR history (`device.snr = packet.snr`)
  - Analytics (`compute_metrics()` SINR statistics)
  - Does **not** directly determine `packet.collided`

- **Phase 2 capture** determines:
  - `packet.collided = True/False`
  - `channel.total_collisions` / `total_receptions` counters

A packet can have a very low SINR (Phase 1) but still not be marked `collided` if no single interferer's energy ratio exceeds the matrix threshold (Phase 2 passes). Conversely, a packet with moderate SINR can be lost if one dominant co-SF interferer has a high RSSI (energy ratio just below threshold).

### 11.3 Multiple Simultaneous Interferers

In Phase 2, each interferer is evaluated **independently**:

```
Interferer A: energy_ratio = 3.5 dB vs threshold = +1 dB → SURVIVES
Interferer B: energy_ratio = −2.0 dB vs threshold = +1 dB → FAILS
→ packet.collided = True (because of B alone)
```

In Phase 1, the same scenario:
```
P_signal = 100 mW (−10 dBm)
P_A      = 50 mW  (−13 dBm), r_A = 0.8
P_B      = 200 mW (−7 dBm),  r_B = 0.3
P_noise  = 0.2 mW (−17 dBm)

I_total = 50×0.8 + 200×0.3 = 40 + 60 = 100 mW
SINR    = 100 / (100 + 0.2) ≈ 0.999 → ≈ −0.0 dB
```

The low SINR correctly reflects the difficult radio environment, even though Phase 2 correctly identifies B as the specific culprit.

---

## 12. Numerical Examples

### Example 1 — Co-SF Capture (Semtech matrix)

**Setup:** Device A (target, SF9, RSSI=−112 dBm), Device B (interferer, SF9, RSSI=−115 dBm), 70% temporal overlap.

**Phase 1:**
```
P_A = 10^(−112/10) = 6.31e-12 mW
P_B = 10^(−115/10) = 3.16e-12 mW, r=0.70
P_noise (BW=125kHz) ≈ 10^(−117/10) = 2.00e-12 mW

I_total = 3.16e-12 × 0.70 = 2.21e-12 mW
SINR = 6.31e-12 / (2.21e-12 + 2.00e-12) = 6.31 / 4.21 = 1.498
packet.sinr = 10×log10(1.498) = 1.75 dB
```

**Phase 2:**
```
SNR = −112 − (−117) = 5.0 dB > snr_min[9] = −12.5 dB → passes

preamble_lock_time = t + 6×4.096ms = t + 24.6ms
(assume B started 10ms after A → before lock → included as interferer)

overlap_duration = 0.70 × 185.4ms = 129.8ms
energy_correction = 10×log10(185.4/129.8) = 10×log10(1.428) = 1.55 dB
energy_ratio = (−112 − (−115)) + 1.55 = 3 + 1.55 = 4.55 dB

Semtech threshold for (SF9 vs SF9): matrix[3][3] = +1 dB
4.55 dB > 1 dB → survived = True → packet.collided = False ✓
```

---

### Example 2 — Co-SF Capture Failure (Goursaud matrix)

**Same setup, but with Goursaud matrix:**

```
Goursaud threshold for (SF9 vs SF9): matrix[3][3] = +6 dB
4.55 dB < 6 dB → survived = False → packet.collided = True ✗
```

**Takeaway:** The same radio conditions result in a successful reception with the Semtech matrix but a collision with the Goursaud matrix. Goursaud predicts ~30–40% lower PDR for high-density co-SF networks.

---

### Example 3 — Preamble Lock Saves the Packet

**Setup:** SF9, ToA=185.4ms. Target starts at t=100.0s. Interferer (SF9, −109 dBm) starts at t=100.030s.

```
preamble_lock_time = 100.0 + 6×0.004096 = 100.0 + 0.02458 = 100.0246s

Interferer start: 100.030 > 100.0246 → arrives AFTER lock

→ Interferer is filtered out by G13
→ Zero interferers in Phase 2 evaluation
→ packet.collided = False ✓ (only SNR check applies)
```

Without G13, the same scenario with RSSI_target=−112, RSSI_interferer=−109:
```
energy_ratio = (−112 − (−109)) + 10×log10(185.4/155.4) = −3 + 0.76 = −2.24 dB
Semtech threshold for SF9 co-SF = +1 dB
−2.24 dB < 1 dB → would fail → packet.collided = True
```

**G13 benefit quantified:** This scenario shows a packet that is saved by the 24.6ms preamble lock window — without G13, this packet would be counted as a collision.

---

### Example 4 — Cross-SF Non-Interference

**Setup:** Device A (SF9, −112 dBm), Device B (SF7, −105 dBm), 100% overlap.

**Phase 2:**
```
threshold = interference_matrix[3][5]  # SF9 target (idx=3), SF7 interferer (idx=5)
          = −15 dB   (Semtech)

energy_correction = 10×log10(185.4/185.4) = 0 dB  (100% overlap)
energy_ratio = (−112 − (−105)) + 0 = −7 dB

−7 dB > −15 dB → survived = True ✓
```

Even though the SF7 interferer is 7 dB stronger, the cross-SF threshold is −15 dB — the target SF9 packet comfortably survives.

**Reverse case** (SF7 target, SF9 interferer, SF7 weaker by 7 dB):
```
threshold = interference_matrix[5][3]  # SF7 target (idx=5), SF9 interferer (idx=3)
          = −9 dB   (Semtech)

energy_ratio = (−112 − (−105)) = −7 dB
−7 dB > −9 dB → survived = True ✓ (barely)
```

If the SF9 interferer were 2 dB stronger: energy_ratio = −9 dB = threshold → exactly at the boundary.

---

### Example 5 — LR-FHSS Collision Analysis

**Setup:** 5 LR-FHSS devices, code rate 1/3, 20B payload, 35 OBW channels.

```
n_payloads = ceil(23/2) = 12
threshold  = ceil(12/3) = 4  (need ≥4 of 12 payload frags to survive)

Each payload fragment has 35 possible channels.
P(collision per fragment | 1 competing packet active on same channel same time) = 1/35 ≈ 2.86%

Expected collisions per payload fragment (4 other packets):
P(collision) ≈ 4/35 ≈ 11.4%

Expected surviving payload frags = 12 × (1 − 0.114) = 12 × 0.886 = 10.6
10.6 >> 4 (threshold) → packet almost certainly decoded ✓

For 20 competing packets:
P(collision per frag) ≈ 20/35 ≈ 57%
Expected surviving = 12 × 0.43 = 5.2
5.2 > 4 (threshold) → still likely decoded, but marginal
```

**LR-FHSS advantage:** 35 OBW channels provide high collision diversity — even with 20 competing devices, most packets are decoded.

---

### Example 6 — ACRDA SIC Recovery

**Setup:** 3 LR-FHSS packets, each with 3 payload fragments, threshold=1.

Before SIC:
```
Packet A: frags on channels [3, 7, 12]     — all intact → decoded directly
Packet B: frags on channels [3, 15, 22]    — ch3 collides with A's frag → 2 intact
Packet C: frags on channels [15, 22, 31]   — ch15+ch22 collide with B → 1 intact (below threshold=1)
                                             ch15 overlaps B; ch22 overlaps B → 0 intact...

Iteration 1: Decode A (3 intact, threshold=1 ✓)
             → _cancel_interference(A): B's ch3 fragment cleared (A is decoded, interference removed)

After cancellation: B now has 3 intact fragments
Iteration 2: Decode B (3 intact, threshold=1 ✓)
             → _cancel_interference(B): C's ch15+ch22 fragments cleared

After cancellation: C now has 3 intact fragments
Iteration 3: Decode C (3 intact, threshold=1 ✓)

Result: All 3 packets decoded in 3 SIC iterations.
```

---

## 13. Comparison with ns-3 and FLoRa

| Feature | PyLoRaWAN | ns-3 LoRaWAN | FLoRa |
|---------|----------|-------------|-------|
| **Interference model** | Energy-based (overlap_ratio × power) | Energy-based (`LoraInterferenceHelper`) | Power-only (binary overlap) |
| **Preamble locking (G13)** | Yes, 6×Tsym | Yes (ns-3 preamble detection) | Not modeled |
| **Co-SF threshold** | +1 dB (Semtech) / +6 dB (Goursaud) | +6 dB (Goursaud default) | +6 dB (Semtech docs) |
| **Cross-SF matrix** | Full 6×6 (both models) | Full 6×6 (Goursaud) | Full 6×6 (Semtech) |
| **Matrix selection** | Runtime configurable | Compile-time | Fixed |
| **DL-UL interference (G8)** | Per-frequency binary block | Yes (half-duplex model) | Not modeled |
| **GW path saturation** | 8 SX1301 paths (hardware accurate) | 8 paths | Infinite capacity |
| **MRC diversity (Sprint 9)** | Linear SNR sum | `LoraInterferenceHelper` | Not modeled |
| **SINR tracking** | Per-packet `p.sinr`, `p.snr`, `p.sir` | Per-packet (log) | PDR only |
| **Per-SF interference** | `packet.interference_per_sf` dict | Not exposed | Not tracked |
| **LR-FHSS collision** | Per-fragment binary, 35 OBW channels | Not available | Not available |
| **ACRDA/SIC** | Iterative post-simulation | Not available | Not available |
| **Legacy O(N²) detection** | Available (validation only) | Not separate | Used as primary |
| **SNR threshold model** | `snr_min_per_sf` (6 values) | Same table | Same table |
