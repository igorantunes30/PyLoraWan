# MAC and Network Configuration — PyLoRaWAN

## Visão Geral

O cenário de referência utiliza LoRaWAN Class A com ADR server-side habilitado e aplicação de duty cycle por sub-banda EU868. Após cada uplink, o device abre as janelas RX1 e RX2 para receber downlinks do Network Server. O ADR ajusta SF e TX power em passos de 3 dB com base na margem de enlace estimada a partir de um histórico de SNR.

---

## 1. LoRaWAN Class A — Janelas RX1 e RX2

Após o fim de cada transmissão uplink, `network.py` agenda as duas janelas de recepção:

```
TX_END + 1 s → RX1_WINDOW_OPEN  (_on_rx1_open)
TX_END + 2 s → RX2_WINDOW_OPEN  (_on_rx2_open)   [se RX1 não entregou DL]
```

| Parâmetro | Valor | Origem |
|---|---|---|
| `receive_delay1` | 1 s | `parametors.py` |
| `receive_delay2` | 2 s | `parametors.py` |
| `trx1` | 1 s | Duração máxima da janela RX1 |
| `trx2` | 2 s | Duração máxima da janela RX2 |
| RX2 frequência (EU868) | 869.525 MHz | `region.rx2_frequency` |
| RX2 SF (EU868) | SF12 (DR0) | `region.rx2_dr = 0` |

Com os defaults EU868, `delay_to_rx2 = max(0, 2−1−1) = 0 s` — as janelas RX1 e RX2 são consecutivas.

**Seleção de janela para downlink:**

Em `_on_rx1_open()`, o NS verifica a fila de downlinks pendentes:

```python
if self.ns.dl_scheduler.has_pending(device.device_id):
    best_gw = self.ns.gateway_manager.select_best_for_downlink(
        device.device_id, self.ns.device_registry)
    if best_gw and self.ns.gateway_manager.can_send_downlink(best_gw.gw_id, time, dl_airtime, "rx1"):
        # entrega em RX1 — RX2 não é aberta
```

Se a entrega em RX1 falhar (GW em duty cycle ou sem pendência), o device transita para `WAIT_RX2` e a janela RX2 é usada como fallback.

---

## 2. Adaptive Data Rate — Configuração

O ADR server-side é habilitado por `adr_enabled = True` em `parametors.py`. O `ADRComponent` em `network_server/components/adr.py` processa cada uplink recebido:

```python
# Margem de enlace
margin  = snr_metric - snr_min_per_sf[packet.sf] - ADR_SNR_MARGIN_DB
n_steps = int(margin / ADR_STEP_DB)
```

| Parâmetro ADR | Valor | Descrição |
|---|---|---|
| `ADR_HISTORY_SIZE` | 20 pacotes | Mínimo de amostras antes de agir |
| `ADR_SNR_MARGIN_DB` | 10 dB | Margem de segurança sobre SNR mínimo |
| `ADR_STEP_DB` | 3 dB | Tamanho de cada passo de ajuste |
| Método de agregação | `"average"` | Média aritmética do histórico SNR |

**Algoritmo de ajuste em passos de 3 dB:**

```
n_steps > 0 (margem positiva):
  Passo 1: SF -= 1 por step  (SF12 → ... → SF7)
  Passo 2: TX_power -= 2 dBm por step  (14 → ... → 2 dBm)

n_steps < 0 (margem negativa):
  Passo 3: TX_power += 2 dBm por step  (2 → ... → 14 dBm)
  Passo 4: SF += 1 por step  (SF7 → ... → SF12)
```

O ajuste é comunicado ao device via `LinkAdrReq` (CID 0x03) entregue na janela RX1 ou RX2. O device responde com `LinkAdrAns` no próximo uplink.

**SNR mínimo por SF** (`snr_min_per_sf` em `parametors.py`):

| SF | SNR mínimo (dB) |
|---|---|
| SF12 | −20.0 |
| SF11 | −17.5 |
| SF10 | −15.0 |
| SF9 | −12.5 |
| SF8 | −10.0 |
| SF7 | −7.5 |

---

## 3. Duty Cycle — EU868

### 3.1 End Device (Uplink)

O duty cycle do uplink é aplicado per sub-banda em `_on_device_send()`:

```python
dc_limit = self.region.get_duty_cycle_limit(device.freq) or (ed_dc_limit_percent / 100.0)
device.dc_release_time = time + airtime / dc_limit
```

Se `time < device.dc_release_time`, o evento `DEVICE_SEND` é reagendado:

```python
delay = device.dc_release_time - time + 0.001
self.scheduler.schedule(delay, EventType.DEVICE_SEND, ...)
```

**Sub-bandas EU868 e limites de duty cycle:**

| Sub-banda | Faixa (MHz) | Limite | Canais |
|---|---|---|---|
| G / G1 | 863.0–868.6 | 1% | 868.1, 868.3, 868.5, 867.x |
| G2 | 868.7–869.2 | 0.1% | — |
| G3 | 869.4–869.65 | 10% | 869.525 (RX2) |
| G4 | 869.7–870.0 | 1% | — |

Com `ed_dc_limit_percent = 1` e `airtime = 0.185 s` (SF9): off-time mínimo = `0.185 / 0.01 − 0.185 = 18.3 s`.

### 3.2 Gateway (Downlink)

O `GatewayManager` aplica duty cycle separado por janela:

```python
dc_limit  = self.rx1_dc if window == "rx1" else self.rx2_dc
off_time  = airtime / dc_limit - airtime
return current_time >= last_tx_time[window] + off_time
```

| Janela | Limite | Parâmetro |
|---|---|---|
| RX1 | 1% | `gw_rx1_dc_limit_percent = 1` |
| RX2 | 10% | `gw_rx2_dc_limit_percent = 10` |

O limite de 10% em RX2 (sub-banda G3, 869.525 MHz) permite maior frequência de downlinks de fallback.

---

## 4. Interferência DL→UL (G8)

Quando o gateway transmite um downlink, o uplink na mesma frequência é bloqueado durante a transmissão:

```python
# network.py — _on_rx1_open() / _on_rx2_open()
best_gw.dl_busy_until[packet.freq] = time + dl_airtime

# gateway.py — process_uplink()
if self.dl_busy_until.get(packet.freq, 0) > current_time:
    packet.collided = True
    return
```

---

## 5. Interferência entre Pacotes — Matrizes

O modelo de interferência padrão é `"semtech"` (Semtech AN1200.18), selecionável via `interference_model` em `parametors.py`:

| Modelo | Co-SF threshold | Referência |
|---|---|---|
| `"semtech"` | 1 dB | Semtech AN1200.18 / LoRaWANSim |
| `"goursaud"` | 6 dB | Goursaud et al. / ns-3 default |

A matriz é usada em `detect_collisions_and_interference()`:

```python
threshold = interference_matrix[sf1_idx][sf2_idx]
rssi_diff = p1.rssi - p2.rssi
if rssi_diff < threshold:
    p1.collided = True
```

---

## 6. Resumo da Configuração MAC do Cenário de Referência

```python
# parametors.py
adr_enabled             = True
ADR_HISTORY_SIZE        = 20
ADR_SNR_MARGIN_DB       = 10
ADR_STEP_DB             = 3
ed_dc_limit_percent     = 1       # 1% uplink EU868
gw_rx1_dc_limit_percent = 1       # 1% DL RX1
gw_rx2_dc_limit_percent = 10      # 10% DL RX2
receive_delay1          = 1       # s
receive_delay2          = 2       # s
trx1                    = 1       # s
trx2                    = 2       # s
interference_model      = "semtech"

# Network()
Network(...,
    region_name    = "EU868",
    adr_enabled    = True,
    adr_method     = "average",
    server_side_adr = True,
)
```
