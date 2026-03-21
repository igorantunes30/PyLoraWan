# Gateway Architecture and Multi-Reception Model — PyLoRaWAN

## Visão Geral

Os gateways são modelados como receptores multi-canal capazes de processar múltiplas transmissões simultâneas, replicando a arquitetura de concentradores modernos como o SX1301. Cada gateway contém 8 reception paths independentes que permitem recepção paralela de pacotes com diferentes SFs e frequências. O fluxo de processamento envolve alocação de path, cálculo de RSSI/SNR/SINR, verificação de sensibilidade e encaminhamento ao Network Server. Em redes multi-gateway, o NS seleciona o gateway de melhor qualidade de enlace para o downlink.

---

## 1. Arquitetura SX1301 — Reception Paths

### 1.1 `ReceptionPath` (`gateway.py`)

Cada demodulador do SX1301 é representado por uma instância de `ReceptionPath`:

```python
class ReceptionPath:
    def __init__(self):
        self.busy   = False
        self.packet = None
        self.tx_end = 0

    def is_free(self, current_time):
        return not self.busy or self.tx_end <= current_time

    def assign(self, packet, tx_end):
        self.busy   = True
        self.packet = packet
        self.tx_end = tx_end

    def release(self, current_time):
        if self.tx_end <= current_time:
            self.busy   = False
            self.packet = None
```

O path é considerado livre se `not self.busy` **ou** se o pacote atribuído já terminou (`tx_end ≤ current_time`). Isso permite reutilização imediata ao final da janela de recepção, sem necessidade de evento de liberação explícito.

### 1.2 `Gateway` (`gateway.py`)

```python
class Gateway:
    def __init__(self, gw_id, x, y, network,
                 max_capacity=100, num_reception_paths=8):
        self.reception_paths  = [ReceptionPath() for _ in range(num_reception_paths)]
        self.saturation_events = 0
        self.dl_busy_until    = {}   # freq → tx_end_time (G8: DL-UL interference)
        self.last_dl_time     = {"rx1": 0, "rx2": 0}
        self.total_dl_sent    = 0
```

| Atributo | Valor padrão | Descrição |
|---|---|---|
| `num_reception_paths` | 8 | Demoduladores paralelos (SX1301) |
| `max_capacity` | 100 | Tamanho máximo da fila `received_packets` |
| `saturation_events` | contador | Incrementado quando todos os 8 paths estão ocupados |
| `dl_busy_until` | `{freq: t}` | Bloqueia recepção UL na frequência enquanto GW transmite DL |

---

## 2. Alocação de Reception Path — `try_allocate_path()`

Ao chegar um pacote, o gateway tenta alocar o primeiro path livre:

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

`packet.rectime` é o airtime do pacote (duração total no ar). Se todos os 8 paths estiverem ocupados, o método retorna `False`, o pacote é marcado como colisão (`packet.collided = True`) e `saturation_events` é incrementado.

**Capacidade paralela:** 8 pacotes simultâneos, independente de SF ou frequência — cada path é um demodulador genérico. Não há restrição de um path por canal/SF como em concentradores reais.

---

## 3. Processamento de Uplink — `process_uplink()`

O handler central de recepção em `gateway.py`:

### 3.1 Verificação de Interferência DL→UL (G8)

```python
if self.dl_busy_until.get(packet.freq, 0) > current_time:
    packet.collided = True
    return
```

Se o gateway está transmitindo um downlink na mesma frequência do pacote recebido, o uplink é descartado. O instante de liberação é registrado por frequência em `dl_busy_until` por `network.py` no momento do envio do DL.

### 3.2 Verificação de Saturação

```python
if not self.try_allocate_path(packet, current_time):
    packet.collided = True
    return
```

### 3.3 Cálculo de RSSI

```python
distance = max(sqrt((device.x - gw.x)² + (device.y - gw.y)² +
                    (ht_m - hr_m)²), 1.0)

path_loss  = network.pathloss(distance, device.freq, model_pathloss, device_x, device_y)
path_loss += network.get_building_penetration(device)   # +perda indoor se is_indoor

packet.rssi = device.tx_power + ed_antenna_gain + gw_antenna_gain - path_loss
```

Parâmetros de antena em `parametors.py`:

| Parâmetro | Valor | Descrição |
|---|---|---|
| `gw_antenna_gain` | 3 dBi | Antena omnidirecional gateway |
| `ed_antenna_gain` | 0 dBi | Antena chip end device |
| `ht_m` | 1.5 m | Altura do device |
| `hr_m` | 30 m | Altura do gateway |

### 3.4 Cálculo de SINR e SNR

A interferência acumulada considera todos os pacotes em transmissão na mesma frequência com sobreposição temporal (modelo energy-based G14):

```python
for (other_pkt, tx_start, tx_end) in network.channel.on_air:
    if other_pkt.freq != packet.freq:
        continue
    # Sobreposição temporal
    overlap_duration = min(current_time + packet.rectime, tx_end) \
                     - max(current_time, tx_start)
    overlap_ratio = overlap_duration / packet.rectime

    other_rssi_linear = 10 ** (other_rssi_dbm / 10)
    interference_power_linear += other_rssi_linear * overlap_ratio
```

```python
noise_linear   = 10 ** (packet.noise_floor / 10)
signal_linear  = 10 ** (packet.rssi / 10)

packet.sinr = 10 × log10(signal_linear / (interference_power_linear + noise_linear))
packet.snr  = packet.rssi - packet.noise_floor
```

O ruído térmico (`noise_floor`) é calculado por `network.calculate_noise_floor(device.bw)`. Para BW = 125 kHz, o resultado é −116.95 dBm (ver `RADIO_PROPAGATION_CHANNEL_MODELING.md`).

### 3.5 Armazenamento do Pacote

```python
if len(self.received_packets) >= self.max_capacity:
    self.balance_load(packet, device)
    return

self.received_packets.append(packet)
```

Se a fila atingir `max_capacity`, `balance_load()` tenta redirecionar o pacote para outro gateway com menor carga via `find_best_gateway(avoid_congestion=True)`.

---

## 4. Seleção de Gateway — `find_best_gateway()` (`network.py`)

Antes de cada transmissão, `network.py` identifica o melhor gateway disponível para o device:

```python
def find_best_gateway(self, device=None, avoid_congestion=False):
    available_gateways = []

    for gateway in self.gateways:
        distance = max(sqrt(Δx² + Δy² + Δh²), 1.0)
        path_loss = self.pathloss(distance, device.freq, model_pathloss, device_x, device_y)
        path_loss += self.get_building_penetration(device)
        rssi = device.tx_power + ed_antenna_gain + gw_antenna_gain - path_loss

        gw_sensitivity = gw_sensitivity_table.get(
            (device.sf, device.bw),
            device.set_sensibilidade(device.sf, device.bw)  # fallback SX1272
        )

        if rssi > gw_sensitivity:
            available_gateways.append((gateway, distance, rssi))

    if not available_gateways:
        device.coverage_status = False
        return None, None, None

    if avoid_congestion:
        return min(other_gateways, key=lambda x: len(x[0].received_packets))

    best = max(available_gateways, key=lambda x: x[2])   # máximo RSSI
    device.coverage_status = True
    return best
```

**Critério de cobertura:** o RSSI calculado deve superar a sensibilidade SX1301 do gateway. Se nenhum gateway satisfaz o limiar, `device.coverage_status = False` e a transmissão é adiada.

**Sensibilidade SX1301** (`gw_sensitivity_table` em `parametors.py`):

| SF | BW 125 kHz | BW 250 kHz |
|---|---|---|
| SF7 | −130.0 dBm | −127.0 dBm |
| SF8 | −132.5 dBm | −129.5 dBm |
| SF9 | −135.0 dBm | −132.0 dBm |
| SF10 | −137.5 dBm | −134.5 dBm |
| SF11 | −140.0 dBm | −137.0 dBm |
| SF12 | −142.5 dBm | −139.0 dBm |

A tabela SX1301 é ~6 dB mais sensível que a tabela SX1272 usada para os end devices.

**Seleção do melhor:** entre todos os gateways com RSSI acima do limiar, é retornado o de maior RSSI (`max(..., key=lambda x: x[2])`).

---

## 5. Multi-Gateway Diversity — MRC

Em redes com múltiplos gateways, `network.py` computa a recepção de todos os gateways em alcance via `GatewayManager.process_uplink_from_all()`:

```python
# network.py — _on_device_send()
if not device.use_lrfhss and len(self.gateways) > 1:
    mrc_receptions = self.ns.gateway_manager.process_uplink_from_all(
        packet, self.gateways, self)
    if len(mrc_receptions) > 1:
        snr_linear_sum = sum(10 ** (r['snr'] / 10.0) for r in mrc_receptions)
        packet.snr_mrc = 10.0 * np.log10(snr_linear_sum)
        packet.mrc_gw_count = len(mrc_receptions)
```

`process_uplink_from_all()` (`network_server/gateway_manager.py`) itera todos os gateways, calcula RSSI/SNR independente para cada um e retorna apenas os que superam a sensibilidade:

```python
for gw in gateways:
    rssi  = device.tx_power + ed_antenna_gain + gw_antenna_gain - path_loss
    snr   = rssi - noise_floor
    if rssi >= sensitivity:
        receptions.append({"gateway": gw, "rssi": rssi, "snr": snr, ...})
```

O SNR combinado por MRC (Maximum Ratio Combining) é calculado como:

```
SNR_MRC = 10 × log₁₀(Σᵢ 10^(SNRᵢ/10))
```

O resultado é armazenado em `packet.snr_mrc` e `packet.mrc_gw_count` para uso pelo ADR e pelas métricas de análise.

---

## 6. Seleção de Gateway para Downlink

Após receber um uplink, o NS registra o SNR de todos os gateways que o receberam em `DeviceStatus.gateways`:

```python
dev_status.gateways[gw_id] = {
    'rssi': r['rssi'],
    'snr':  r['snr'],
    'sinr': None,
    'time': time,
}
```

Na janela RX1 ou RX2, `GatewayManager.select_best_for_downlink()` seleciona o gateway de melhor SNR registrado:

```python
def select_best_for_downlink(self, device_id, device_registry):
    for gw_id, gw_info in status.gateways.items():
        snr = gw_info.get("snr") or -999
        if snr > best_snr:
            best_gw_id = gw_id
            best_snr   = snr
    return self.gateways[best_gw_id].gateway
```

---

## 7. Duty Cycle de Downlink no Gateway

O `GatewayManager` aplica duty cycle separado para janelas RX1 e RX2:

```python
def can_send_downlink(self, gw_id, current_time, airtime, window="rx1"):
    dc_limit  = self.rx1_dc if window == "rx1" else self.rx2_dc
    off_time  = airtime / dc_limit - airtime
    last_tx   = gw_status.last_tx_time.get(window, 0)
    return current_time >= last_tx + off_time

def record_downlink(self, gw_id, current_time, airtime, window):
    gw_status.last_tx_time[window] = current_time + airtime
    gw_status.total_dl_sent += 1
```

| Janela | `dc_limit` padrão | Descrição |
|---|---|---|
| RX1 | 1% | Sub-banda G/G1 (868.1–868.5 MHz) |
| RX2 | 10% | Sub-banda G3 (869.525 MHz) |

Se o duty cycle do gateway não permitir o envio, o downlink é adiado para a próxima janela disponível (RX2 ou próximo ciclo).

---

## 8. Fluxo Completo de Recepção

```
_on_device_send(device, t)
  │
  ├── find_best_gateway(device)
  │     ├── Para cada GW: distância 3D → path_loss → RSSI
  │     ├── RSSI > gw_sensitivity_table[sf, bw]? → inclui na lista
  │     └── retorna GW de maior RSSI (ou None → coverage_status=False)
  │
  ├── best_gateway.process_uplink(packet)
  │     ├── dl_busy_until[freq] > t? → packet.collided=True, return
  │     ├── try_allocate_path(packet, t)
  │     │     ├── path livre? → path.assign(packet, t+airtime)
  │     │     └── todos ocupados? → saturation_events++, collided=True, return
  │     ├── RSSI = tx_power + G_ed + G_gw - path_loss
  │     ├── SINR = S / (I_acumulada + N)  [energy-based G14]
  │     ├── SNR  = RSSI - noise_floor
  │     └── received_packets.append(packet)
  │
  ├── [multi-GW] gateway_manager.process_uplink_from_all(packet, gateways)
  │     └── SNR_MRC = 10×log10(Σ 10^(SNRᵢ/10))  → packet.snr_mrc
  │
  └── ns.on_uplink_received(packet, best_gateway)
        └── device_registry.update → gateways[gw_id] = {rssi, snr, time}


_on_rx1_open(device, t)
  │
  └── gateway_manager.select_best_for_downlink(device_id, registry)
        ├── itera registry.gateways → max SNR
        ├── can_send_downlink(gw_id, t, airtime, "rx1")?
        │     dc_limit=1%, off_time = airtime/0.01 - airtime
        ├── Sim → entrega DL, record_downlink(), dl_busy_until[freq] = t+airtime
        └── Não → tenta RX2
```

---

## 9. Parâmetros Configuráveis

| Parâmetro | Arquivo | Default | Efeito |
|---|---|---|---|
| `num_reception_paths` | `Gateway.__init__` | 8 | Demoduladores paralelos SX1301 |
| `max_capacity` | `Gateway.__init__` | 100 | Tamanho da fila `received_packets` |
| `gw_sensitivity_table` | `parametors.py` | SX1301 (−130 a −142.5 dBm) | Limiar de recepção por SF/BW |
| `gw_antenna_gain` | `parametors.py` | 3 dBi | Ganho antena gateway |
| `hr_m` | `parametors.py` | 30 m | Altura do gateway |
| `gw_rx1_dc` | `GatewayManager.__init__` | 1% | Duty cycle DL janela RX1 |
| `gw_rx2_dc` | `GatewayManager.__init__` | 10% | Duty cycle DL janela RX2 |
