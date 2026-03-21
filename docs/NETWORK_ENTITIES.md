# PyLoRaWAN — Network Entities

> Documentação técnica de todas as entidades de rede do simulador.
> Cobre EndDevice, Gateway, Packet, EnergyModel, BatteryModel, Security, Regions, PacketTracker e Network Server.

---

## Sumário

1. [EndDevice](#1-enddevice)
2. [Gateway e ReceptionPath](#2-gateway-e-receptionpath)
3. [Packet](#3-packet)
4. [EnergyModel](#4-energymodel)
5. [BatteryModel e EnergyHarvester](#5-batterymodel-e-energyharvester)
6. [Security — OTAA e MIC](#6-security--otaa-e-mic)
7. [Regional Parameters](#7-regional-parameters)
8. [PacketTracker](#8-packettracker)
9. [Network Server](#9-network-server)
   - 9.1 [NetworkServer (server.py)](#91-networkserver-serverpy)
   - 9.2 [DeviceRegistry e DeviceStatus](#92-deviceregistry-e-devicestatus)
   - 9.3 [GatewayManager e GatewayStatus](#93-gatewaymanager-e-gatewaystatus)
   - 9.4 [DownlinkScheduler e DownlinkPacket](#94-downlinkscheduler-e-downlinkpacket)
   - 9.5 [ADRComponent](#95-adrcomponent)
   - 9.6 [NetworkController](#96-networkcontroller)
10. [Relacionamentos entre Entidades](#10-relacionamentos-entre-entidades)

---

## 1. EndDevice

**Arquivo:** `enddevice.py`

Representa um nó terminal LoRaWAN com camadas PHY, MAC e gerenciamento de energia. Inclui suporte a OTAA, Classes A/B/C, ADR, LR-FHSS e mobilidade.

### RadioState (Enum MAC — 7 estados)

```python
class RadioState(Enum):
    IDLE    = 0   # aguardando próximo ciclo
    TX      = 1   # transmitindo uplink
    WAIT_RX1 = 2  # standby aguardando janela RX1 (1s)
    RX1     = 3   # escutando janela RX1
    WAIT_RX2 = 4  # standby aguardando janela RX2 (2s)
    RX2     = 5   # escutando janela RX2
    SLEEP   = 6   # deep sleep até próximo TX
```

### Atributos de Identidade e Posição

| Atributo       | Tipo    | Descrição                                      |
|----------------|---------|------------------------------------------------|
| `device_id`    | int     | Identificador único                            |
| `x`, `y`       | float   | Posição em metros (0 a `area_size`)            |
| `speed`        | float   | Velocidade de mobilidade (m/s)                 |
| `lorawan_class`| str     | `'A'`, `'B'` ou `'C'` (aleatório)             |
| `traffic_type` | str     | `'periodic'`, `'sporadic'` ou `'critical'`    |
| `is_indoor`    | bool    | 30% chance indoor (afeta building penetration)|
| `activation_mode` | str | `'OTAA'` (padrão) ou `'ABP'`                 |

### Atributos PHY LoRa

| Atributo    | Tipo  | Descrição                                            |
|-------------|-------|------------------------------------------------------|
| `sf`        | int   | Spreading Factor (7–12, aleatório da `sf_range`)    |
| `bw`        | int   | Bandwidth em Hz (aleatório da lista, ex: 125000)    |
| `freq`      | float | Frequência em MHz (aleatória da lista configurada)  |
| `tx_power`  | float | Potência TX em dBm (14 dBm padrão EU868)            |
| `cr`        | int   | Coding Rate — 1=4/5, 2=4/6, 3=4/7, 4=4/8           |
| `pl`        | int   | Payload útil em bytes (20 padrão)                   |
| `airtime`   | float | ToA calculado em segundos (via SX1272 formula)      |

### Atributos de Recepção e Histórico

| Atributo         | Tipo    | Descrição                                   |
|------------------|---------|---------------------------------------------|
| `rssi`           | float   | RSSI do último pacote (dBm)                 |
| `snr`            | float   | SNR do último pacote (dB)                   |
| `snr_history`    | list    | Histórico de SNR para ADR                   |
| `rssi_history`   | list    | Histórico de RSSI                           |
| `coverage_status`| bool    | Está em cobertura do melhor GW              |
| `current_gateway`| Gateway | Gateway atual                               |

### Atributos de Controle MAC

| Atributo                  | Tipo  | Descrição                                    |
|---------------------------|-------|----------------------------------------------|
| `radio_state`             | RadioState | Estado atual da FSM                   |
| `dc_release_time`         | float | Quando duty cycle libera próximo TX          |
| `frame_counter_up`        | int   | Contador de uplinks (incrementado a cada TX) |
| `_last_fc_down`           | int   | Último FC de downlink (replay protection)    |
| `confirmed_ratio`         | float | Fração de mensagens confirmadas (0.3)        |
| `pending_mac_commands`    | list  | MAC commands pendentes para próximo UL       |
| `adr_enabled`             | bool  | ADR habilitado no dispositivo                |
| `adr_ack_cnt`             | int   | Contador backoff ADR                         |
| `_available_channels`     | list  | Canais disponíveis para frequency hopping    |

### Atributos de Janelas RX (configuráveis via MAC commands)

| Atributo         | Padrão | Descrição                                  |
|------------------|--------|--------------------------------------------|
| `_rx1_delay`     | 1.0 s  | Delay da janela RX1                        |
| `_rx2_delay`     | 2.0 s  | Delay da janela RX2                        |
| `_rx1_dr_offset` | 0      | Offset de DR do RX1 relativo ao UL         |
| `_rx2_freq`      | None   | Frequência RX2 (None = default regional)   |
| `_rx2_sf`        | None   | SF do RX2 (None = default regional)        |
| `_rx2_bw`        | None   | BW do RX2 (None = default regional)        |
| `_max_duty_cycle`| None   | Limite de DC definido pelo NS              |

### Atributos OTAA

| Atributo     | Tipo  | Descrição                                        |
|--------------|-------|--------------------------------------------------|
| `app_key`    | bytes | 16 bytes — chave mestra (gerado pelo Network)   |
| `dev_eui`    | bytes | 8 bytes — deterministico do `device_id`         |
| `app_eui`    | bytes | 8 bytes — AppEUI padrão de simulação            |
| `dev_nonce`  | int   | 2 bytes — incrementado a cada JoinRequest       |
| `app_nonce`  | int   | 3 bytes — recebido no JoinAccept do NS          |
| `dev_addr`   | int   | 32-bit — endereço alocado pelo NS               |
| `nwk_skey`   | bytes | 16 bytes — derivado no JoinAccept               |
| `app_skey`   | bytes | 16 bytes — derivado no JoinAccept               |

### Atributos Class B

| Atributo            | Valor  | Descrição                               |
|---------------------|--------|-----------------------------------------|
| `beacon_period`     | 128 s  | Período do beacon padrão LoRaWAN        |
| `ping_period`       | 32 s   | Período de ping slot                    |
| `ping_offset`       | int    | Offset aleatório do ping slot           |
| `is_synchronized`   | bool   | Device sincronizado ao beacon           |
| `last_beacon_time`  | float  | Timestamp do último beacon recebido     |

### Mapeamento FSM → EnergyModel

```python
_ENERGY_STATE_MAP = {
    "IDLE":     "STANDBY",   # 1.4 mA
    "TX":       "TX",        # 28–44 mA
    "WAIT_RX1": "STANDBY",   # 1.4 mA
    "RX1":      "RX",        # 11.2 mA
    "WAIT_RX2": "STANDBY",   # 1.4 mA
    "RX2":      "RX",        # 11.2 mA
    "SLEEP":    "SLEEP",     # 0.0015 mA
}
```

### Métodos Principais

#### `calculate_airtime() → float`
Cálculo de ToA (Time-on-Air) segundo datasheet SX1272:

```python
Tsym = 2^SF / BW
Tpream = (8 + 4.25) * Tsym
DE = 1 if Tsym > 0.016 else 0         # low data rate optimization
numerator = 8*PL - 4*SF + 28 + 16 - 0  # CRC=1, H=0
payloadSymbNB = 8 + max(ceil(numerator / (4*(SF - 2*DE))) * (CR + 4), 0)
ToA = Tpream + payloadSymbNB * Tsym
```

#### `transition_state(new_state, time) → RadioState`
Transiciona a FSM MAC e registra energia no EnergyModel:
```python
energy_state = _ENERGY_STATE_MAP[new_state.name]
energy_model.transition(EnergyState[energy_state], time)
radio_state = new_state
```

#### `can_receive_downlink(current_time) → bool`
Elegibilidade para receber downlink por classe:
- **Classe A**: apenas em estados `RX1` ou `RX2`
- **Classe B**: durante ping slots — `(time_since_beacon % ping_period) < 30ms`
- **Classe C**: sempre, exceto durante `TX`

#### `prepare_join_request() → dict`
Prepara JoinRequest OTAA:
```python
dev_nonce = (dev_nonce + 1) & 0xFFFF
mic = compute_join_mic(app_key, app_eui, dev_eui, dev_nonce)
return {dev_eui, app_eui, dev_nonce, mic}
```

#### `process_join_accept(app_nonce, net_id, dev_addr) → bool`
Processa JoinAccept e deriva session keys:
```python
nwk_skey, app_skey = derive_session_keys(app_key, app_nonce, net_id, dev_nonce)
dev_addr = dev_addr
frame_counter_up = 0
_last_fc_down = 0
```

#### `validate_frame_counter_down(fc_down) → bool`
Proteção contra replay: rejeita se `fc_down <= _last_fc_down`.

#### `validate_payload_size(region) → int`
Retorna `min(pl, region.get_max_payload(sf, bw))`.

#### `select_channel() → float`
Seleciona canal aleatório de `_available_channels` (frequency hopping).

#### `move(area_size, mobility_enabled, model)`
Random walk: desloca `(dx, dy) = speed * (cos θ, sin θ)` com `θ ~ U(0, 2π)`.

#### `get_device_report() → dict`
Retorna snapshot: `{device_id, sf, freq, snr, rssi, classe, energy_consumed_mJ, indoor, battery_soc, activation}`.

---

## 2. Gateway e ReceptionPath

**Arquivo:** `gateway.py`

### ReceptionPath

Representa um demodulador individual do chip SX1301. O SX1301 possui **8 demoduladores paralelos**, permitindo até 8 recepções simultâneas.

```python
class ReceptionPath:
    busy: bool        # ocupado ou não
    packet: Packet    # pacote sendo recebido
    tx_end: float     # quando a recepção termina

    def is_free(current_time) → bool:
        return not busy or tx_end <= current_time

    def assign(packet, tx_end):
        busy = True; packet = packet; tx_end = tx_end

    def release(current_time):
        if tx_end <= current_time: busy = False; packet = None
```

### Gateway — Atributos

| Atributo             | Tipo   | Descrição                                          |
|----------------------|--------|----------------------------------------------------|
| `gw_id`              | int    | Identificador único                                |
| `x`, `y`             | float  | Posição em metros                                  |
| `network`            | Network| Referência ao orquestrador                        |
| `reception_paths`    | list   | 8 instâncias de `ReceptionPath` (SX1301)          |
| `num_reception_paths`| int    | 8 (padrão SX1301)                                 |
| `saturation_events`  | int    | Vezes que todos os paths estavam ocupados          |
| `received_packets`   | list   | Fila de ULs recebidos (max_capacity=100)           |
| `max_capacity`       | int    | Capacidade máxima da fila (100)                   |
| `assigned_frequencies`| set  | Frequências atribuídas ao GW                      |
| `last_dl_time`       | dict   | `{"rx1": float, "rx2": float}` — último DL enviado|
| `total_dl_sent`      | int    | Total de downlinks enviados                        |
| `dl_busy_until`      | dict   | `{freq → tx_end_time}` — G8: DL-UL interference  |

### Métodos Principais

#### `try_allocate_path(packet, current_time) → bool`
Itera sobre os 8 `ReceptionPath`, aloca o primeiro livre:
```
Se nenhum livre → saturation_events++ → return False
Senão → path.assign(packet, current_time + packet.rectime) → return True
```

#### `release_paths(current_time)`
Chama `path.release(current_time)` em todos os 8 paths — libera os que terminaram.

#### `active_paths_count(current_time) → int`
Conta paths com `is_free() == False`.

#### `process_uplink(packet)` — Cálculo do link budget completo

**Passo 1 — DL-UL interference (G8):**
```python
if dl_busy_until.get(packet.freq, 0) > current_time:
    packet.collided = True; return
```

**Passo 2 — Alocação de path:**
```python
if not try_allocate_path(packet, current_time):
    packet.collided = True; return   # gateway saturado
```

**Passo 3 — Link budget:**
```python
distance = sqrt(dx² + dy² + (ht_m - hr_m)²)
path_loss = network.pathloss(distance, freq, model, dev.x, dev.y)
path_loss += network.get_building_penetration(device)   # G3: indoor
packet.rssi = tx_power + ed_antenna_gain + gw_antenna_gain - path_loss
```

**Passo 4 — Interferência por SF (G14 — energy-based):**
```python
for (other_pkt, tx_start, tx_end) in channel.on_air:
    if other_pkt.freq != packet.freq: continue
    overlap = min(tx_end, pkt.tx_end) - max(tx_start, pkt.tx_start)
    if overlap <= 0: continue

    overlap_ratio = overlap / packet.rectime
    other_rssi_linear = 10^(other_rssi_dBm / 10)
    interference_power_linear += other_rssi_linear * overlap_ratio
    interference_per_sf[other_pkt.sf] += other_rssi_linear * overlap_ratio
```

**Passo 5 — SINR:**
```python
noise_linear = 10^(noise_floor / 10)
signal_linear = 10^(packet.rssi / 10)
SINR = signal_linear / (interference_power_linear + noise_linear)
packet.sinr = 10 * log10(SINR)
packet.snr  = packet.rssi - noise_floor
packet.sir  = packet.rssi - 10*log10(interference_power_linear)  # se I > 0
```

#### `balance_load(packet, device)`
Se fila cheia: redireciona para `find_best_gateway(avoid_congestion=True)`.

#### `dynamic_frequency_assignment()`
Atribui frequência não usada nos pacotes recentes para minimizar interferência.

---

## 3. Packet

**Arquivo:** `packet.py`

Estrutura de dados central que acompanha cada transmissão do início ao fim.

### Atributos

**Identidade e PHY:**

| Atributo        | Tipo   | Inicialização       | Descrição                          |
|-----------------|--------|---------------------|------------------------------------|
| `packet_id`     | UUID   | `uuid.uuid4()`      | Identificador globalmente único    |
| `device_id`     | int    | construtor          | ID do dispositivo origem           |
| `sf`            | int    | construtor          | Spreading Factor                   |
| `tx_power`      | float  | construtor          | Potência TX em dBm                 |
| `bw`            | int    | construtor          | Bandwidth em Hz                    |
| `freq`          | float  | construtor          | Frequência em MHz                  |
| `arrival_time`  | float  | construtor          | Timestamp de início da TX          |
| `rectime`       | float  | construtor          | Duração da TX (ToA em segundos)    |
| `tx_start`      | float  | `= arrival_time`    | Alias de arrival_time              |
| `tx_end`        | float  | `= arrival_time + rectime` | Timestamp de fim da TX    |

**Métricas de Recepção:**

| Atributo           | Tipo   | Inicialização | Descrição                              |
|--------------------|--------|---------------|----------------------------------------|
| `rssi`             | float  | construtor    | RSSI calculado (dBm)                   |
| `snr`              | float  | `None → float`| SNR = RSSI − noise_floor (dB)         |
| `sinr`             | float  | `None → float`| SINR (dB) — calculado no gateway       |
| `sir`              | float  | `None → float`| SIR = RSSI − 10log10(I) (dB)          |
| `snr_mrc`          | float  | `None`        | SNR combinado com MRC (Sprint 9)       |
| `mrc_gw_count`     | int    | `1`           | Número de GWs no MRC                   |
| `noise_floor`      | float  | `None → float`| Ruído térmico (dBm)                    |
| `interference_per_sf`| dict | `{}`          | `{SF → potência linear acumulada}`     |

**Resultado:**

| Atributo          | Tipo  | Padrão | Descrição                                     |
|-------------------|-------|--------|-----------------------------------------------|
| `collided`        | bool  | False  | Colisão detectada por `evaluate_reception()`  |
| `received`        | bool  | True   | Recebido com sucesso                          |
| `packet_type`     | str   | uplink | `"uplink"`, `"downlink"`, `"critical"`, `"ack"`|

**MAC:**

| Atributo            | Tipo  | Padrão | Descrição                                    |
|---------------------|-------|--------|----------------------------------------------|
| `mic`               | bytes | None   | 4 bytes — MIC calculado pelo dispositivo    |
| `mic_valid`         | bool  | None   | Resultado da verificação no NS               |
| `confirmed`         | bool  | False  | Mensagem confirmada (espera ACK)            |
| `ack_received`      | bool  | False  | ACK recebido em RX1 ou RX2                  |
| `is_retransmission` | bool  | False  | É uma retransmissão                         |
| `retry_count`       | int   | 0      | Número de retransmissões tentadas            |
| `frame_counter`     | int   | 0      | FCnt do uplink                               |
| `gateway_id`        | int   | None   | GW que recebeu o UL (para roteamento DL)    |

**LR-FHSS:**

| Atributo           | Tipo | Padrão  | Descrição                              |
|--------------------|------|---------|----------------------------------------|
| `phy_type`         | str  | `"CSS"` | `"CSS"` ou `"LR-FHSS"`               |
| `lrfhss_fragments` | list | None    | Lista de `LRFHSSFragment` se LR-FHSS  |

### `__repr__`
```
<Packet ID=... | Device=N | SF=N | TP: N RSSI=X dBm | SINR=X dB | SNR=X dB | NF=X dBm | Collided=T/F | Type=uplink>
```

---

## 4. EnergyModel

**Arquivo:** `energymodel.py`

Rastreia consumo de energia por estado do rádio, compatível com `ns-3 LoraRadioEnergyModel`.

### RadioState (Enum Energia — 4 estados)

```python
class RadioState(Enum):
    SLEEP   = auto()   # 0.0015 mA
    STANDBY = auto()   # 1.4 mA
    TX      = auto()   # 28–44 mA (por potência)
    RX      = auto()   # 11.2 mA
```

### Correntes por Estado (SX1272)

| Estado    | Corrente    | Tensão | Fonte             |
|-----------|-------------|--------|-------------------|
| SLEEP     | 0.0015 mA   | 3.3 V  | Datasheet SX1272  |
| STANDBY   | 1.4 mA      | 3.3 V  | Datasheet SX1272  |
| RX        | 11.2 mA     | 3.3 V  | Datasheet SX1272  |
| TX (14dBm)| 38.0 mA     | 3.3 V  | Datasheet SX1272  |
| TX (12dBm)| 35.1 mA     | 3.3 V  | Datasheet RN2483  |
| TX (10dBm)| 32.4 mA     | 3.3 V  | Datasheet RN2483  |

### Atributos

| Atributo               | Tipo   | Descrição                                        |
|------------------------|--------|--------------------------------------------------|
| `voltage`              | float  | 3.3 V                                            |
| `energy_consumed`      | float  | Total acumulado em mJ                            |
| `current_state`        | RadioState | Estado atual                               |
| `state_entry_time`     | float  | Timestamp de entrada no estado atual            |
| `_current_tx_power`    | int    | Potência TX ativa (dBm)                         |
| `state_durations`      | dict   | `{RadioState → segundos acumulados}`            |
| `energy_breakdown`     | dict   | `{RadioState → mJ acumulados}`                  |
| `_battery`             | BatteryModel | Referência opcional                       |
| `_sim_time_last_tx_end`| float  | Fim do último ciclo TX (para calcular sleep)    |

### Métodos

#### `transition(new_state, time, tx_power=None) → float`
Debita energia do estado anterior e transiciona:
```
duration = time - state_entry_time
current = tx_current_table[tx_power] se TX, else STATE_CURRENT_MA[state]
energy_mJ = current_mA * voltage_V * duration_s

energy_consumed += energy_mJ
energy_breakdown[current_state] += energy_mJ
state_durations[current_state] += duration

battery.consume(energy_mJ)  # se disponível
current_state = new_state
state_entry_time = time
```

#### `update_energy(device, airtime, sim_time=None)` — Ciclo TX Completo

Simula um ciclo Class A completo:

```
1. SLEEP:    sim_time - _sim_time_last_tx_end  (gap entre TXs)
2. TX:       airtime @ device.tx_power
3. STANDBY:  receive_delay1 = 1 s
4. RX:       trx1 = 1 s
5. STANDBY:  max(0, receive_delay2 - receive_delay1 - trx1)
6. RX:       trx2 = 2 s
7. SLEEP:    transiciona (contabilizado no próximo ciclo)
```

Energia típica por ciclo por SF:

| SF  | Airtime  | Energia TX | Energia RX Windows | Total ciclo |
|-----|----------|------------|---------------------|-------------|
| SF7 | ~36 ms   | ~4.5 mJ    | ~86 mJ              | ~91 mJ      |
| SF9 | ~144 ms  | ~18 mJ     | ~86 mJ              | ~104 mJ     |
| SF12| ~1155 ms | ~144 mJ    | ~86 mJ              | ~230 mJ     |

#### `get_avg_current_ua(total_time_s) → float`
Corrente média em µA: `(energy_consumed_mJ / (voltage * time_s)) * 1000`.

#### `stats() → dict`
```python
{
    "total_energy_mj": float,
    "current_state": str,
    "state_durations_s": {state_name: float},
    "energy_breakdown_mj": {state_name: float},
}
```

---

## 5. BatteryModel e EnergyHarvester

**Arquivo:** `battery.py`

### BatteryModel

Compatível com `ns-3 BasicEnergySource`.

#### Atributos

| Atributo         | Padrão    | Descrição                                  |
|------------------|-----------|--------------------------------------------|
| `capacity_mah`   | 2400 mAh  | Capacidade nominal (2× AA alcalinas)       |
| `voltage`        | 3.3 V     | Tensão nominal                             |
| `capacity_mj`    | calculado | `mAh * V * 3.6` em mJ                    |
| `remaining_mj`   | capacity  | Energia restante em mJ                     |
| `depleted`       | False     | Flag de esgotamento                        |
| `harvester`      | None      | Referência a `EnergyHarvester` se presente|

#### Métodos

| Método                                | Descrição                                         |
|---------------------------------------|---------------------------------------------------|
| `consume(energy_mj) → bool`           | Debita energia; retorna False se esgotada        |
| `harvest(power_mw, duration_s)`       | Adiciona energia (clamped à capacidade)           |
| `soc_percent() → float`               | State of Charge: `(remaining / capacity) * 100`  |
| `remaining_mah() → float`             | Energia restante em mAh                          |
| `estimate_lifetime_days(avg_mj_h)`    | `remaining_mj / avg_consumption_mj * (1/24)`    |
| `stats() → dict`                      | `{capacity, remaining_mah, soc, consumed, depleted}` |

### EnergyHarvester

Diferencial: nenhum dos 4 simuladores comparados (FLoRa, ns-3, LoRaWANSim, LR-FHSS-sim) possui energy harvesting.

#### Atributos

| Atributo         | Descrição                                        |
|------------------|--------------------------------------------------|
| `model`          | `"solar"` ou `"constant"`                       |
| `peak_power_mw`  | Potência pico em mW (ex: 100 mW para solar)     |
| `total_harvested_mj` | Energia total colhida na simulação          |

#### `get_power(time_of_day_hours) → float`

```python
# Modelo solar: perfil senoidal das 6h às 18h
if model == "solar" and 6 <= hour <= 18:
    return peak_power_mw * sin(π * (hour - 6) / 12)

# Modelo constante
if model == "constant":
    return peak_power_mw
```

#### `harvest_energy(battery, sim_time_s, duration_s) → float`
Converte `sim_time_s` para hora do dia, calcula potência instantânea, chama `battery.harvest()`.

---

## 6. Security — OTAA e MIC

**Arquivo:** `security.py`

Implementa LoRaWAN 1.0.x OTAA: derivação de chaves, MIC de JoinRequest e MIC de data frames.

Usa `pycryptodome` (AES-128 real) se disponível; fallback automático para HMAC-SHA256 truncado da stdlib.

### Funções

#### `generate_app_key() → bytes`
`os.urandom(16)` — 16 bytes aleatórios como chave mestra.

#### `generate_dev_eui(device_id) → bytes`
DevEUI determinístico: `struct.pack('>Q', 0x0102030405060000 | (device_id & 0xFFFF))`.

#### `generate_app_eui() → bytes`
AppEUI fixo de simulação: `70:B3:D5:7E:D0:00:00:00`.

#### `derive_session_keys(app_key, app_nonce, net_id, dev_nonce) → (nwk_skey, app_skey)`
Conforme LoRaWAN 1.0.x seção 6.2.5:
```
input_nwk = 0x01 | AppNonce(3B LE) | NetID(3B LE) | DevNonce(2B LE) | 0x00..00
input_app = 0x02 | AppNonce(3B LE) | NetID(3B LE) | DevNonce(2B LE) | 0x00..00

NwkSKey = AES128_ECB(AppKey, input_nwk)
AppSKey = AES128_ECB(AppKey, input_app)
```

#### `compute_join_mic(app_key, app_eui, dev_eui, dev_nonce) → bytes`
```
msg = MHDR(0x00) | AppEUI(8B) | DevEUI(8B) | DevNonce(2B LE)
MIC = AES128_CMAC(AppKey, msg)[0:4]
```

#### `compute_frame_mic(nwk_skey, dev_addr, frame_counter, direction, payload_bytes) → bytes`
```
B0 = 0x49 | 0x00*4 | Dir(1B) | DevAddr(4B LE) | FCnt(4B LE) | 0x00 | len(payload)(1B)
MIC = AES128_CMAC(NwkSKey, B0 | payload)[0:4]
```

#### `verify_frame_mic(...) → bool`
Computa e compara com MIC recebido. Proteção contra replay e adulteração.

---

## 7. Regional Parameters

**Arquivo:** `regions.py`

### Hierarquia de Classes

```
RegionalParameters (base)
├── EU868   — Europa 863–870 MHz
├── US915   — Américas 902–928 MHz
├── AU915   — Austrália 915–928 MHz
└── AS923   — Ásia 923–925 MHz
```

### RegionalParameters (base)

#### Atributos Comuns

| Atributo              | Descrição                                          |
|-----------------------|----------------------------------------------------|
| `name`                | Nome da região                                     |
| `frequency_range`     | Tupla `(min_MHz, max_MHz)`                        |
| `default_channels`    | Lista de frequências padrão                        |
| `additional_channels` | Canais opcionais configuráveis                     |
| `rx2_frequency`       | Frequência da janela RX2 (MHz)                    |
| `rx2_dr`              | Data Rate da janela RX2                            |
| `max_tx_power_dbm`    | Potência TX máxima regulatória                    |
| `duty_cycle`          | Dict `{(f_low, f_high): limite}` — sub-bandas     |
| `max_dwell_time_ms`   | Tempo máximo por transmissão (None = sem limite)  |
| `dr_table`            | Dict `{DR: {sf, bw}}`                             |
| `max_payload_bytes`   | Dict `{DR: max_bytes}`                            |
| `receive_delay1/2`    | 1 s / 2 s                                         |

#### Métodos

| Método                           | Descrição                                    |
|----------------------------------|----------------------------------------------|
| `get_channels(include_additional)`| Lista todos os canais disponíveis           |
| `sf_to_dr(sf, bw) → int`         | Converte SF/BW para índice DR               |
| `dr_to_sf_bw(dr) → (sf, bw)`     | Converte DR para SF/BW                      |
| `get_max_payload(sf, bw) → int`  | Payload máximo em bytes para o DR           |
| `get_duty_cycle_limit(freq) → float` | DC limit da sub-banda da frequência    |
| `check_dwell_time(airtime_s) → bool` | Verifica conformidade com dwell time   |

### Parâmetros por Região

| Região | Frequências default       | RX2 freq    | TX max  | Dwell | Duty cycle |
|--------|---------------------------|-------------|---------|-------|------------|
| EU868  | 868.1, 868.3, 868.5 MHz  | 869.525 MHz | 16 dBm  | N/A   | 1% / 0.1% / 10% |
| US915  | 64 canais (902.3–914.9)  | 923.3 MHz   | 30 dBm  | 400ms | Sem limite |
| AU915  | 64 canais (915.2–927.8)  | 923.3 MHz   | 30 dBm  | 400ms | Sem limite |
| AS923  | 923.2, 923.4 MHz          | 923.2 MHz   | 16 dBm  | 400ms | 1%         |

### EU868 — Sub-bandas (G7)

```
G/G1: 863.0–868.6 MHz → 1%    (868.1, 868.3, 868.5, 867.x)
G2:   868.7–869.2 MHz → 0.1%
G3:   869.4–869.65 MHz → 10%  (RX2: 869.525 MHz)
G4:   869.7–870.0 MHz → 1%
```

### DR Table EU868

| DR | SF  | BW     | Max payload |
|----|-----|--------|-------------|
| 0  | SF12| 125kHz | 51 bytes    |
| 1  | SF11| 125kHz | 51 bytes    |
| 2  | SF10| 125kHz | 51 bytes    |
| 3  | SF9 | 125kHz | 115 bytes   |
| 4  | SF8 | 125kHz | 222 bytes   |
| 5  | SF7 | 125kHz | 222 bytes   |
| 6  | SF7 | 250kHz | 222 bytes   |

---

## 8. PacketTracker

**Arquivo:** `packettracker.py`

Rastreia todos os pacotes transmitidos, mantendo histórico por dispositivo e estatísticas globais.

### Atributos

| Atributo               | Tipo | Descrição                                      |
|------------------------|------|------------------------------------------------|
| `packets`              | list | Pacotes originais (não retransmissões)         |
| `retransmitted_packets`| list | Pacotes retransmitidos                         |
| `packet_history`       | dict | `{device_id → [Packet, ...]}`                 |
| `total_retransmissions`| int  | Contador global de retransmissões              |
| `unique_packet_count`  | int  | Contador de pacotes únicos                     |

### Métodos

#### `add_packet(packet, is_retransmission=False)`
- Se `is_retransmission=True`: adiciona a `retransmitted_packets`, incrementa contador
- Senão: adiciona a `packets`, incrementa `unique_packet_count`
- Ambos: adiciona ao `packet_history[device_id]`

#### `get_stats() → dict`
```python
{
    "Total Pacotes": int,
    "Colisões": int,
    "Retransmissões": int,
    "Entregues com Sucesso": int,
    "Taxa de Entrega (PDR)": float  # percentual
}
```
PDR = `successful / total * 100`

#### `get_device_stats(device_id) → dict`
Mesmas métricas por dispositivo específico, usando `packet_history[device_id]`.

#### `export_device_log(network, filename="device_log.csv")`
CSV com colunas: `Tempo, Device ID, SF, TX Power, RSSI, SNR, Freq, BW, Collided, Confirmed, Frame Counter`.

---

## 9. Network Server

**Diretório:** `network_server/`

### Estrutura de arquivos

```
network_server/
├── __init__.py
├── server.py          — NetworkServer (ponto de entrada)
├── device_registry.py — DeviceRegistry + DeviceStatus
├── gateway_manager.py — GatewayManager + GatewayStatus
├── scheduler.py       — DownlinkScheduler + DownlinkPacket
├── controller.py      — NetworkController (pipeline de componentes)
└── components/
    ├── adr.py         — ADRComponent (5 políticas)
    ├── duty_cycle.py  — DutyCycleComponent
    ├── link_check.py  — LinkCheckComponent
    ├── dev_status.py  — DevStatusComponent
    └── new_channel.py — NewChannelComponent
```

---

### 9.1 NetworkServer (server.py)

Ponto de entrada do NS. Coordena todos os subsistemas.

#### Atributos

| Atributo          | Tipo                 | Descrição                               |
|-------------------|----------------------|-----------------------------------------|
| `device_registry` | DeviceRegistry       | Estado de todos os devices              |
| `gateway_manager` | GatewayManager       | Gestão de GWs e seleção para DL        |
| `dl_scheduler`    | DownlinkScheduler    | Fila de downlinks por device            |
| `controller`      | NetworkController    | Pipeline de componentes (ADR, etc.)    |
| `adr_component`   | ADRComponent         | ADR server-side                         |
| `dc_component`    | DutyCycleComponent   | Duty cycle enforcement                  |
| `link_check`      | LinkCheckComponent   | LinkCheckReq/Ans                        |
| `dev_status`      | DevStatusComponent   | DevStatus req/ans                       |
| `new_channel`     | NewChannelComponent  | NewChannelReq                           |
| `total_uplinks`   | int                  | ULs processados                        |
| `total_downlinks` | int                  | DLs enviados                           |

#### Métodos Principais

##### `on_join_request(device_id, dev_eui, app_eui, dev_nonce, app_key) → dict`
Processa JoinRequest OTAA completo:
```python
app_nonce = os.urandom(3)          # aleatório
net_id = 0x000001                  # fixo para simulação
dev_addr = os.urandom(4) & 0x01FFFFFF

nwk_skey, app_skey = derive_session_keys(app_key, app_nonce, net_id, dev_nonce)

# Registra com session keys
status.nwk_skey = nwk_skey
status.app_skey = app_skey
status.dev_addr = dev_addr
status.joined = True

return {app_nonce, net_id, dev_addr}
```

##### `on_uplink_received(packet, gateway) → list[MAC_command]`
Pipeline completo de processamento de UL:
```
1. Verifica MIC (se OTAA session keys disponíveis — Sprint 10)
2. Valida frame counter (replay detection)
3. Atualiza DeviceRegistry com RSSI/SNR do GW
4. Executa controller.on_new_packet() → gera MAC commands
5. Se packet.confirmed e não colidiu: agenda ACK (prioridade 0)
6. Agenda MAC commands para DL (prioridade 1)
7. Retorna lista de MAC commands
```

##### `get_downlink(device_id) → DownlinkPacket | None`
Chama `dl_scheduler.get_next(device_id)`, incrementa `total_downlinks`.

---

### 9.2 DeviceRegistry e DeviceStatus

**Arquivo:** `device_registry.py`

#### DeviceStatus — Atributos

| Atributo               | Descrição                                          |
|------------------------|----------------------------------------------------|
| `device_id`            | ID do dispositivo                                  |
| `dev_addr`             | Endereço de rede (hex string ou int)              |
| `joined`               | Device completou OTAA join                         |
| `last_packet`          | Referência ao último `Packet` recebido             |
| `last_seen_time`       | Timestamp do último UL                             |
| `gateways`             | `{gw_id: {rssi, snr, sinr, time}}` — per-GW stats |
| `frame_counter_up`     | Último FCnt UL válido                              |
| `frame_counter_down`   | FCnt DL (incrementado a cada DL)                  |
| `pending_mac_commands` | Comandos aguardando próximo UL                     |
| `needs_ack`            | Device aguarda ACK para mensagem confirmada       |
| `adr_ack_cnt`          | Contador backoff ADR                               |
| `nwk_skey`             | 16 bytes — session key de rede (Sprint 10)        |
| `app_skey`             | 16 bytes — session key de aplicação               |

#### DeviceRegistry — Métodos

| Método                                  | Descrição                                   |
|-----------------------------------------|---------------------------------------------|
| `register(device_id, dev_addr=None)`    | Registra device, retorna `DeviceStatus`    |
| `update(packet, gateway) → DeviceStatus`| Atualiza último pacote e métricas por GW   |
| `validate_frame_counter(id, fc) → bool` | Rejeita fc <= último válido (replay)       |
| `needs_reply(device_id) → bool`         | `needs_ack or pending_mac_commands != []`   |
| `get_best_gateway(device_id)`           | GW com maior SNR no último pacote          |
| `get_gw_count(device_id) → int`         | Número de GWs que receberam último UL      |
| `get_status(device_id) → DeviceStatus` | Lookup por ID                               |

---

### 9.3 GatewayManager e GatewayStatus

**Arquivo:** `gateway_manager.py`

#### GatewayStatus

```python
@dataclass
class GatewayStatus:
    gateway: Gateway
    last_tx_time: dict     # {"rx1": float, "rx2": float}
    total_dl_sent: int
    duty_cycle_usage: float
```

#### GatewayManager — Atributos

| Atributo   | Padrão | Descrição                   |
|------------|--------|-----------------------------|
| `gateways` | dict   | `{gw_id → GatewayStatus}`  |
| `rx1_dc`   | 0.01   | Duty cycle máximo RX1 (1%) |
| `rx2_dc`   | 0.10   | Duty cycle máximo RX2 (10%)|

#### Métodos

##### `can_send_downlink(gw_id, current_time, airtime, window="rx1") → bool`
```python
dc_limit = rx1_dc if window == "rx1" else rx2_dc
off_time = airtime / dc_limit - airtime    # tempo mínimo entre DLs
return current_time >= last_tx_time[window] + off_time
```

Exemplo (airtime=0.05s, DC=1%):
- `off_time = 0.05 / 0.01 - 0.05 = 4.95s`
- Próximo DL possível apenas 4.95s após o anterior.

##### `select_best_for_downlink(device_id, device_registry) → Gateway | None`
Seleciona GW com maior SNR do último pacote do device.

##### `record_downlink(gw_id, current_time, airtime, window)`
Atualiza `last_tx_time[window] = current_time + airtime`.

---

### 9.4 DownlinkScheduler e DownlinkPacket

**Arquivo:** `scheduler.py`

Fila de prioridades (min-heap via `heapq`) para downlinks.

#### Prioridades

| Prioridade | Tipo             | Valor |
|------------|------------------|-------|
| 0          | ACK              | Mais urgente |
| 1          | MAC Command      | Alta |
| 2          | Application Data | Baixa |

#### DownlinkPacket

```python
@dataclass
class DownlinkPacket:
    device_id: int
    payload: bytes     # b'' se ACK puro
    packet_type: str   # "ack", "mac_command", "app_data"
```

#### DownlinkScheduler — Métodos

| Método                                      | Descrição                                     |
|---------------------------------------------|-----------------------------------------------|
| `schedule(device_id, packet, priority=None)`| Insere na fila (auto-detecta prioridade)     |
| `get_next(device_id=None) → (id, pkt)`      | Extrai o de maior prioridade (opcional: por device) |
| `has_pending(device_id=None) → bool`        | Verifica se há DL pendente                   |
| `pending_count() → int`                     | Total na fila                                |
| `stats() → dict`                            | `{scheduled, sent, dropped, pending}`        |

**Heap entry**: `(priority, counter, device_id, packet)` — `counter` garante FIFO entre mesma prioridade.

---

### 9.5 ADRComponent

**Arquivo:** `components/adr.py`

ADR server-side com 5 políticas de métrica SNR. Baseado em `ns-3 AdrComponent`.

#### Constantes

```python
ADR_ACK_LIMIT = 64   # Frames sem DL antes de ativar ADR ACK request
ADR_ACK_DELAY = 32   # Frames adicionais antes de backoff forçado
```

#### Políticas de Métrica SNR

| Método        | Cálculo                               |
|---------------|---------------------------------------|
| `average`     | `mean(snr_history)`                  |
| `maximum`     | `max(snr_history)`                   |
| `minimum`     | `min(snr_history)`                   |
| `percentile`  | `percentile(snr_history, 90)`        |
| `ewma`        | Média exponencial `α=0.3`            |

#### Algoritmo ADR (`on_packet`)

```
1. Acumula SNR no histórico (deque de tamanho ADR_HISTORY_SIZE=20)
2. Aguarda histórico cheio

3. snr_metric = compute_snr_metric(history)
4. required_snr = snr_min_per_sf[SF]   (ex: SF9 → -12.5 dB)
5. margin = snr_metric - required_snr - margin_db  (margin_db=10)
6. n_steps = int(margin / ADR_STEP_DB)  (ADR_STEP_DB=3)

Ajustes:
  Passo 1: margin > 0 → reduz SF (SF-- por step) até SF7
  Passo 2: margem ainda positiva → reduz TX power (-2 dBm por step) até 2 dBm
  Passo 3: margin < 0 → aumenta TX power (+2 dBm por step) até 14 dBm
  Passo 4: margem ainda negativa → aumenta SF até SF12

7. Se houve mudança: retorna [LinkAdrReq(dr=12-new_sf, tx_power=new_tx_power)]
8. Senão: retorna []
```

#### `check_backoff(device_status) → (tx_power, sf) | None`
Aplica backoff ADR quando `adr_ack_cnt >= ADR_ACK_LIMIT + ADR_ACK_DELAY`:
- Aumenta `tx_power` em +2 dBm (até 14)
- Se já em 14 dBm: aumenta `SF` em +1 (até SF12)

---

### 9.6 NetworkController

**Arquivo:** `controller.py`

Pipeline de componentes que processa uplinks em cadeia.

```python
class NetworkController:
    components: list   # [ADRComponent, DutyCycleComponent, LinkCheckComponent, ...]

    def add_component(component): ...

    def on_new_packet(packet, device_status) → list[MAC_command]:
        mac_commands = []
        for component in components:
            cmds = component.on_packet(packet, device_status)
            mac_commands.extend(cmds)
        return mac_commands

    def on_join(device_id, status): ...
```

---

## 10. Relacionamentos entre Entidades

```
Network (orquestrador)
│
├── EndDevice × 50
│     ├── EnergyModel (1:1)
│     │     └── BatteryModel (opcional, 1:1)
│     │           └── EnergyHarvester (opcional, 1:1)
│     └── usa RegionalParameters (via network.region)
│
├── Gateway × N
│     ├── ReceptionPath × 8 (1:8 — SX1301)
│     └── Packet[] (received_packets queue)
│
├── Packet (criado por transmissão)
│     ├── pertence a EndDevice (device_id)
│     └── recebido por Gateway (gateway_id)
│
├── ChannelModel
│     └── on_air: [(Packet, tx_start, tx_end)]
│
├── NetworkServer
│     ├── DeviceRegistry
│     │     └── DeviceStatus × N  (1 por device)
│     ├── GatewayManager
│     │     └── GatewayStatus × N (1 por gateway)
│     ├── DownlinkScheduler
│     │     └── DownlinkPacket (fila prioridade)
│     └── NetworkController
│           ├── ADRComponent
│           ├── DutyCycleComponent
│           ├── LinkCheckComponent
│           ├── DevStatusComponent
│           └── NewChannelComponent
│
└── PacketTracker
      └── packets + retransmitted_packets + packet_history
```

### Ciclo de Vida de uma Transmissão

```
EndDevice.prepare_join_request()
    → NS.on_join_request() → DeviceStatus criado
    → EndDevice.process_join_accept() → NwkSKey/AppSKey/DevAddr configurados

EndDevice seleciona (sf, freq, bw)
    → Packet criado (UUID, PHY params, arrival_time, rectime)
    → Gateway.process_uplink(packet)
         → ReceptionPath alocado
         → link budget: RSSI, interferência, SINR calculados
    → ChannelModel.add_transmission(packet)
    → NS.on_uplink_received(packet)
         → MIC verificado (nwk_skey)
         → DeviceStatus atualizado
         → ADRComponent gera LinkAdrReq
         → DownlinkScheduler.schedule(ACK) se confirmed
    → EventScheduler agenda TX_END

TX_END:
    → ChannelModel.evaluate_reception()  — packet.collided = T/F
    → ReceptionPath liberado
    → EventScheduler agenda RX1_OPEN

RX1_OPEN:
    → EndDevice.transition_state(RX1)
    → DownlinkScheduler.get_next(device_id) → DownlinkPacket
    → GatewayManager.can_send_downlink() → verifica duty cycle
    → Se OK: packet.ack_received = True; _apply_mac_commands()

RX2_OPEN (se sem DL em RX1):
    → Mesma lógica com RX2 params (869.525 MHz, SF12)
    → Se colisão + retransmissions < max:
         → exponential backoff + reschedule DEVICE_SEND

PacketTracker.add_packet(packet)
    → estatísticas atualizadas
```

---

*Documentação gerada a partir do código-fonte — Sprint 7 concluído.*
