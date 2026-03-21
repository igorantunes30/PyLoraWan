# PyLoRaWAN — Core Simulation Engine

> Documentação técnica completa do motor de simulação.
> Baseado no estado atual do código (Sprint 7 concluído, Sprint 8 planejado).

---

## Sumário

1. [Visão Geral](#1-visão-geral)
2. [Arquitetura de Componentes](#2-arquitetura-de-componentes)
3. [Event Scheduler — Motor Discreto](#3-event-scheduler--motor-discreto)
4. [Network — Orquestrador Principal](#4-network--orquestrador-principal)
5. [Fluxo Completo de Simulação](#5-fluxo-completo-de-simulação)
6. [Channel Model — SINR e Interferência](#6-channel-model--sinr-e-interferência)
7. [End Device — FSM e OTAA](#7-end-device--fsm-e-otaa)
8. [Gateway — SX1301 e Recepção](#8-gateway--sx1301-e-recepção)
9. [Packet — Estrutura de Dados](#9-packet--estrutura-de-dados)
10. [Modelos de Propagação](#10-modelos-de-propagação)
11. [Modelo de Energia](#11-modelo-de-energia)
12. [Network Server e Subsistemas](#12-network-server-e-subsistemas)
13. [LR-FHSS e ACRDA](#13-lr-fhss-e-acrda)
14. [Algoritmos Físicos Chave](#14-algoritmos-físicos-chave)
15. [Configuração Global (parametors.py)](#15-configuração-global-parametorspy)
16. [Saídas: Métricas, Exportação e Gráficos](#16-saídas-métricas-exportação-e-gráficos)
17. [Mapa de Arquivos](#17-mapa-de-arquivos)

---

## 1. Visão Geral

O PyLoRaWAN é um simulador LoRaWAN full-stack escrito em Python puro (apenas numpy). Simula a operação completa de uma rede LoRaWAN por 1 hora, incluindo camadas PHY, MAC, e Network Server.

### Cenário de Referência

| Parâmetro         | Valor                        |
|-------------------|------------------------------|
| End Devices (EDs) | 50 (configurável)            |
| Gateways (GWs)    | 1 (configurável)             |
| Área              | 10 km × 10 km                |
| Duração           | 3600 s (1 hora)              |
| Região            | EU868 (padrão)               |
| Spreading Factors | SF7 a SF12                   |
| TX Power          | 14 dBm (máx EU868)           |
| Payload           | 20 bytes                     |
| Taxa de envio     | Poisson (λ = 1/300 s padrão) |

### PDR Típico (cenário referência)
- **CSS**: ~90%
- **LR-FHSS**: ~100% (fragmentação + SIC elimina colisões)

---

## 2. Arquitetura de Componentes

```
┌─────────────────────────────────────────────────────────────────┐
│                         Network (network.py)                    │
│  Orquestrador central — inicializa, executa, coleta estatísticas│
└───────────────────────────┬─────────────────────────────────────┘
                            │
        ┌───────────────────┼───────────────────────┐
        │                   │                       │
┌───────▼──────┐   ┌────────▼──────┐   ┌───────────▼────────────┐
│EventScheduler│   │  ChannelModel │   │    NetworkServer (NS)  │
│(event_sched.)│   │  (channel.py) │   │  (network_server/)     │
│Priority Queue│   │  SINR, on-air │   │  ADR, DL scheduler     │
└──────────────┘   └───────────────┘   └────────────────────────┘
        │
┌───────▼──────────────────────────────────────────────────────┐
│                  Loop de Eventos                             │
│  DEVICE_SEND → TX_END → RX1_OPEN → RX2_OPEN → DEVICE_SEND  │
└──────────────────────────────────────────────────────────────┘
        │                            │
┌───────▼──────────┐      ┌──────────▼───────────┐
│  EndDevice × 50  │      │    Gateway × 1        │
│  (enddevice.py)  │      │    (gateway.py)       │
│  FSM, OTAA,      │      │    8 ReceptionPaths   │
│  EnergyModel     │      │    SX1301, DL tracking│
└──────────────────┘      └──────────────────────┘
        │
┌───────▼──────────────────────────────────────────────────────┐
│                    Packet (packet.py)                        │
│  UUID, SF, RSSI, SINR, collided, lrfhss_fragments, MIC, ... │
└──────────────────────────────────────────────────────────────┘
```

### Dependências entre módulos

```
network.py
  ├── parametors.py          (config global)
  ├── event_scheduler.py     (fila de eventos)
  ├── channel.py             (SINR, on-air)
  ├── enddevice.py           (dispositivos)
  │     ├── energymodel.py
  │     ├── battery.py
  │     └── security.py
  ├── gateway.py             (gateway SX1301)
  ├── packet.py              (estrutura de dados)
  ├── network_server/        (NS completo)
  │     ├── adr_component.py
  │     ├── gateway_manager.py
  │     ├── downlink_scheduler.py
  │     └── device_registry.py
  ├── mac_commands.py        (MAC LoRaWAN 1.1)
  ├── lrfhss.py              (fragmentação + SIC)
  ├── regions.py             (EU868, US915, AU915, AS923)
  ├── analytics.py           (métricas)
  └── plots.py               (visualização)
```

---

## 3. Event Scheduler — Motor Discreto

**Arquivo:** `event_scheduler.py`

O motor de simulação é baseado em **Discrete Event Simulation (DES)** com fila de prioridade (min-heap via `heapq`).

### EventType (Enum)

| Evento              | Descrição                              |
|---------------------|----------------------------------------|
| `DEVICE_SEND`       | Dispara transmissão de uplink          |
| `PACKET_TX_END`     | Fim da transmissão (após airtime)      |
| `RX1_WINDOW_OPEN`   | Abre janela RX1 (1s após TX)           |
| `RX2_WINDOW_OPEN`   | Abre janela RX2 (2s após TX)           |
| `MOBILITY_UPDATE`   | Atualiza posição (a cada 1s)           |
| `BEACON`            | Beacon Class B (a cada 128s)           |
| `ENERGY_HARVEST`    | Coleta de energia (a cada 60s)         |
| `JOIN_REQUEST`      | Fase OTAA inicial                      |
| `SIMULATION_END`    | Termina simulação                      |

### Estrutura de Evento

```python
@dataclass
class Event:
    time: float          # tempo absoluto de disparo
    event_id: int        # tie-breaker (monotônico)
    event_type: EventType
    callback: Callable
    args: tuple
    cancelled: bool = False

    def __lt__(self, other):
        return (self.time, self.event_id) < (other.time, other.event_id)
```

### EventScheduler — API Principal

```python
scheduler.schedule(delay, event_type, callback, *args)
# → agenda em current_time + delay

scheduler.schedule_absolute(time, event_type, callback, *args)
# → agenda em tempo absoluto

scheduler.cancel(event)
# → marca event.cancelled = True (lazy deletion)
# → dispara compaction se threshold excedido

scheduler.run(until=3600)
# → Main loop: pop evento, verifica tempo, chama callback

scheduler.compact()
# → Reconstrói heap removendo eventos cancelados
# → O(N) — executado automaticamente após N cancelamentos
```

### Complexidade

| Operação     | Complexidade |
|--------------|-------------|
| `schedule()` | O(log N)    |
| `cancel()`   | O(1)        |
| `run()` step | O(log N)    |
| `compact()`  | O(N)        |

---

## 4. Network — Orquestrador Principal

**Arquivo:** `network.py` (~1227 linhas)

### Atributos Principais

```python
class Network:
    # Componentes
    scheduler: EventScheduler
    channel: ChannelModel
    ns: NetworkServer
    lorawan_mac: LoRaWANMAC

    # Entidades
    devices: list[EndDevice]       # 50 dispositivos
    gateways: list[Gateway]        # 1 gateway (configurável)

    # Estado
    current_time: float            # tempo de simulação atual
    energy_log: list[float]        # série temporal de energia

    # Configuração de propagação
    model_pathloss: str            # "log_normal_shadowing" padrão
    correlated_shadowing: bool
    shadowing_map: CorrelatedShadowingMap
    interference_model: str        # "semtech" ou "goursaud"

    # LR-FHSS
    lrfhss_ratio: float
    lrfhss_phy: LRFHSS_PHY
    lrfhss_channel: LRFHSS_Channel
    acrda: ACRDA
```

### CorrelatedShadowingMap (G2)

Implementa correlação espacial do desvanecimento usando interpolação bilinear:

```python
# Grid de ruído gaussiano
grid_step = 50m
grid[i][j] = N(0, σ=3.57 dB)

def get_shadowing(x, y):
    # Interpolação bilinear entre os 4 nós mais próximos
    # Distância de correlação ≈ 40m (equiv. ao ns-3)
    return bilinear_interp(x, y, grid)
```

### Estratégias de Deployment

| Tipo              | Distribuição                             |
|-------------------|------------------------------------------|
| `random_uniform`  | Posições aleatórias uniformes            |
| `grid`            | Grade regular (√N × √N)                  |
| `circular`        | Anel ao redor do centro                  |
| `hexagonal`       | Empacotamento hexagonal                  |
| `clustered`       | Grupos com distância inter-cluster       |
| `from_file`       | Carrega CSV de posições                  |

---

## 5. Fluxo Completo de Simulação

### 5.1 Inicialização

```
1. Criar Network(parametors)
   ├── initialize_devices()
   │    ├── Posicionar 50 EDs (estratégia de deployment)
   │    ├── Atribuir SF aleatório (7-12)
   │    ├── Atribuir frequência aleatória (868.1/868.3/868.5 MHz)
   │    ├── Gerar AppKey, DevEUI, AppEUI (OTAA)
   │    ├── Marcar 30% como indoor (building penetration)
   │    └── Selecionar lrfhss_ratio × 50 dispositivos para LR-FHSS
   │
   └── initialize_gateways()
        └── Posicionar GWs em grade + registrar no NS
```

### 5.2 Fase de Join OTAA

```
Para cada dispositivo:
  device.prepare_join_request()
    └── DevNonce++, MIC = CMAC(AppKey, JoinRequest)

  ns.on_join_request(device, join_request)
    ├── Validar MIC
    ├── Gerar AppNonce, NetID, DevAddr
    └── Derivar NwkSKey, AppSKey

  device.process_join_accept(app_nonce, net_id, dev_addr)
    ├── Armazenar DevAddr, NwkSKey, AppSKey
    └── Reset frame_counter_up = 0
```

### 5.3 Agendamento Inicial

```
Para cada dispositivo:
  scheduler.schedule(delay=Poisson(λ), DEVICE_SEND, _on_device_send, device)

scheduler.schedule(1.0, MOBILITY_UPDATE, _on_mobility_update)   # se mobility
scheduler.schedule(128.0, BEACON, _on_beacon)                   # se Class B
scheduler.schedule(60.0, ENERGY_HARVEST, _on_energy_harvest)    # se harvesting
```

### 5.4 Loop Principal de Eventos

```
scheduler.run(until=3600)  →  while heap não vazio e current_time ≤ 3600:
  event = heappop()
  if event.cancelled: continue
  current_time = event.time
  event.callback(*event.args)
```

### 5.5 Handler: _on_device_send(device)

```
1. Verificar bateria → se esgotada, return
2. Verificar duty cycle → se current_time < dc_release_time:
      reschedule com delay residual e return
3. Selecionar canal aleatório
4. Verificar cobertura (find_best_gateway)
5. Se LR-FHSS:
      lrfhss_phy.create_fragments(packet)
      lrfhss_channel.add_fragments(fragments)
      acrda.register_packet(packet)
6. Se CSS:
      gateway.process_uplink(packet)     ← calcula RSSI/SINR
      channel.add_transmission(packet)   ← registra on-air
7. ns.on_uplink_received(packet) → gera MAC commands
8. _apply_mac_commands(device, mac_commands)
9. Calcular duty cycle release:
      dc_release_time = time + airtime / dc_limit
10. Atualizar energia: energy_model.update_energy(device, airtime)
11. scheduler.schedule(airtime, TX_END, _on_tx_end, device, packet)
12. scheduler.schedule(next_interval, DEVICE_SEND, _on_device_send, device)
```

### 5.6 Handler: _on_tx_end(device, packet)

```
1. transition_state(WAIT_RX1)
2. Se LR-FHSS:
      lrfhss_channel.check_fragment_collisions(packet.lrfhss_fragments)
      result = lrfhss_channel.evaluate_packet(fragments, threshold)
3. Se CSS:
      result = channel.evaluate_reception(packet, gateway)
4. gateway.release_paths(current_time)
5. channel.cleanup_expired(current_time)
6. scheduler.schedule(receive_delay1, RX1_OPEN, _on_rx1_open, device, packet)
```

### 5.7 Handler: _on_rx1_open(device, packet)

```
1. transition_state(RX1)
2. dl = ns.dl_scheduler.get_downlink(device.dev_addr)
3. Se dl disponível:
      gw = ns.gateway_manager.select_gateway(device)
      Se gw.can_send_downlink(freq, time, airtime, window='rx1'):
          → Enviar DL (ACK + MAC commands)
          → packet.ack_received = True
          → _apply_mac_commands(device, dl.mac_commands)
          → return   (não abre RX2)
4. Senão:
      scheduler.schedule(remaining_delay, RX2_OPEN, _on_rx2_open, device, packet)
```

### 5.8 Handler: _on_rx2_open(device, packet)

```
1. transition_state(RX2)
2. freq = device._rx2_freq ou 869.525 MHz
   sf   = device._rx2_sf  ou SF12
3. Mesma lógica de DL da RX1, mas com duty cycle RX2 (10%)
4. Se colisão E retransmission_attempts < max_retransmissions:
      backoff = random(1, 10) * 2^attempt
      scheduler.schedule(backoff, DEVICE_SEND, _on_device_send, device)
      packet.is_retransmission = True
      device.retransmission_attempts++
5. Senão: transition_state(IDLE)
```

### 5.9 Pós-Simulação

```
acrda.process_window()              # SIC para LR-FHSS
metrics = analytics.compute_metrics(network)
analytics.export_json(metrics)
analytics.export_csv_detailed(network)
analytics.export_npz(network)
plots.plot_all(network)
network.print_statistics()
```

---

## 6. Channel Model — SINR e Interferência

**Arquivo:** `channel.py`

### Rastreamento On-Air

```python
on_air: list[tuple[Packet, tx_start, tx_end]]
```

Todo pacote em transmissão é adicionado ao `on_air`. O método `cleanup_expired(time)` remove pacotes com `tx_end ≤ time`.

### evaluate_reception(packet, gateway) — Detecção de Colisão

#### Passo 1: Verificação de SNR mínimo

```python
SNR_min = {SF7: -7.5, SF8: -10.0, SF9: -12.5,
           SF10: -15.0, SF11: -17.5, SF12: -20.0}  # dB

if RSSI - noise_floor < SNR_min[SF]:
    return False  # sinal fraco demais
```

#### Passo 2: Preamble Locking (G13)

```python
Tsym = 2^SF / BW
preamble_lock_time = tx_start + 6 * Tsym

# Interferentes que chegam DEPOIS do lock não afetam sincronização
# (apenas interferentes com tx_start < preamble_lock_time importam)
```

#### Passo 3: Seleção da Matriz de Interferência

```python
# Índice: SF7 → 5, SF12 → 0
sf_idx = 12 - SF

# Duas opções configuráveis:
SEMTECH_MATRIX[sf_target][sf_interferer]  # threshold Semtech AN1200.18 (co-SF = 1 dB)
GOURSAUD_MATRIX[sf_target][sf_interferer] # threshold ns-3 (co-SF = 6 dB, mais conservador)
```

#### Passo 4: Captura de Energia (G14)

```
Para cada interferente com sobreposição temporal em mesma frequência:

  overlap = min(tx_end, other.tx_end) - max(tx_start, other.tx_start)

  energy_ratio_db = (RSSI_signal - RSSI_interferer)
                  + 10 * log10(T_packet / T_overlap)

  threshold_db = matrix[sf_signal][sf_interferer]

  Se energy_ratio_db < threshold_db:
      → COLISÃO (interferência domina)
  Senão:
      → SUCESSO (sinal captura o canal)
```

Este modelo captura o princípio físico de que pacotes mais longos acumulam mais energia relativa contra interferências curtas — equivalente ao modelo ns-3 de interferência baseada em energia.

---

## 7. End Device — FSM e OTAA

**Arquivo:** `enddevice.py`

### RadioState FSM (MAC — 7 estados)

```
    ┌──────────────────────────────────────────────────────────┐
    │                                                          │
    ▼                                                          │
  IDLE ──(send)──► TX ──(airtime)──► WAIT_RX1                 │
                                        │                      │
                                     (1s)                      │
                                        ▼                      │
                                       RX1 ──(no DL)──► WAIT_RX2
                                                            │
                                                         (2s)
                                                            ▼
                                                           RX2 ──► SLEEP ──► IDLE
```

### Mapeamento FSM → Energia

```python
_ENERGY_STATE_MAP = {
    RadioState.IDLE:      EnergyState.STANDBY,
    RadioState.WAIT_RX1:  EnergyState.STANDBY,
    RadioState.WAIT_RX2:  EnergyState.STANDBY,
    RadioState.TX:        EnergyState.TX,
    RadioState.RX1:       EnergyState.RX,
    RadioState.RX2:       EnergyState.RX,
    RadioState.SLEEP:     EnergyState.SLEEP,
}
```

### Cálculo de Airtime (SX1272)

```python
def calculate_airtime(self):
    Tsym = 2**SF / BW
    Tpream = (8 + 4.25) * Tsym

    DE = 1 if Tsym > 16e-3 else 0  # low data rate optimization
    H  = 0                           # header presente

    payloadSymbNB = 8 + max(
        ceil((8*PL - 4*SF + 28 + 16*CRC - 20*H) / (4*(SF - 2*DE))) * (CR + 4),
        0
    )
    ToA = Tpream + payloadSymbNB * Tsym
    return ToA
```

### OTAA — Derivação de Chaves

```python
def prepare_join_request(self):
    self.dev_nonce += 1
    mic = security.compute_join_mic(self.app_key, self.dev_eui,
                                    self.app_eui, self.dev_nonce)
    return JoinRequest(dev_eui, app_eui, dev_nonce, mic)

def process_join_accept(self, app_nonce, net_id, dev_addr):
    self.nwk_skey = security.derive_nwk_skey(
        self.app_key, app_nonce, net_id, self.dev_nonce)
    self.app_skey = security.derive_app_skey(
        self.app_key, app_nonce, net_id, self.dev_nonce)
    self.dev_addr = dev_addr
    self.frame_counter_up = 0
```

### Elegibilidade para RX por Classe

| Classe | Janelas RX abertas                          |
|--------|---------------------------------------------|
| A      | Apenas RX1 e RX2 (após cada uplink)         |
| B      | RX1, RX2 + ping slots (alinhados ao beacon) |
| C      | Sempre (exceto durante TX)                  |

---

## 8. Gateway — SX1301 e Recepção

**Arquivo:** `gateway.py`

### ReceptionPath (SX1301)

O SX1301 possui **8 demoduladores paralelos**. Cada path:

```python
@dataclass
class ReceptionPath:
    busy: bool = False
    packet: Packet = None
    tx_end: float = 0.0

    def is_free(self, current_time) -> bool:
        return not self.busy or current_time >= self.tx_end

    def assign(self, packet, tx_end):
        self.busy = True
        self.packet = packet
        self.tx_end = tx_end
```

Se todos os 8 paths estiverem ocupados → `saturation_events++`, pacote descartado.

### process_uplink(packet) — Cálculo do Link Budget

```python
def process_uplink(self, packet):
    # 1. Interferência DL-UL (G8)
    if self.dl_busy_until.get(packet.freq, 0) > current_time:
        packet.collided = True
        return

    # 2. Alocar reception path
    if not self.try_allocate_path(packet, current_time):
        self.saturation_events += 1
        return

    # 3. Calcular link budget
    distance = sqrt(dx² + dy² + dh²)
    path_loss = network.pathloss(distance, freq, model, dev.x, dev.y)
    path_loss += network.get_building_penetration(device)   # G3: indoor
    RSSI = tx_power + ed_antenna_gain + gw_antenna_gain - path_loss

    # 4. Acumular interferência por SF (G14)
    for other in channel.on_air:
        if other.freq == packet.freq and other != packet:
            overlap = min(tx_end, other.tx_end) - max(tx_start, other.tx_start)
            if overlap > 0:
                overlap_ratio = overlap / packet.rectime
                other_RSSI_linear = 10**(other_RSSI/10)
                interference_per_sf[other.sf] += other_RSSI_linear * overlap_ratio

    # 5. Calcular SINR
    noise_linear = 10**(noise_floor/10)
    signal_linear = 10**(RSSI/10)
    SINR = signal_linear / (sum(interference_per_sf.values()) + noise_linear)
    packet.sinr = 10 * log10(SINR)

    # 6. Armazenar resultado
    packet.rssi = RSSI
    packet.snr  = RSSI - noise_floor
    packet.noise_floor = noise_floor
    packet.interference_per_sf = interference_per_sf
```

### Controle DL-UL (G8)

```python
# Ao enviar downlink:
self.dl_busy_until[freq] = current_time + dl_airtime

# Durante process_uplink:
if dl_busy_until[packet.freq] > current_time:
    packet.collided = True
```

---

## 9. Packet — Estrutura de Dados

**Arquivo:** `packet.py`

```python
@dataclass
class Packet:
    # Identificação
    packet_id: str          # UUID
    device_id: int
    frame_counter: int
    packet_type: str        # "uplink", "downlink", "ack"

    # PHY
    sf: int                 # 7–12
    tx_power: float         # dBm
    bw: int                 # Hz
    freq: float             # MHz
    pl: int                 # bytes
    rectime: float          # ToA em segundos

    # Timing
    arrival_time: float     # timestamp de início
    tx_start: float         # = arrival_time
    tx_end: float           # = arrival_time + rectime

    # Métricas de Recepção
    rssi: float             # dBm
    snr: float              # dB
    sinr: float             # dB
    noise_floor: float      # dBm
    interference_per_sf: dict  # {SF → W linear}

    # Resultado
    collided: bool = False
    received: bool = False

    # MAC
    mic: bytes              # 4 bytes
    mic_valid: bool
    confirmed: bool = False
    ack_received: bool = False
    is_retransmission: bool = False
    retry_count: int = 0
    gateway_id: int = None

    # LR-FHSS
    phy_type: str = "CSS"   # ou "LR-FHSS"
    lrfhss_fragments: list = None
```

---

## 10. Modelos de Propagação

**Arquivo:** `network.py` — método `pathloss()`

### 9 Modelos Disponíveis

#### 1. `log_normal_shadowing` (padrão — equivalente ao FLoRa/ns-3)

```
PL = PL_d0 + 10 * γ * log10(d / d0) + X_σ

PL_d0 = 7.7 dB  (d0 = 1m)
γ = 3.76         (expoente urbano)
σ = 3.57 dB      (desvio padrão shadowing)
X_σ ~ N(0, σ)    (novo sample por transmissão)
```

#### 2. `correlated_shadowing` (G2 — spatial correlation)

```
PL = PL_d0 + 10 * γ * log10(d / d0) + shadowing_map.get_shadowing(x, y)

Shadowing correlacionado espacialmente via interpolação bilinear
Grid 50m, σ = 3.57 dB, correlação ≈ 40m
```

#### 3. `okumura_hata` (macro urbano)

```
PL = 69.55 + 26.16*log10(f) - 13.82*log10(ht) - CF
   + (44.9 - 6.55*log10(ht)) * log10(d)

CF = fator de correção dependente de ht, hr, f
```

#### 4. `fspl` (espaço livre)

```
PL = 20*log10(d) + 20*log10(f) + 32.45
```

#### 5. `log_distance`

```
PL = 20*log10(f) + 10 * γ * log10(d) - 28,  γ = 3.5
```

#### 6. `cost_hata` (suburbano/rural)

Similar ao Okumura-Hata com coeficientes ajustados.

#### 7. `building_penetration`

```
PL = log_normal_base + BPL
BPL = lognormal(μ=ln(20), σ=0.5)  dB  (apenas se device.is_indoor)
```

#### 8. `fading` (Rayleigh/Rician/Nakagami)

```
PL_base = log_normal
fading_sample ~ Rayleigh/Rician/Nakagami
fading_dB = 20 * log10(sample)
PL = PL_base - fading_dB
```

#### 9. `oulu` (LoRaSim)

```
PL = 127.41 + 10 * 2.08 * log10(d / 40) + N(0, 3.57)
d0 = 40m,  PL_d0 = 127.41 dB,  γ = 2.08
```

### Ruído Térmico

```python
def calculate_noise_floor(bw):
    k = 1.38e-23   # J/K (Boltzmann)
    T = 294.15     # K  (21°C)
    NF = 6.0       # dB (noise figure SX1301)
    return 10 * log10(k * T * bw) + 30 + NF
    # Para BW=125kHz → ≈ -120 dBm
```

### Ganhos de Antena

| Componente   | Valor típico |
|--------------|-------------|
| ED (isotrópica) | 2.15 dBi |
| GW (direcional) | 6 dBi    |

---

## 11. Modelo de Energia

**Arquivo:** `energymodel.py`

### RadioState → Corrente (SX1272)

| Estado     | Corrente   | Tensão | Fonte                 |
|------------|-----------|--------|-----------------------|
| `SLEEP`    | 0.0015 mA | 3.3 V  | Datasheet SX1272      |
| `STANDBY`  | 1.4 mA    | 3.3 V  | Datasheet SX1272      |
| `RX`       | 11.2 mA   | 3.3 V  | Datasheet SX1272      |
| `TX` (14dBm)| 38.0 mA  | 3.3 V  | Datasheet SX1272/RN2483|
| `TX` (12dBm)| 35.1 mA  | 3.3 V  | Datasheet RN2483      |
| `TX` (10dBm)| 32.4 mA  | 3.3 V  | Datasheet RN2483      |

### transition(new_state, time, tx_power=None)

```python
def transition(self, new_state, time, tx_power=None):
    duration = time - self.state_entry_time
    current_ma = self._get_current(self.current_state, self._current_tx_power)

    energy_mj = current_ma * self.voltage_v * duration  # mA * V * s = mJ
    self.energy_consumed += energy_mj
    self.energy_breakdown[self.current_state] += energy_mj
    self.state_durations[self.current_state] += duration

    if self._battery:
        self._battery.consume(energy_mj)

    self.current_state = new_state
    self.state_entry_time = time
    if tx_power is not None:
        self._current_tx_power = tx_power
```

### Ciclo Completo de TX (update_energy)

```
Ciclo por transmissão:
  1. SLEEP: gap desde último TX até início deste
  2. TX:      airtime_s @ tx_current[power]
  3. STANDBY: receive_delay1 = 1s
  4. RX1:     trx1 = 1s
  5. STANDBY: receive_delay2 - receive_delay1 - trx1
  6. RX2:     trx2 = 2s
  7. SLEEP:   até próximo TX
```

Energia típica por ciclo: **10–100 mJ** (varia com SF e período de sono).

---

## 12. Network Server e Subsistemas

**Diretório:** `network_server/`

### Componentes do NS

```
NetworkServer
├── DeviceRegistry      — estado por dispositivo (SF, SNR hist., cobertura)
├── GatewayManager      — seleção de GW, duty cycle DL
├── DownlinkScheduler   — fila de DLs por dev_addr
├── ADRComponent        — ADR server-side (G5)
└── LinkCheckHandler    — processamento de LinkCheckReq
```

### ADR Server-Side (G5)

```
Entrada: SNR history (último 20 pacotes), SF atual, região
Saída: LinkAdrReq com novo DR (SF/BW) e TX power

Lógica:
  SNR_max = max(snr_history)
  margin = SNR_max - SNR_min[SF] - adr_margin_db  (adr_margin = 10dB)

  While margin >= step_db (3dB):
      Se SF > 7: SF--
      Senão: TX_power -= step
      margin -= step

  Gerar LinkAdrReq → enfileirado no DownlinkScheduler
```

### GatewayManager — Duty Cycle DL

```python
def can_send_downlink(self, gw_id, time, airtime, window):
    limit = 0.01 if window == 'rx1' else 0.10
    last_tx = self.last_dl_time[gw_id][window]
    min_gap = airtime / limit
    return (time - last_tx) >= min_gap
```

### MAC Commands Processados (_apply_mac_commands)

| CID  | Comando           | Efeito no Dispositivo                     |
|------|-------------------|-------------------------------------------|
| 0x02 | LinkCheckReq/Ans  | Device verifica cobertura                 |
| 0x03 | LinkAdrReq        | Atualiza SF, TX power, canais habilitados |
| 0x04 | DutyCycleReq      | Define duty cycle agregado máximo         |
| 0x05 | RXParamSetupReq   | Configura freq/SF do RX1 offset e RX2    |
| 0x06 | DevStatusReq      | Solicita bateria + SNR margin             |
| 0x07 | NewChannelReq     | Adiciona/modifica canal                   |
| 0x08 | RXTimingSetupReq  | Altera receive_delay1                     |
| 0x0A | DlChannelReq      | Configura canal exclusivo de downlink     |

---

## 13. LR-FHSS e ACRDA

**Arquivo:** `lrfhss.py`

### Fragmentação LR-FHSS

```
Code Rate 1/3:  2 bytes/fragmento, threshold = ceil(n_payloads / 3)
Code Rate 2/3:  4 bytes/fragmento, threshold = ceil(n_payloads / 3)
Code Rate 1/2:  3 bytes/fragmento, threshold = ceil(n_payloads / 2)

Por pacote (20 bytes, CR=1/3):
  n_payloads = ceil(23 / 2) = 12 fragmentos de payload
  n_headers  = 3 (fixo)
  threshold  = ceil(12 / 3) = 4 fragmentos mínimos para reconstrução

  Duração total ≈ 3 * 233ms + 12 * 102ms + 15 * 6.5ms ≈ 2.1s
```

### Hopping Sequence

```python
def create_fragments(self, packet_id, payload_size, tx_start):
    n_headers, n_payloads, threshold = self.fragment_packet(payload_size)

    for i in range(n_headers):
        channel = random.randint(0, self.obw - 1)  # 0–34 (35 canais OBW)
        fragments.append(LRFHSSFragment(
            frag_type='header', channel=channel, duration=self.header_duration,
            packet_id=packet_id, tx_start=tx_start + i * (header_duration + wait)
        ))
    # ... similar para payloads
```

### ACRDA — Successive Interference Cancellation (SIC)

```
process_window():
  Iteração 1:
    Fragmentos sem colisão → decodificados com certeza
    Pacote pai reconstruído se threshold atingido

  Iteração 2:
    Sinal dos fragmentos decodificados é removido do canal
    Mais fragmentos tornam-se decodificáveis

  Repetir até convergência (ou max_iterations)
```

**Efeito:** Elimina colisões cascata em LR-FHSS → PDR ≈ 100% vs ~90% CSS no cenário de referência.

---

## 14. Algoritmos Físicos Chave

### 14.1 SINR Completo

```
Signal_linear    = 10^(RSSI_dBm / 10)   [mW]
Noise_linear     = 10^(noise_floor / 10) [mW]
Interf_linear    = Σ 10^(RSSI_i/10) × overlap_ratio_i

SINR_dB = 10 × log10(Signal / (Noise + Interf))
```

### 14.2 Captura por Energia (G14)

```
energy_ratio_db = (RSSI_signal - RSSI_interferer)
                + 10 × log10(T_packet / T_overlap)

threshold_db = interference_matrix[sf_signal][sf_interferer]

Colisão: energy_ratio_db < threshold_db
```

### 14.3 Duty Cycle

```
airtime_s = calculate_airtime()
dc_limit  = regional_sub_band_limit(freq)   # ex: 0.01 para EU868 g-band
dc_release = current_time + airtime / dc_limit
```

### 14.4 Seleção de SF (find_best_gateway)

```
Para cada SF 7..12:
  distance = sqrt(dx² + dy² + dh²)
  path_loss = pathloss(distance, ...)
  RSSI = tx_power + gains - path_loss

  Se RSSI > gw_sensitivity[SF, BW]:
    selecionar este SF (mais rápido/eficiente possível)
    break
Senão: SF = 12 (mais robusto)
```

### 14.5 Validação Analítica (Ps1/Ps2)

```python
# Por SF (LoRaWANSim — Magrin et al. 2017)
G_SF = (n_SF × ToA_SF) / (avg_period × n_channels)
Ps1 = exp(-2 × G_SF / M) × P_coverage_SF   # ALOHA puro

# RX2 fallback (apenas confirmados)
G_RX2 = lambda_failed × ToA_RX2 / 1
Ps2   = exp(-2 × G_RX2)

# PDR analítico vs simulado
analytical_pdr = Ps1 + confirmed_ratio × (1 - Ps1) × Ps2
difference = |sim_pdr - analytical_pdr|
```

---

## 15. Configuração Global (parametors.py)

**Arquivo:** `parametors.py` (~131 linhas)

### Parâmetros de Rede

```python
num_devices       = 50
num_gateways      = 1
area_size         = 10000      # metros
simulation_time   = 3600       # segundos
lambda_rate       = 1/300      # Hz (Poisson)
```

### PHY LoRa

```python
sf_range          = [7, 8, 9, 10, 11, 12]
tx_power          = 14         # dBm (máx EU868)
frequency_mhz     = [868.1, 868.3, 868.5]
bw                = 125000     # Hz
cr                = 1          # (4/5)
pl                = 20         # bytes
```

### Sensibilidade SX1272 (End Device)

| SF  | BW 125kHz |
|-----|----------|
| SF7 | -124 dBm |
| SF8 | -127 dBm |
| SF9 | -130 dBm |
| SF10| -133 dBm |
| SF11| -135 dBm |
| SF12| -137 dBm |

### Sensibilidade SX1301 (Gateway, ~6dB melhor)

| SF  | BW 125kHz   |
|-----|------------|
| SF7 | -130.0 dBm |
| SF8 | -132.5 dBm |
| SF9 | -135.0 dBm |
| SF10| -137.5 dBm |
| SF11| -140.0 dBm |
| SF12| -142.5 dBm |

### Matriz de Interferência Semtech (AN1200.18)

```
        SF7   SF8   SF9   SF10  SF11  SF12
SF7  [  1,   -8,  -9,  -9,  -9,  -9]
SF8  [ -11,   1,  -11, -12, -13, -13]
SF9  [ -15,  -13,  1,  -13, -14, -15]
SF10 [ -19,  -18, -17,   1, -17, -18]
SF11 [ -22,  -22, -21, -20,   1, -20]
SF12 [ -25,  -25, -25, -24, -23,   1]
```

Valores em dB. Co-SF: threshold = 1 dB (Semtech). Goursaud: co-SF = 6 dB.

---

## 16. Saídas: Métricas, Exportação e Gráficos

### compute_metrics() → Dict completo

**simulation:**
- `duration_s`, `num_devices`, `num_gateways`, `region`, `model_pathloss`, `deployment_type`

**performance:**
- `pdr_percent` — PDR geral
- `pdr_per_sf` — dict {SF → PDR%}
- `pdr_vs_distance_km` — dict {distância → PDR%}
- `total_packets`, `collisions`, `retransmissions`
- `retransmission_rate_percent`
- `delay_p50_ms`, `delay_p95_ms`

**energy:**
- `total_network_mj`, `avg/min/max_device_mj`
- `avg_per_sf_mj` — dict {SF → mJ}
- `breakdown_percent` — dict {TX/RX/STANDBY/SLEEP → %}
- `avg_battery_lifetime_days`

**radio:**
- `sf_distribution` — dict {SF → count}
- `avg/min/p5_sinr_db`

**analytical:** (se computável)
- `ps1_per_sf`, `ps2`, `analytical_pdr`, `simulation_pdr`, `difference`

### Exportações

| Formato  | Método                       | Conteúdo                          |
|----------|------------------------------|-----------------------------------|
| JSON     | `export_json(metrics)`       | Métricas agregadas                |
| CSV      | `export_csv_detailed()`      | 1 linha por pacote (15 campos)    |
| CSV      | `export_device_summary()`    | 1 linha por dispositivo           |
| NPZ      | `export_npz()`               | 15 arrays numpy comprimidos       |

### Gráficos (plots.py — 8 figuras PNG)

| Função                    | Visualização                           |
|---------------------------|----------------------------------------|
| `plot_pdr_per_sf`         | Bar chart PDR(%) × SF                  |
| `plot_pdr_vs_distance`    | Linha PDR × distância (bins 1km)       |
| `plot_energy_per_sf`      | Error bars energia × SF                |
| `plot_delay_cdf`          | CDF delays (P50, P90, P95 marcados)    |
| `plot_sinr_distribution`  | Histograma SINR (média e P5 marcados)  |
| `plot_collision_heatmap`  | Heatmap 2D colisões + posição GWs      |
| `plot_throughput_over_time`| Step plot pkts OK por janela 60s      |
| `plot_battery_soc_timeline`| Histograma SoC final (%)              |

Chamada única: `plots.plot_all(network, prefix="run1_")`

---

## 17. Mapa de Arquivos

| Arquivo                                  | Papel                                    | Linhas |
|------------------------------------------|------------------------------------------|--------|
| `network.py`                             | Orquestrador principal, event handlers   | ~1227  |
| `parametors.py`                          | Configuração global                      | ~131   |
| `channel.py`                             | SINR, on-air tracking, captura de energia| ~135   |
| `enddevice.py`                           | FSM Class A/B/C, OTAA, mobilidade        | ~278   |
| `gateway.py`                             | SX1301 8 paths, link budget, DL-UL       | ~213   |
| `packet.py`                              | Estrutura de dados do pacote             | ~61    |
| `event_scheduler.py`                     | Min-heap DES, lazy deletion              | ~112   |
| `energymodel.py`                         | Consumo por estado (datasheet SX1272)    | ~150   |
| `lrfhss.py`                              | Fragmentação, hopping, ACRDA/SIC         | ~200   |
| `mac_commands.py`                        | MAC commands LoRaWAN 1.1                 | ~300   |
| `analytics.py`                           | Métricas, modelos analíticos, exportação | ~500   |
| `plots.py`                               | 8+ gráficos matplotlib (Agg backend)     | ~309   |
| `battery.py`                             | BatteryModel + EnergyHarvester           | ~100   |
| `regions.py`                             | EU868, US915, AU915, AS923               | ~150   |
| `security.py`                            | OTAA MIC, KDF session keys               | ~80    |
| `protocolos.py`                          | LoRaWANMAC (frame formatting)            | ~120   |
| `packettracker.py`                       | Estatísticas por pacote e dispositivo    | ~80    |
| `deployment.py`                          | Geradores de posição (grid, hex, etc.)   | ~100   |
| `config_loader.py`                       | Carregamento de YAML                     | ~50    |
| `network_server/adr_component.py`        | ADR server-side, LinkAdrReq              | ~100   |
| `network_server/gateway_manager.py`      | Seleção GW, duty cycle DL               | ~80    |
| `network_server/downlink_scheduler.py`   | Fila de downlinks por device             | ~60    |
| `network_server/device_registry.py`      | Estado e histórico por dispositivo       | ~80    |

---

*Gerado automaticamente a partir do código-fonte — Sprint 7 concluído.*
