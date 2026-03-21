# Downlink Scheduling — PyLoRaWAN

## Visão Geral

O subsistema de downlink coordena a entrega de pacotes do Network Server (NS) para os End Devices (EDs) através das janelas de recepção Class A (RX1 e RX2). Envolve cinco camadas:

1. **DownlinkScheduler** — fila de prioridades no NS
2. **GatewayManager** — seleção de gateway e duty cycle de DL
3. **Network.py event handlers** — `_on_rx1_open` / `_on_rx2_open`
4. **MACCommandProcessor** — aplicação de MAC commands no ED
5. **DutyCycleComponent** — controle de sub-banda no NS

```
NetworkServer
├── DownlinkScheduler  (priority heap)
├── GatewayManager     (DC tracking por janela)
└── NetworkController  (componentes que geram MAC commands)
       ├── ADRComponent     → LinkAdrReq
       ├── LinkCheckComponent → LinkCheckAns
       ├── DevStatusComponent → DevStatusReq
       ├── DutyCycleComponent (passivo)
       └── NewChannelComponent → NewChannelReq
```

---

## 1. DownlinkPacket e DownlinkScheduler

### 1.1 Classe `DownlinkPacket`

**Arquivo:** `network_server/scheduler.py:12`

| Atributo | Tipo | Valores | Descrição |
|---|---|---|---|
| `device_id` | str/int | qualquer | ID do device destino |
| `payload` | bytes \| MACCommand | `b''` default | Carga útil ou objeto MAC command |
| `packet_type` | str | `"ack"`, `"mac_command"`, `"app_data"` | Tipo para auto-detecção de prioridade |

```python
class DownlinkPacket:
    def __init__(self, device_id, payload=None, packet_type="ack"):
        self.device_id = device_id
        self.payload = payload or b''
        self.packet_type = packet_type
```

### 1.2 Classe `DownlinkScheduler`

**Arquivo:** `network_server/scheduler.py:20`

Implementa uma **min-heap de prioridades** usando `heapq`. Cada entrada na heap é uma tupla:

```
(priority, counter, device_id, packet)
```

O `counter` é um **tiebreaker monotônico** (FIFO dentro da mesma prioridade), evitando comparações diretas entre objetos `DownlinkPacket`.

#### Níveis de Prioridade

| Constante | Valor | Tipo de Pacote | Justificativa |
|---|---|---|---|
| `PRIORITY_ACK` | 0 | `"ack"` | Confirmação de uplink confirmado — tempo crítico |
| `PRIORITY_MAC_COMMAND` | 1 | `"mac_command"` | ADR, link check etc — importante mas pode esperar RX2 |
| `PRIORITY_APP_DATA` | 2 | `"app_data"` | Dados de aplicação — melhor esforço |

#### Atributos de Estado

| Atributo | Tipo | Descrição |
|---|---|---|
| `queue` | list (heap) | Fila interna `heapq` |
| `_counter` | int | Tiebreaker FIFO monotônico |
| `total_scheduled` | int | Total de pacotes enfileirados |
| `total_sent` | int | Total entregues via `get_next()` |
| `total_dropped` | int | Pacotes descartados (DL-DL ou timeout) |

#### Métodos

**`schedule(device_id, packet, priority=None)`**

Auto-detecta prioridade via `packet.packet_type` se `priority=None`:

```python
def schedule(self, device_id, packet, priority=None):
    if priority is None:
        if packet.packet_type == "ack":
            priority = self.PRIORITY_ACK         # 0
        elif packet.packet_type == "mac_command":
            priority = self.PRIORITY_MAC_COMMAND  # 1
        else:
            priority = self.PRIORITY_APP_DATA     # 2
    self._counter += 1
    heapq.heappush(self.queue, (priority, self._counter, device_id, packet))
    self.total_scheduled += 1
```

**`get_next(device_id=None)`**

- `device_id=None` → pop global do topo da heap (pacote de maior prioridade geral)
- `device_id=X` → busca linear `O(N)` pelo device, re-heapifica após remoção

```python
# Para device específico:
for i, (prio, cnt, dev_id, packet) in enumerate(self.queue):
    if dev_id == device_id:
        self.queue.pop(i)
        heapq.heapify(self.queue)   # O(N) — necessário após remoção arbitrária
        self.total_sent += 1
        return dev_id, packet
```

> **Nota de complexidade:** `heapq.heapify` após `pop(i)` tem custo `O(N)`. Na prática, como `N` é pequeno (1 pacote pendente por device), o impacto é desprezível.

**`has_pending(device_id=None)`**

```python
def has_pending(self, device_id=None):
    if device_id is None:
        return len(self.queue) > 0
    return any(dev_id == device_id for (_, _, dev_id, _) in self.queue)
```

**`stats()`** → retorna dict com `total_scheduled`, `total_sent`, `total_dropped`, `pending`.

---

## 2. Agendamento no NetworkServer

**Arquivo:** `network_server/server.py:90`

`on_uplink_received()` decide o que enfileirar após processar cada uplink:

```python
def on_uplink_received(self, packet, gateway):
    # 1. Valida MIC (OTAA)
    # 2. Valida frame counter
    # 3. Atualiza DeviceRegistry
    # 4. Executa NetworkController → retorna lista de MAC commands
    mac_commands = self.controller.on_new_packet(packet, status)

    # 5. Agenda ACK se pacote confirmado e não colidido
    if packet.confirmed and not packet.collided:
        ack = DownlinkPacket(packet.device_id, packet_type="ack")
        self.dl_scheduler.schedule(packet.device_id, ack)
        status.needs_ack = True

    # 6. Agenda cada MAC command como pacote separado
    if mac_commands:
        for cmd in mac_commands:
            mc_pkt = DownlinkPacket(packet.device_id, payload=cmd, packet_type="mac_command")
            self.dl_scheduler.schedule(packet.device_id, mc_pkt)

    return mac_commands
```

### Interação com NetworkController

O `NetworkController` itera todos os componentes registrados:

```python
# controller.py
def on_new_packet(self, packet, device_status):
    all_commands = []
    for component in self.components:
        cmds = component.on_packet(packet, device_status)
        all_commands.extend(cmds)
    return all_commands
```

Componentes que geram MAC commands de downlink:

| Componente | Condição de Geração | MAC Command Gerado |
|---|---|---|
| `ADRComponent` | Histórico ≥ `ADR_HISTORY_SIZE` e SF/power diferem | `LinkAdrReq(DR, TxPower, ChMask)` |
| `LinkCheckComponent` | Device fez `LinkCheckReq` (`request_link_check()`) | `LinkCheckAns(margin, gw_count)` |
| `DevStatusComponent` | Periodicamente (a cada N pacotes) | `DevStatusReq()` |
| `NewChannelComponent` | Configuração de canal adicional | `NewChannelReq(ch_index, freq, min_dr, max_dr)` |
| `DutyCycleComponent` | Passivo — não gera MAC commands | — |

---

## 3. Janela RX1

**Arquivo:** `network.py:595`, evento `EventType.RX1_WINDOW_OPEN`

**Timing:** `t_tx_end + receive_delay1` (1 segundo, padrão EU868)

### Algoritmo RX1

```
_on_rx1_open(device, packet):
  1. device.radio_state = RadioState.RX1
  2. SE dl_scheduler.has_pending(device_id):
       a. best_gw = gateway_manager.select_best_for_downlink()
       b. SE best_gw não é None:
            dl_airtime = _calc_dl_airtime(packet.sf, packet.bw)  # SF do uplink
            c. SE can_send_downlink(best_gw.gw_id, time, dl_airtime, "rx1"):
                 dl_packet = ns.get_downlink(device_id)
                 d. SE _network_dl_busy[packet.freq] > time:  # DL-DL collision
                      re-schedule → dl_scheduler.schedule(dl_packet)  # volta para fila
                      → cai para RX2
                 e. SENÃO (sem colisão):
                      gateway_manager.record_downlink("rx1")
                      best_gw.dl_busy_until[packet.freq] = time + dl_airtime
                      _network_dl_busy[packet.freq] = time + dl_airtime
                      SE ack → packet.ack_received = True
                      SE payload MAC → _apply_mac_commands(device, [cmd])
                      device.retransmission_attempts = 0
                      device.adr_ack_cnt = 0
                      device.radio_state = RadioState.IDLE
                      RETURN  ← DL entregue, RX2 cancelada
  3. SENÃO (sem DL ou GW ocupado):
       device.radio_state = RadioState.WAIT_RX2
       scheduler.schedule(trx1 + delay_to_rx2, RX2_WINDOW_OPEN, _on_rx2_open)
```

### SF na Janela RX1

O downlink RX1 usa o **mesmo SF do uplink** (`packet.sf`). O cálculo do airtime DL:

```python
def _calc_dl_airtime(self, sf, bw):
    # Airtime estimado para payload DL típico (~13 bytes: PHYPayload mínimo)
    # Usa mesma fórmula ToA do SX1272
    return device_ref.calculate_airtime(sf=sf, bw=bw, pl=13)
```

---

## 4. Janela RX2

**Arquivo:** `network.py:637`, evento `EventType.RX2_WINDOW_OPEN`

**Timing:** `t_tx_end + receive_delay2` (2 segundos, padrão EU868)

### Parâmetros RX2 por Região

| Região | Frequência RX2 | DR RX2 | SF RX2 | BW |
|---|---|---|---|---|
| EU868 | 869.525 MHz | DR0 | SF12 | 125 kHz |
| US915 | 923.3 MHz | DR8 | SF12 (500k) | 500 kHz |
| AU915 | 923.3 MHz | DR8 | SF12 (500k) | 500 kHz |
| AS923 | 923.2 MHz | DR2 | SF10 | 125 kHz |

Os parâmetros podem ser **sobrescritos por `RXParamSetupReq` (MAC 0x05)**:

```python
rx2_freq = getattr(device, '_rx2_freq', None) or self.region.rx2_frequency
if getattr(device, '_rx2_sf', None) is not None:
    rx2_sf = device._rx2_sf
    rx2_bw = getattr(device, '_rx2_bw', None) or self.region.dr_to_sf_bw(self.region.rx2_dr)[1]
else:
    rx2_sf, rx2_bw = self.region.dr_to_sf_bw(self.region.rx2_dr)
```

### Algoritmo RX2

Idêntico ao RX1, exceto:
- Usa `rx2_freq`, `rx2_sf`, `rx2_bw` (fixos por região ou configurados via MAC)
- Duty cycle verificado com `window="rx2"` → limite 10% (vs 1% no RX1)
- Após entrega bem-sucedida: `device.radio_state = RadioState.SLEEP` → `IDLE`
- Após falha (sem DL): verifica retransmissão exponential backoff

```python
# Retransmissão após RX2 falha
if packet.collided and device.retransmission_attempts < device.max_retransmissions:
    backoff = random.uniform(1, 10) * (2 ** device.retransmission_attempts)
    device.retransmission_attempts += 1
    self.scheduler.schedule(backoff, EventType.DEVICE_SEND, self._on_retransmit, device)
```

**Fórmula de backoff:**
```
t_retx = Uniform(1, 10) × 2^attempt
```

| `attempt` | Backoff mínimo | Backoff máximo |
|---|---|---|
| 0 | 1 s | 10 s |
| 1 | 2 s | 20 s |
| 2 | 4 s | 40 s |
| 3 | 8 s | 80 s |

---

## 5. GatewayManager — Seleção e Duty Cycle

**Arquivo:** `network_server/gateway_manager.py`

### 5.1 Seleção de Gateway

```python
def select_best_for_downlink(self, device_id, device_registry):
    status = device_registry.get_status(device_id)
    # Itera todos os GWs que receberam uplinks do device
    # Retorna o GW com maior SNR de uplink registrado
    best_gw_id, best_snr = None, -999
    for gw_id, gw_info in status.gateways.items():
        snr = gw_info.get("snr") or -999
        if snr > best_snr:
            best_gw_id, best_snr = gw_id, snr
    return self.gateways[best_gw_id].gateway
```

**Critério:** maximizar SNR de uplink como proxy para qualidade do enlace de downlink (reciprocidade de canal em TDD).

### 5.2 Verificação de Duty Cycle

**Fórmula do off-time obrigatório:**

```
off_time = airtime / dc_limit - airtime = airtime × (1/dc_limit - 1)
```

Condição de permissão:

```
current_time ≥ last_tx_time[window] + off_time
```

```python
def can_send_downlink(self, gw_id, current_time, airtime, window="rx1"):
    dc_limit = self.rx1_dc if window == "rx1" else self.rx2_dc
    off_time = airtime / dc_limit - airtime
    last_tx = gw_status.last_tx_time.get(window, 0)
    return current_time >= last_tx + off_time
```

| Janela | Limite DC | Off-time para airtime=0,1 s |
|---|---|---|
| RX1 | 1% | 9,9 s |
| RX2 | 10% | 0,9 s |

### 5.3 Registro de Downlink

```python
def record_downlink(self, gw_id, current_time, airtime, window="rx1"):
    gw_status.last_tx_time[window] = current_time + airtime
    gw_status.total_dl_sent += 1
```

`last_tx_time[window]` é atualizado para `t_end_dl = current_time + airtime`, representando o fim da transmissão downlink (não o início).

---

## 6. Bloqueio DL-DL (Half-Duplex de Rede)

**Estrutura:** `Network._network_dl_busy: dict[float, float]` — `freq → busy_until`

Após cada DL entregue:

```python
best_gw.dl_busy_until[freq] = time + dl_airtime      # bloqueia UL no GW (G8)
self._network_dl_busy[freq] = time + dl_airtime        # bloqueia outro DL na mesma freq
```

Verificação antes de entregar DL:

```python
if self._network_dl_busy.get(packet.freq, 0) > time:
    # DL-DL collision — re-schedule para próxima janela
    self.ns.dl_scheduler.schedule(device.device_id, dl_packet)
```

**Interação com G8:** `dl_busy_until` bloqueia recepção de uplinks no gateway na mesma frequência enquanto há DL transmitindo (half-duplex do SX1301).

---

## 7. MAC Commands — Catálogo Completo

**Arquivo:** `mac_commands.py`

### 7.1 Uplink (ED → NS)

| CID | Classe | Campos | Descrição |
|---|---|---|---|
| 0x02 | `LinkCheckReq` | — | Device solicita verificação de cobertura |
| 0x03 | `LinkAdrAns` | `power_ack`, `data_rate_ack`, `channel_mask_ack` | Resposta ao LinkAdrReq |
| 0x04 | `DutyCycleAns` | — | Confirma DutyCycleReq |
| 0x05 | `RXParamSetupAns` | `rx1_dr_offset_ack`, `rx2_dr_ack`, `channel_ack` | Confirma RXParamSetupReq |
| 0x06 | `DevStatusAns` | `battery` (0-255), `margin` (-32 a +31 dB) | Reporta bateria e SNR margin |
| 0x07 | `NewChannelAns` | `dr_range_ok`, `channel_freq_ok` | Confirma NewChannelReq |
| 0x08 | `RXTimingSetupAns` | — | Confirma RXTimingSetupReq |
| 0x0A | `DlChannelAns` | `uplink_freq_exists`, `channel_freq_ok` | Confirma DlChannelReq |

### 7.2 Downlink (NS → ED)

| CID | Classe | Campos | Descrição |
|---|---|---|---|
| 0x02 | `LinkCheckAns` | `margin` (dB), `gw_count` | SNR margin acima do mínimo; N° de GWs que receberam |
| 0x03 | `LinkAdrReq` | `data_rate`, `tx_power`, `ch_mask`, `nb_trans` | Ajuste de DR, potência e máscara de canais |
| 0x04 | `DutyCycleReq` | `max_dc_cycle` (0-15) | Limite agregado: `DC = 1/2^max_dc_cycle` |
| 0x05 | `RXParamSetupReq` | `rx1_dr_offset` (0-5), `rx2_dr`, `rx2_frequency` | Configura offsets RX1 e parâmetros RX2 |
| 0x06 | `DevStatusReq` | — | Solicita relatório de bateria/SNR |
| 0x07 | `NewChannelReq` | `ch_index`, `frequency`, `min_dr`, `max_dr` | Adiciona/modifica canal LoRa |
| 0x08 | `RXTimingSetupReq` | `delay` (1-15 s) | Ajusta delay da janela RX1 |
| 0x0A | `DlChannelReq` | `ch_index`, `frequency` | Define frequência exclusiva de DL |

### 7.3 `MACCommandProcessor.process_downlink_commands()`

**Arquivo:** `mac_commands.py:180`

Aplica cada MAC command recebido no downlink ao objeto `EndDevice`:

```python
def process_downlink_commands(self, device, commands):
    for cmd in commands:
        if isinstance(cmd, LinkAdrReq):
            # DR = 12 - SF; valida 7 ≤ SF ≤ 12, 2 ≤ TxPower ≤ 14 dBm
            new_sf = 12 - cmd.payload["data_rate"]
            device.sf = new_sf
            device.tx_power = cmd.payload["tx_power"]
            device.airtime = device.calculate_airtime()   # recalcula ToA

        elif isinstance(cmd, DutyCycleReq):
            max_dc = cmd.payload["max_dc_cycle"]
            device._max_duty_cycle = None if max_dc == 0 else 1.0 / (2 ** max_dc)

        elif isinstance(cmd, RXParamSetupReq):
            device._rx1_dr_offset = cmd.payload["rx1_dr_offset"]  # 0-5
            device._rx2_sf = 12 - cmd.payload["rx2_dr"]
            device._rx2_freq = cmd.payload["rx2_frequency"]

        elif isinstance(cmd, DevStatusReq):
            # Cria DevStatusAns com battery (0-254, linear de SoC%) e SNR margin
            battery = max(1, min(254, int(device.battery.soc_percent() * 254 / 100)))
            snr_margin = min(31, max(-32, int(device.snr)))
            return [DevStatusAns(battery, snr_margin)]

        elif isinstance(cmd, NewChannelReq):
            device._available_channels.append(cmd.payload["frequency"])

        elif isinstance(cmd, RXTimingSetupReq):
            device._rx1_delay = cmd.payload["delay"]

        elif isinstance(cmd, DlChannelReq):
            return [DlChannelAns()]
```

---

## 8. Network._apply_mac_commands()

**Arquivo:** `network.py`, chamado em `_on_device_send` e nos handlers RX1/RX2

```python
# _on_device_send (linha ~536):
mac_commands = self.ns.on_uplink_received(packet, best_gateway)
if mac_commands:
    self._apply_mac_commands(device, mac_commands)

# _on_rx1_open / _on_rx2_open:
if dl_packet.payload and hasattr(dl_packet.payload, 'cid'):
    self._apply_mac_commands(device, [dl_packet.payload])
```

`_apply_mac_commands` delega ao `MACCommandProcessor.process_downlink_commands()`, que modifica os atributos do `EndDevice` in-place.

**Duplo caminho de entrega de MAC commands:**
1. **Inline no uplink** — `on_uplink_received()` retorna `mac_commands`; `_apply_mac_commands()` é chamado imediatamente no `_on_device_send`
2. **Via RX window** — MAC command é enfileirado no `dl_scheduler` e entregue na próxima janela RX1 ou RX2 aberta

> Na simulação, o caminho 1 é predominante (execução síncrona). O caminho 2 é usado quando o ACK precisa ser combinado com o MAC command num único frame DL.

---

## 9. DutyCycleComponent (NS-side)

**Arquivo:** `network_server/components/duty_cycle.py`

Componente passivo — não gera MAC commands automaticamente, mas expõe APIs para verificação e tracking.

### Estrutura de Sub-bandas EU868

```python
sub_band_usage = {
    (863.0, 868.0): last_tx_end,   # 1%
    (868.0, 868.6): last_tx_end,   # 1%
    (868.7, 869.2): last_tx_end,   # 0.1%
    (869.4, 869.65): last_tx_end,  # 10%  ← RX2
}
```

### can_transmit()

```python
def can_transmit(self, frequency, current_time, airtime):
    sub_band = self._get_sub_band(frequency)
    dc_limit = self.region.duty_cycle.get(sub_band, 0.01)
    last_tx_end = self.sub_band_usage.get(sub_band, 0)
    required_off_time = airtime / dc_limit - airtime
    return current_time >= last_tx_end + required_off_time
```

### get_next_available_time()

```python
def get_next_available_time(self, frequency, current_time, airtime):
    sub_band = self._get_sub_band(frequency)
    dc_limit = self.region.duty_cycle.get(sub_band, 0.01)
    last_tx_end = self.sub_band_usage.get(sub_band, 0)
    return last_tx_end + airtime / dc_limit - airtime
```

---

## 10. Diagrama de Sequência Completo

```
DEVICE                  NETWORK               GATEWAY_MANAGER    DL_SCHEDULER
  │                       │                        │                   │
  │──DEVICE_SEND──────────►│                        │                   │
  │  (uplink t=T)          │                        │                   │
  │                       │──on_uplink_received()───►                   │
  │                       │  [MIC, FC, Registry]    │                   │
  │                       │  [NetworkController]    │                   │
  │                       │  confirmed=True?        │                   │
  │                       │──────schedule(ACK, p=0)─────────────────────►
  │                       │  mac_commands?          │                   │
  │                       │──────schedule(MAC, p=1)─────────────────────►
  │                       │                        │                   │
  │──TX_END───────────────►│                        │                   │
  │  t = T + airtime       │                        │                   │
  │                       │  [channel.evaluate_reception()]             │
  │                       │  [release_paths()]      │                   │
  │                       │──schedule(RX1, Δ=1s)   │                   │
  │                       │                        │                   │
  │ ◄──RX1_WINDOW_OPEN────│                        │                   │
  │  t = T + airtime + 1s  │                        │                   │
  │                       │──has_pending()──────────────────────────────►
  │                       │ ◄───────────────────────────────── True ────│
  │                       │──select_best_for_downlink()────►            │
  │                       │ ◄──────────────────── best_gw ──│           │
  │                       │──can_send_downlink(rx1)─────────►           │
  │                       │ ◄──────────────────── True/False│           │
  │                       │                                             │
  │   [dc OK, no DL-DL]   │──get_downlink()─────────────────────────────►
  │                       │ ◄──────────────────────────────── packet ───│
  │                       │──record_downlink(rx1)──►                    │
  │                       │──dl_busy_until[freq]=t+airtime              │
  │                       │──_network_dl_busy[freq]=t+airtime           │
  │ ◄──ACK/MAC delivered──│                        │                   │
  │  radio_state = IDLE    │                        │                   │
  │                       │                        │                   │
  │ [RX1 failed → RX2]    │──schedule(RX2, Δ=1s)  │                   │
  │                       │                        │                   │
  │ ◄──RX2_WINDOW_OPEN────│  SF12 @ 869.525 MHz    │                   │
  │  t = T + airtime + 2s  │  [mesma lógica RX1]    │                   │
  │                       │  can_send_downlink(rx2) → 10% DC           │
  │ ◄──ACK/MAC delivered──│                        │                   │
  │  radio_state = SLEEP → IDLE                    │                   │
  │                       │                        │                   │
  │ [RX2 falhou, collided] │                        │                   │
  │──retransmit (backoff)──►                        │                   │
```

---

## 11. Exemplos Numéricos

### Exemplo 1 — ACK em RX1, duty cycle OK

Contexto: EU868, SF9, BW=125 kHz, pacote confirmado, t_tx_end=10,0 s

- Airtime uplink: `T_sym = 2^9/125000 = 4,096 ms`; ToA ≈ 328,7 ms
- t_RX1 = 10,0 + 0,3287 + 1,0 = **11,3287 s**
- dl_airtime (SF9, payload=13 B) ≈ 205,8 ms
- GW last_tx RX1 = 0 → off_time = 0,2058/0,01 – 0,2058 = **20,38 s**; 11,33 ≥ 0+20,38 → **Falso** → RX1 bloqueada
- Vai para RX2

### Exemplo 2 — ACK em RX2, duty cycle 10%

Continuando do Exemplo 1:
- t_RX2 = 10,0 + 0,3287 + 2,0 = **12,3287 s**
- dl_airtime RX2 (SF12, BW=125k, 13 B) ≈ 1318,0 ms
- off_time_rx2 = 1,318/0,10 – 1,318 = **11,86 s**; 12,33 ≥ 0+11,86 → **Verdadeiro** → DL entregue
- `record_downlink("rx2")`: `last_tx_time["rx2"] = 12,3287 + 1,318 = 13,647 s`
- Próximo DL em RX2 disponível: 13,647 + 11,86 = **25,51 s**

### Exemplo 3 — LinkAdrReq via ADR

Device SF12 com SNR médio = –5 dB após 20 pacotes:
- `snr_min_per_sf[12] = –20 dB`; `ADR_SNR_MARGIN_DB = 10 dB`
- `margin = –5 – (–20) – 10 = +5 dB`
- `n_steps = 5 / 3 = 1` (ADR_STEP_DB = 3)
- Passo 1: SF12 → **SF11**; n_steps = 0
- `LinkAdrReq(data_rate=1, tx_power=14, ch_mask=0xFFFF)` enfileirado (prioridade 1)
- Device recebe em RX1: `device.sf = 11`, `device.airtime` recalculado

### Exemplo 4 — DL-DL Collision

- Dois devices A e B transmitem quase simultaneamente na mesma frequência 868,1 MHz
- Device A → DL entregue em RX1: `_network_dl_busy[868.1] = 11,33 + 0,206 = 11,536 s`
- Device B → RX1 abre em t=11,4 s: `_network_dl_busy[868.1] = 11,536 > 11,4`
- Colisão DL-DL detectada → pacote B devolvido ao `dl_scheduler`
- Device B tenta RX2 em t=12,4 s em 869,525 MHz → sem colisão → entregue

### Exemplo 5 — Fila de Prioridades com Múltiplos Tipos

Fila após uplink confirmado que gera ADR:

```
heap = [
    (0, 1, "dev-01", DownlinkPacket("dev-01", type="ack")),        # ACK
    (1, 2, "dev-01", DownlinkPacket("dev-01", LinkAdrReq, "mac")), # MAC
]
```

`get_next("dev-01")` → retorna ACK (prioridade 0, counter 1).
Segunda chamada → retorna LinkAdrReq (prioridade 1, counter 2).
Se MAC command chegar antes: prioridade garante ACK sempre primeiro.

### Exemplo 6 — RXParamSetupReq modifica RX2

NS envia `RXParamSetupReq(rx1_dr_offset=0, rx2_dr=2, rx2_frequency=868.525)`:

Antes: `rx2_sf=12`, `rx2_freq=869.525 MHz`
Depois: `device._rx2_sf = 12 - 2 = 10`, `device._rx2_freq = 868.525 MHz`
DL airtime RX2 reduz de ≈1318 ms para ≈329 ms (SF10 vs SF12) → off-time 10% cai de 11,9 s para 3,0 s.

---

## 12. Comparação com Implementações de Referência

| Aspecto | PyLoRaWAN | ns-3 LoRaWAN | FLoRa (OMNeT++) |
|---|---|---|---|
| Fila de prioridades DL | Min-heap Python `heapq` | STL `priority_queue` | OMNeT++ `cQueue` |
| Seleção de GW | Max SNR uplink | Mais recente / SNR | RSSI |
| DC RX1 gateway | 1% (`gw_rx1_dc`) | 1% (ETSI g) | 1% |
| DC RX2 gateway | 10% (`gw_rx2_dc`) | 10% (ETSI g1) | 10% |
| RX2 EU868 padrão | SF12 @ 869.525 MHz | SF12 @ 869.525 MHz | SF12 @ 869.525 MHz |
| `RXParamSetupReq` | Suportado (`_rx2_sf`, `_rx2_freq`) | Suportado | Parcial |
| Colisão DL-DL | `_network_dl_busy` dict | Por canais sobrepostos | Não modelado |
| Backoff retransmissão | `Uniform(1,10) × 2^n` | Exponencial fixo | Não modelado |
| MAC commands gerados | ADR, LinkCheck, DevStatus, NewChannel | ADR, LinkCheck | ADR |

---

## 13. Referências

- **LoRaWAN 1.0.3 Specification**, §18 (Class A receive windows), §5 (MAC commands)
- **LoRaWAN 1.1 Specification**, §5.2 (DutyCycleReq), §5.5 (RXParamSetupReq), §5.7 (NewChannelReq)
- **ETSI EN 300 220-2** — sub-band duty cycle limits EU868 (g: 1%, g1: 10%)
- **Semtech SX1301 Datasheet** — concentrador 8 demoduladores, half-duplex UL/DL
- **Magrin et al., 2017** — "Performance evaluation of LoRa networks" (LoRaWANSim reference)
- **ns-3 lorawan module** — `network-server.cc`, `network-controller-components.cc`
- **FLoRa (OMNeT++)** — `NetworkServer.cc`, downlink scheduling implementation
