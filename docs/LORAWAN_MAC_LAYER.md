# LoRaWAN MAC Layer Modeling — PyLoRaWAN

## Visão Geral

A camada MAC implementa o protocolo LoRaWAN Class A através de uma máquina de estados finita (FSM) explícita em `enddevice.py`, coordenada por handlers de evento em `network.py`. Cada ciclo de transmissão percorre estados bem definidos, com janelas de recepção obrigatórias após o uplink. O dever de ciclo é aplicado por sub-banda com base nos parâmetros regionais. Todos os MAC commands LoRaWAN 1.1 relevantes são processados por `MACCommandProcessor` em `mac_commands.py`.

---

## 1. FSM de Estado de Rádio — `RadioState`

Definido em `enddevice.py` como enum Python:

```python
class RadioState(Enum):
    IDLE     = 0   # aguardando próximo ciclo de TX
    TX       = 1   # transmitindo uplink
    WAIT_RX1 = 2   # aguardando abertura da janela RX1
    RX1      = 3   # janela RX1 aberta
    WAIT_RX2 = 4   # aguardando abertura da janela RX2
    RX2      = 5   # janela RX2 aberta
    SLEEP    = 6   # deep sleep entre ciclos
```

Os estados são atribuídos diretamente pelos handlers de evento em `network.py` — não existe um método de transição centralizado chamado pelo loop principal (ver ENERGY_ACCOUNTING.md para `transition_state()` e por que ele não é usado no loop de eventos).

O estado inicial de cada device é `RadioState.IDLE` (`enddevice.py:55`).

---

## 2. Ciclo Class A — Sequência de Eventos

O ciclo completo de transmissão envolve quatro handlers encadeados em `network.py`:

```
DEVICE_SEND                PACKET_TX_END         RX1_WINDOW_OPEN      RX2_WINDOW_OPEN
     │                          │                       │                    │
 _on_device_send()         _on_tx_end()           _on_rx1_open()       _on_rx2_open()
     │                          │                       │                    │
 IDLE → TX                TX → WAIT_RX1          WAIT_RX1 → RX1       WAIT_RX2 → RX2
     │                          │                       │                    │
 Emite pacote              Avalia canal            Tenta DL NS          Fallback DL
 Agenda TX_END             Agenda RX1 (+1s)        Se não: WAIT_RX2     SLEEP
 Agenda próx SEND                                  Agenda RX2 (+trx1)
```

### 2.1 `_on_device_send()`

```python
# Verificações iniciais
if device.battery is not None and device.battery.depleted:
    return  # device esgotado

if time < device.dc_release_time:
    delay = device.dc_release_time - time + 0.001
    self.scheduler.schedule(delay, EventType.DEVICE_SEND, ...)
    return  # duty cycle não liberado
```

Após as verificações, o handler:
1. Seleciona canal (`device.select_channel()` — escolha aleatória de `_available_channels`)
2. Calcula airtime via `device.calculate_airtime()`
3. Chama `device.energy_model.update_energy(device, airtime, sim_time=time)`
4. Calcula e registra o próximo instante de liberação de duty cycle:
   ```python
   dc_limit = self.region.get_duty_cycle_limit(device.freq) or (ed_dc_limit_percent / 100.0)
   device.dc_release_time = time + airtime / dc_limit
   ```
5. Incrementa `device.frame_counter_up` e cria `Packet`
6. Agenda `PACKET_TX_END` em `+airtime` e próximo `DEVICE_SEND` com delay Poisson

### 2.2 `_on_tx_end()`

```python
device.radio_state = RadioState.WAIT_RX1
```

Avalia recepção pelo `ChannelModel` (SNR, colisões, G13/G14). Agenda:

```python
self.scheduler.schedule(receive_delay1, EventType.RX1_WINDOW_OPEN,
                        self._on_rx1_open, device, packet)
```

**`receive_delay1 = 1 s`** — parâmetro de `parametors.py`, importado diretamente.

### 2.3 `_on_rx1_open()`

```python
device.radio_state = RadioState.RX1
```

Verifica fila de downlink no NS (`self.ns.dl_scheduler.has_pending(device.device_id)`). Se houver downlink pendente, seleciona o melhor gateway e tenta entrega. Se a entrega ocorrer, o ciclo encerra aqui.

Se não houver DL em RX1:

```python
delay_to_rx2 = max(0, receive_delay2 - receive_delay1 - trx1)
device.radio_state = RadioState.WAIT_RX2
self.scheduler.schedule(trx1 + delay_to_rx2, EventType.RX2_WINDOW_OPEN,
                        self._on_rx2_open, device, packet)
```

Com os defaults EU868 (`receive_delay2=2`, `receive_delay1=1`, `trx1=1`):
```
delay_to_rx2 = max(0, 2 - 1 - 1) = 0
```
As janelas RX1 e RX2 ocorrem consecutivamente sem pausa entre elas.

### 2.4 `_on_rx2_open()`

```python
device.radio_state = RadioState.RX2
```

RX2 usa frequência e SF configurados por `RxParamSetupReq` se disponíveis, caso contrário os padrões regionais:

```python
rx2_freq = getattr(device, '_rx2_freq', None) or self.region.rx2_frequency
if getattr(device, '_rx2_sf', None) is not None:
    rx2_sf = device._rx2_sf
```

Para EU868: `rx2_frequency = 869.525 MHz`, `rx2_dr = 0` → SF12.

Após RX2, o device transita para SLEEP (contabilizado via `update_energy()`).

---

## 3. Temporização das Janelas

| Parâmetro | Valor padrão | Origem |
|---|---|---|
| `receive_delay1` | 1 s | `parametors.py` |
| `receive_delay2` | 2 s | `parametors.py` |
| `trx1` | 1 s | `parametors.py` (duração máx RX1) |
| `trx2` | 2 s | `parametors.py` (duração máx RX2) |

```
t₀                t₀+airtime   t₀+at+1s   t₀+at+2s   t₀+at+4s
│◄────── TX ─────►│◄─WAIT_RX1─►│◄── RX1 ──►│◄── RX2 ──►│◄── SLEEP
                              RX1 abre    RX2 abre    SLEEP
                              (+rd1=1s)   (+trx1=1s)
```

`RXTimingSetupReq` pode alterar `device._rx1_delay` para 1–15 s.

---

## 4. Cálculo de Airtime

`calculate_airtime()` em `enddevice.py`:

```python
def calculate_airtime(self):
    Tsym = (2.0 ** self.sf) / self.bw
    Tpream = (8 + 4.25) * Tsym
    DE = 1 if Tsym > 0.016 else 0       # Low Data Rate Optimization
    H, CRC = 0, 1                        # H=0: cabeçalho explícito; CRC habilitado
    numerator = 8.0 * self.pl - 4.0 * self.sf + 28 + 16 * CRC - 20 * H
    denominator = 4.0 * (self.sf - 2 * DE)
    payloadSymbNB = 8 + max(math.ceil(numerator / denominator) * (self.cr + 4), 0)
    return Tpream + payloadSymbNB * Tsym
```

DE=1 (Low Data Rate Optimization) é ativado automaticamente quando `Tsym > 16 ms`, conforme LoRaWAN — isto ocorre para SF11 e SF12 com BW 125 kHz.

---

## 5. Aplicação de Duty Cycle

### 5.1 Controle no End Device (`network.py`)

Após cada transmissão (TX inicial ou retransmissão), o instante de liberação é calculado:

```python
dc_limit = self.region.get_duty_cycle_limit(device.freq) or (ed_dc_limit_percent / 100.0)
device.dc_release_time = time + airtime / dc_limit
```

O off-time mínimo é `airtime × (1/dc_limit − 1)`. Exemplo: airtime = 0.5 s, dc_limit = 0.01 → off-time = 49.5 s.

No início de cada `_on_device_send()` e `_on_retransmit()`:

```python
if time < device.dc_release_time:
    delay = device.dc_release_time - time + 0.001
    self.scheduler.schedule(delay, EventType.DEVICE_SEND, ...)
    return
```

O event é reagendado automaticamente para o próximo instante válido — sem descarte de transmissão.

### 5.2 Sub-bandas EU868 (`regions.py`)

```python
duty_cycle = {
    (863.0, 868.6): 0.01,    # G/G1 — canais 868.1/868.3/868.5 e 867.x
    (868.7, 869.2): 0.001,   # G2 — 0.1%
    (869.4, 869.65): 0.10,   # G3 — 10% (canal RX2)
    (869.7, 870.0): 0.01,    # G4
}
```

`get_duty_cycle_limit(frequency)` percorre o dicionário e retorna o limite da sub-banda que contém a frequência; se nenhuma sub-banda corresponder, retorna `0.01`.

### 5.3 Outros Planos Regionais

| Região | Duty Cycle | Dwell Time |
|---|---|---|
| EU868 | Por sub-banda (ver acima) | Sem limite |
| US915 | Nenhum | 400 ms |
| AU915 | Nenhum | 400 ms |
| AS923 | `(923.0, 925.0): 0.01` | 400 ms |

### 5.4 `DutyCycleComponent` (Network Server)

`network_server/components/duty_cycle.py` mantém rastreamento server-side por sub-banda:

```python
def can_transmit(self, frequency, current_time, airtime):
    dc_limit = self.region.duty_cycle.get(sub_band, 0.01)
    last_tx_end = self.sub_band_usage.get(sub_band, 0)
    required_off_time = airtime / dc_limit - airtime
    return current_time >= last_tx_end + required_off_time

def record_transmission(self, frequency, current_time, airtime):
    self.sub_band_usage[sub_band] = current_time + airtime
```

`get_next_available_time()` retorna o instante exato de liberação sem polling.

---

## 6. Ativação de Dispositivo

Dois modos suportados, configurados por device:

| Modo | Atributo | Setup |
|---|---|---|
| OTAA | `activation_mode = "OTAA"` | `app_key`, `dev_eui`, `app_eui`; `nwk_skey`/`app_skey` derivados no JoinAccept |
| ABP | `activation_mode = "ABP"` | `setup_abp(dev_addr, nwk_skey, app_skey)` |

Frame counter uplink (`frame_counter_up`) é incrementado a cada TX. Frame counter downlink é validado por `validate_frame_counter_down(fc_down)` — rejeita valores não crescentes.

MIC é calculado se `device.nwk_skey is not None` e `device.dev_addr` é inteiro:

```python
if device.nwk_skey is not None and isinstance(device.dev_addr, int):
    from security import compute_frame_mic
    _frame_mic = compute_frame_mic(...)
```

---

## 7. MAC Commands

### 7.1 Tabela de CIDs

| CID | Downlink (NS→ED) | Uplink (ED→NS) |
|---|---|---|
| 0x02 | `LinkCheckAns` | `LinkCheckReq` |
| 0x03 | `LinkAdrReq` | `LinkAdrAns` |
| 0x04 | `DutyCycleReq` | `DutyCycleAns` |
| 0x05 | `RXParamSetupReq` | `RXParamSetupAns` |
| 0x06 | `DevStatusReq` | `DevStatusAns` |
| 0x07 | `NewChannelReq` | `NewChannelAns` |
| 0x08 | `RXTimingSetupReq` | `RXTimingSetupAns` |
| 0x0A | `DlChannelReq` | `DlChannelAns` |

### 7.2 `MACCommandProcessor`

`process_uplink_commands(device, commands)` — processa comandos recebidos do device:
- `LinkCheckReq` → gera `LinkCheckAns(margin, gw_count=1)` onde `margin = max(0, snr - snr_min)`
- `DevStatusAns` → registra status (passthrough)
- `LinkAdrAns` → registra confirmação (passthrough)

`process_downlink_commands(device, commands)` — aplica comandos recebidos do NS:

```python
for cmd in commands:
    if isinstance(cmd, LinkAdrReq):     resp = self._apply_link_adr(device, cmd)
    elif isinstance(cmd, DutyCycleReq): self._apply_duty_cycle(device, cmd); resp = DutyCycleAns()
    elif isinstance(cmd, RXParamSetupReq): resp = self._apply_rx_param_setup(device, cmd)
    elif isinstance(cmd, DevStatusReq): resp = self._create_dev_status_ans(device)
    elif isinstance(cmd, NewChannelReq): resp = self._apply_new_channel(device, cmd)
    elif isinstance(cmd, RXTimingSetupReq): self._apply_rx_timing(device, cmd); resp = RXTimingSetupAns()
    elif isinstance(cmd, DlChannelReq): resp = self._apply_dl_channel(device, cmd)
    elif isinstance(cmd, LinkCheckAns): self._handle_link_check_ans(device, cmd)
```

### 7.3 `_apply_link_adr()` — Ajuste de DR e Potência

```python
new_power = cmd.payload.get("tx_power", device.tx_power)
if 2 <= new_power <= 14:
    device.tx_power = new_power
else:
    power_ack = False

new_dr = cmd.payload.get("data_rate")
if new_dr is not None:
    new_sf = 12 - new_dr          # DR0=SF12, DR5=SF7
    if 7 <= new_sf <= 12:
        device.sf = new_sf
        device.airtime = device.calculate_airtime()   # recalcula airtime
    else:
        dr_ack = False
```

Usa `.get()` com fallback — acesso seguro sem `KeyError`. Recalcula `device.airtime` imediatamente após mudança de SF, mantendo o campo em sincronia.

Retorna `LinkAdrAns(power_ack, data_rate_ack, channel_mask_ack)`.

### 7.4 `_apply_duty_cycle()` — Limite Agregado

```python
max_dc = cmd.payload.get("max_dc_cycle", 0)
if max_dc == 0:
    device._max_duty_cycle = None       # sem limite adicional
else:
    device._max_duty_cycle = 1.0 / (2 ** max_dc)
```

`max_dc_cycle` de 1–15 produz limites de 1/2 a 1/32768. O campo `device._max_duty_cycle` é armazenado mas não integrado automaticamente ao cálculo de `dc_release_time` no loop principal — existe como extensão para uso customizado.

### 7.5 `_apply_rx_param_setup()` — Parâmetros RX

```python
# RX1 DR offset: 0-5
if 0 <= rx1_dr_offset <= 5:
    device._rx1_dr_offset = rx1_dr_offset

# RX2 DR: mapeado via SF = 12 - rx2_dr
rx2_sf = 12 - rx2_dr
if 7 <= rx2_sf <= 12:
    device._rx2_sf = rx2_sf
    device._rx2_bw = 125000

# RX2 frequência
if rx2_freq is not None and rx2_freq > 0:
    device._rx2_freq = rx2_freq
```

Os atributos `_rx2_sf`, `_rx2_bw`, `_rx2_freq` são lidos em `_on_rx2_open()` para substituir os defaults regionais.

### 7.6 `_create_dev_status_ans()` — Status de Bateria e SNR

```python
# Nível de bateria: 0=externo, 1-254=SoC%, 255=desconhecido
if device.battery is not None:
    soc = device.battery.soc_percent()
    battery = 0 if soc <= 0 else max(1, min(254, int(soc * 254 / 100)))
else:
    battery = 255

snr_margin = min(31, max(-32, int(device.snr))) if device.snr is not None else 0
```

### 7.7 `_apply_new_channel()` — Canal Adicional

Adiciona `frequency` a `device._available_channels` se não presente. `select_channel()` passa a incluir o novo canal na seleção aleatória.

### 7.8 `_apply_rx_timing()` — Delay RX1

```python
device._rx1_delay = delay   # 1-15 s
```

### 7.9 `_apply_dl_channel()` — Frequência DL

Retorna `DlChannelAns()` (confirmação sem efeito adicional na implementação atual).

---

## 8. Suporte Multi-Classe

`can_receive_downlink(current_time)` diferencia comportamento por classe:

```python
def can_receive_downlink(self, current_time):
    if self.lorawan_class == 'A':
        return self.radio_state in [RadioState.RX1, RadioState.RX2]
    elif self.lorawan_class == 'B':
        if not self.is_synchronized:
            return False
        time_since_beacon = current_time - self.last_beacon_time
        ping_phase = time_since_beacon % self.ping_period
        return ping_phase < 0.030   # janela ping de 30 ms
    elif self.lorawan_class == 'C':
        return self.radio_state != RadioState.TX
```

Class B requer sincronização de beacon (`process_beacon()` seta `is_synchronized = True`) e usa `ping_period=32 s` com `ping_offset` aleatório. Class C mantém recepção contínua exceto durante TX — corrente idle = `rx_current_ma` (11.2 mA) em vez de `sleep_current_ma` (0.0015 mA).

---

## 9. Fluxo Completo de um Ciclo Class A

```
DEVICE_SEND @ t₀
  │
  ├── battery.depleted? → return
  ├── t₀ < dc_release_time? → reagenda com delay
  ├── select_channel()           → device.freq = random.choice(_available_channels)
  ├── calculate_airtime()        → Tsym, Tpream, payloadSymbNB, airtime
  ├── update_energy(airtime, t₀) → sleep gap + TX + STANDBY + RX1 + STANDBY + RX2 + SLEEP
  ├── dc_release_time = t₀ + airtime / dc_limit
  ├── frame_counter_up += 1
  ├── Packet(device_id, sf, tx_power, bw, freq, rssi, t₀, airtime)
  ├── scheduler.schedule(airtime, PACKET_TX_END, _on_tx_end)
  └── scheduler.schedule(next_poisson, DEVICE_SEND, _on_device_send)

PACKET_TX_END @ t₀ + airtime
  │
  ├── device.radio_state = WAIT_RX1
  ├── channel.evaluate_reception(packet, on_air)   → snr, sinr, collided
  └── scheduler.schedule(1s, RX1_WINDOW_OPEN, _on_rx1_open)

RX1_WINDOW_OPEN @ t₀ + airtime + 1s
  │
  ├── device.radio_state = RX1
  ├── ns.dl_scheduler.has_pending(device_id)?
  │     Sim → select_best_for_downlink → GW duty cycle OK?
  │           Sim → entrega DL, _apply_mac_commands(device, mac_cmds) → fim ciclo
  │           Não → (GW DC bloqueado)
  └── device.radio_state = WAIT_RX2
      scheduler.schedule(trx1 + 0s, RX2_WINDOW_OPEN, _on_rx2_open)

RX2_WINDOW_OPEN @ t₀ + airtime + 2s
  │
  ├── device.radio_state = RX2
  ├── rx2_freq = device._rx2_freq or region.rx2_frequency (EU868: 869.525 MHz)
  ├── rx2_sf  = device._rx2_sf  or region.rx2_dr→SF (EU868: SF12)
  ├── tenta DL fallback
  └── device.radio_state → SLEEP (implícito — próximo TX é SLEEP no energy model)
```

---

## 10. Parâmetros MAC Configuráveis

| Parâmetro | Arquivo | Default | Efeito |
|---|---|---|---|
| `receive_delay1` | `parametors.py` | 1 s | Abertura da janela RX1 após TX_END |
| `receive_delay2` | `parametors.py` | 2 s | Abertura da janela RX2 após TX_END |
| `trx1` | `parametors.py` | 1 s | Duração máxima da janela RX1 |
| `trx2` | `parametors.py` | 2 s | Duração máxima da janela RX2 |
| `ed_dc_limit_percent` | `parametors.py` | 1% | Limite de DC fallback (sem sub-banda) |
| `region` | `network.py` | `EU868` | Define sub-bandas, RX2 params, DR table |
| `device._rx1_delay` | via `RXTimingSetupReq` | = receive_delay1 | Delay RX1 por device |
| `device._rx2_sf` | via `RXParamSetupReq` | None → regional | SF da janela RX2 |
| `device._rx2_freq` | via `RXParamSetupReq` | None → regional | Frequência da janela RX2 |
| `device._max_duty_cycle` | via `DutyCycleReq` | None | Limite DC agregado adicional |
