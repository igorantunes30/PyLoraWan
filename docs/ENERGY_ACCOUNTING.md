# Energy Accounting — PyLoRaWAN

## Visão Geral

Durante a simulação, cada End Device rastreia seu estado operacional. Sempre que um device transita entre estados — transmissão, recepção, standby ou sleep — o módulo de energia atualiza o consumo acumulado. O consumo é calculado a partir da corrente característica de cada estado e da duração de permanência nele. Esse mecanismo permite estimar o consumo total e o tempo de vida da bateria de cada device.

---

## 1. Dois Conjuntos de Estados

O simulador mantém **dois conjuntos de estados** distintos para cada device:

### 1.1 Estados MAC — `enddevice.RadioState` (7 estados)

Definidos em `enddevice.py`, representam o estado lógico do protocolo LoRaWAN Class A:

```python
class RadioState(Enum):
    IDLE     = 0   # aguardando próximo ciclo
    TX       = 1   # transmitindo uplink
    WAIT_RX1 = 2   # aguardando abertura da janela RX1
    RX1      = 3   # janela RX1 aberta
    WAIT_RX2 = 4   # aguardando abertura da janela RX2
    RX2      = 5   # janela RX2 aberta
    SLEEP    = 6   # sleep profundo
```

### 1.2 Estados de Energia — `energymodel.RadioState` (4 estados)

Definidos em `energymodel.py`, representam o estado físico do rádio SX1272:

```python
class RadioState(Enum):
    SLEEP   = auto()   # 0.0015 mA
    STANDBY = auto()   # 1.4 mA
    TX      = auto()   # 22.3–38.0 mA (depende da potência)
    RX      = auto()   # 11.2 mA
```

### 1.3 Mapeamento MAC → Energia

O `EndDevice` define `_ENERGY_STATE_MAP` para traduzir estados MAC nos estados físicos de energia:

```python
_ENERGY_STATE_MAP = {
    "IDLE":     "STANDBY",   # rádio acordado, oscilador ativo → 1.4 mA
    "TX":       "TX",        # transmissão ativa
    "WAIT_RX1": "STANDBY",   # aguardando janela — rádio não dorme
    "RX1":      "RX",        # recepção ativa
    "WAIT_RX2": "STANDBY",   # aguardando janela — rádio não dorme
    "RX2":      "RX",        # recepção ativa
    "SLEEP":    "SLEEP",     # deep sleep entre ciclos
}
```

`IDLE`, `WAIT_RX1` e `WAIT_RX2` mapeiam para `STANDBY` porque o rádio mantém o oscilador ativo para abrir a janela no momento exato — não há transição para sleep entre TX e RX.

---

## 2. Primitiva de Contabilização — `transition()`

O núcleo do modelo está em `EnergyModel.transition()` (`energymodel.py`). Ele é chamado **ao sair de um estado**: contabiliza a energia do estado que está encerrando, depois registra a entrada no novo estado.

```python
def transition(self, new_state, time, tx_power=None):
    duration = max(0, time - self.state_entry_time)

    # corrente do estado que está SAINDO
    if self.current_state == RadioState.TX:
        current_ma = tx_current_table.get(self._current_tx_power, 38)
    else:
        current_ma = STATE_CURRENT_MA[self.current_state]

    # E = I × V × t   [mA × V × s = mJ]
    energy = current_ma * self.voltage * duration

    self.energy_consumed                      += energy
    self.energy_breakdown[self.current_state] += energy
    self.state_durations[self.current_state]  += duration

    if self._battery is not None and energy > 0:
        self._battery.consume(energy)          # debita da bateria

    self.current_state    = new_state
    self.state_entry_time = time
    if tx_power is not None:
        self._current_tx_power = tx_power

    return energy   # mJ do estado encerrado
```

**Regra fundamental:** a energia é sempre contabilizada na **transição de saída**, não na entrada. Assim, nenhuma energia é perdida entre eventos.

---

## 3. Onde a Contabilização é Disparada

### 3.1 Caminho principal — `update_energy()` em batch

Na simulação, a contabilização de energia acontece **uma vez por transmissão**, no início do evento `DEVICE_SEND`, em `network.py`:

```python
# network.py — _on_device_send()
device.energy_model.update_energy(device=device, airtime=airtime, sim_time=time)
```

`update_energy()` executa as seis transições do ciclo Class A em sequência, usando durações fixas dos parâmetros globais:

```
t₀            t₀+airtime   t₀+airtime+1s  t₀+airtime+2s  t₀+airtime+3s  t₀+airtime+5s
│◄─── TX ────►│◄─ STANDBY ►│◄──── RX1 ───►│◄─ STANDBY ──►│◄──── RX2 ───►│◄─ SLEEP ──►
│  22–38 mA   │   1.4 mA   │   11.2 mA    │   1.4 mA (0s)│   11.2 mA    │ 0.0015 mA
│  airtime(SF)│    rd1=1s  │    trx1=1s   │ rd2_eff=0s   │    trx2=2s   │ até prox TX
```

> `rd2_eff = max(0, receive_delay2 − receive_delay1 − trx1) = max(0, 2−1−1) = 0` com os defaults EU868. As janelas RX1 e RX2 ocorrem consecutivamente.

As seis chamadas internas a `transition()` são:

```python
t = self.state_entry_time

self.transition(RadioState.TX,      t, tx_power=device.tx_power)  ; t += airtime
self.transition(RadioState.STANDBY, t)                             ; t += receive_delay1
self.transition(RadioState.RX,      t)                             ; t += trx1
self.transition(RadioState.STANDBY, t)                             ; t += rd2_eff
self.transition(RadioState.RX,      t)                             ; t += trx2
self.transition(RadioState.SLEEP,   t)
```

O mesmo fluxo é executado na retransmissão (`_on_retransmit()`):

```python
# network.py — _on_retransmit()
device.energy_model.update_energy(device=device, airtime=airtime, sim_time=time)
```

### 3.2 Contabilização do sleep entre ciclos

`update_energy()` recebe `sim_time` — o timestamp real da simulação no início do TX. Antes de lançar o ciclo, ele fecha o gap de sleep desde o fim do último ciclo:

```python
if sim_time is not None and sim_time > self._sim_time_last_tx_end:
    sleep_duration = sim_time - self._sim_time_last_tx_end
    sleep_energy   = 0.0015 × 3.3 × sleep_duration     # mJ
    self.energy_consumed                    += sleep_energy
    self.energy_breakdown[RadioState.SLEEP] += sleep_energy
    self.state_durations[RadioState.SLEEP]  += sleep_duration
    if self._battery is not None:
        self._battery.consume(sleep_energy)
```

Após o ciclo, o marcador é atualizado:

```python
self._sim_time_last_tx_end = sim_time + airtime + receive_delay1 + trx1 + rd2_eff + trx2
```

No primeiro TX do device, `_sim_time_last_tx_end = 0.0`, portanto o sleep desde o instante zero até o primeiro TX também é contabilizado.

### 3.3 `transition_state()` — caminho alternativo (não usado pelo loop de eventos)

`EndDevice` expõe `transition_state(new_state, time)` que mapeia estado MAC → estado de energia e chama `energy_model.transition()` diretamente. Este método existe para uso externo (testes, extensões), mas **não é chamado pelos handlers de evento** de `network.py`. Os handlers atribuem `device.radio_state` diretamente; toda a energia é capturada via `update_energy()`.

```python
# enddevice.py — disponível, mas não usado no loop de simulação
def transition_state(self, new_state, time):
    energy_state_name = self._ENERGY_STATE_MAP.get(new_state.name, "SLEEP")
    self.energy_model.transition(EnergyState[energy_state_name], time)
    self.radio_state = new_state
```

> Não combinar `update_energy()` e `transition_state()` no mesmo ciclo para o mesmo device — resultaria em dupla contabilização.

---

## 4. Acumuladores por Device

Cada `EnergyModel` mantém três acumuladores independentes, atualizados a cada chamada a `transition()`:

| Acumulador | Tipo | Conteúdo |
|---|---|---|
| `energy_consumed` | `float` (mJ) | Total acumulado desde o início da simulação |
| `energy_breakdown` | `dict {RadioState → float}` | mJ por estado físico (SLEEP, STANDBY, TX, RX) |
| `state_durations` | `dict {RadioState → float}` | Segundos acumulados por estado |

Acesso direto:

```python
device.energy_model.energy_consumed             # mJ total
device.energy_model.energy_breakdown            # {RadioState.TX: x, ...}
device.energy_model.get_energy_breakdown()      # {'TX': x, 'RX': y, ...}  (strings)
device.energy_model.get_avg_current_ua(sim_t)   # corrente média em µA
```

---

## 5. Debito na Bateria

Quando um `BatteryModel` está associado (`energy_model.set_battery(battery)`), cada chamada a `transition()` que gera energia positiva chama `battery.consume(energy_mj)`:

```python
# energymodel.py — dentro de transition()
if self._battery is not None and energy > 0:
    self._battery.consume(energy)
```

`BatteryModel.consume()` subtrai o valor de `remaining_mj`. Quando `remaining_mj ≤ 0`, seta `depleted = True` e retorna `False`. O debito subsequente é ignorado (bateria já zerada).

Antes de cada transmissão, `network.py` verifica a flag:

```python
# network.py — _on_device_send() e _on_retransmit()
if device.battery is not None and device.battery.depleted:
    return   # device sem energia — ciclo encerrado
```

O device para de transmitir e não agenda próximos eventos. Seu `energy_consumed` fica congelado no valor no momento da depleção.

---

## 6. Energy Harvesting — Crédito Periódico

Se configurado, um `EnergyHarvester` recarrega a bateria a cada 60 s via o evento `ENERGY_HARVEST`:

```python
# network.py — _on_energy_harvest()
for device in self.devices:
    if device.battery is not None and device.battery.harvester is not None:
        device.battery.harvester.harvest_energy(device.battery, time, 60.0)
self.scheduler.schedule(60.0, EventType.ENERGY_HARVEST, self._on_energy_harvest)
```

`harvest_energy()` calcula a potência disponível no horário atual da simulação e chama `battery.harvest(power_mw, duration_s)`, que soma energia a `remaining_mj` (clamped à capacidade máxima) e limpa a flag `depleted` se necessário.

O harvesting **não afeta** `energy_consumed` no `EnergyModel` — ele opera diretamente na bateria, representando energia de entrada, não consumo.

---

## 7. Coleta de Métricas — `analytics.py`

Ao final da simulação, `compute_metrics(network)` agrega os acumuladores de todos os devices:

```python
# Energia total da rede e por device
total_energy_mj  = sum(d.energy_model.energy_consumed for d in network.devices)
energy_per_device = [d.energy_model.energy_consumed for d in network.devices]

# Energia média por SF
energy_per_sf[sf] = mean(d.energy_model.energy_consumed
                         for d in devices if d.sf == sf)

# Breakdown percentual — soma por estado em todos os devices
breakdown_totals = {state: 0.0 for state in RadioState}
for d in network.devices:
    for state, val in d.energy_model.energy_breakdown.items():
        breakdown_totals[state] += val

energy_breakdown_pct[state.name] = breakdown_totals[state] / total_energy_mj * 100
```

**Estimativa de lifetime** — baseada na taxa de consumo observada durante a simulação:

```python
for d in network.devices:
    if d.battery is not None and network.simulation_time > 0:
        consumed_pct     = 100.0 - d.battery.soc_percent()
        rate_pct_per_s   = consumed_pct / network.simulation_time
        lifetime_s       = 100.0 / rate_pct_per_s
        battery_lifetime_days.append(lifetime_s / 86400)
```

A taxa é extrapolada linearmente: se o device consumiu X% da bateria em `simulation_time` segundos, o lifetime é `100% / (X% / simulation_time)`. Não assume comportamento futuro diferente do observado.

**Estrutura retornada em `metrics["energy"]`:**

```python
{
    "total_network_mj":        float,   # soma de todos os devices
    "avg_per_device_mj":       float,
    "min_per_device_mj":       float,
    "max_per_device_mj":       float,
    "avg_per_sf_mj":           {7: float, 8: float, ..., 12: float},
    "breakdown_percent":       {"TX": %, "RX": %, "STANDBY": %, "SLEEP": %},
    "avg_battery_lifetime_days": float | None,  # None se battery_capacity_mah não configurado
}
```

---

## 8. Fluxo Completo por Ciclo de Transmissão

```
_on_device_send(device, t=300s)
  │
  ├── battery.depleted? → True: return (device morto)
  │
  ├── calcula airtime (depende de SF, BW, PL, CR)
  │
  ├── update_energy(device, airtime, sim_time=300s)
  │     │
  │     ├── sleep gap: 300s - 0s = 300s
  │     │     E_sleep = 0.0015 × 3.3 × 300 = 1.485 mJ → energy_consumed, breakdown[SLEEP]
  │     │     battery.consume(1.485)
  │     │
  │     ├── transition(TX,      t,    tx_power=14)  → E[prev=SLEEP]=0 (já contabilizado)
  │     ├── transition(STANDBY, t+at)               → E_TX  = 38×3.3×at   mJ
  │     ├── transition(RX,      t+at+1)             → E_SBY = 1.4×3.3×1   = 4.62 mJ
  │     ├── transition(STANDBY, t+at+2)             → E_RX1 = 11.2×3.3×1  = 36.96 mJ
  │     ├── transition(RX,      t+at+2)             → E_SBY2= 1.4×3.3×0   = 0 mJ
  │     ├── transition(SLEEP,   t+at+4)             → E_RX2 = 11.2×3.3×2  = 73.92 mJ
  │     │
  │     └── _sim_time_last_tx_end = 300 + at + 4
  │
  └── scheduler.schedule(airtime, TX_END, _on_tx_end, device)
      scheduler.schedule(next_delay, DEVICE_SEND, _on_device_send, device)

--- 60s depois ---

_on_energy_harvest()
  └── harvester.harvest_energy(battery, t=360s, duration=60s)
        power = 100 × sin(π × (hour-6)/12)   [mW]
        battery.remaining_mj += power × 60
        (energy_consumed NÃO é alterado)

--- fim da simulação ---

compute_metrics(network)
  └── agrega energy_consumed e energy_breakdown de todos os devices
      estima lifetime via taxa de consumo observada
```

---

## 9. Resumo dos Pontos de Contato

| Onde | O que acontece | Arquivo |
|---|---|---|
| `_on_device_send()` | `update_energy()` — contabiliza ciclo TX completo | `network.py` |
| `_on_retransmit()` | `update_energy()` — mesmo fluxo para retransmissões | `network.py` |
| `EnergyModel.transition()` | Primitiva: fecha estado anterior, abre novo | `energymodel.py` |
| `EnergyModel.update_energy()` | Executa 6 transições do ciclo Class A em batch | `energymodel.py` |
| `BatteryModel.consume()` | Debita `remaining_mj`; seta `depleted` se zero | `battery.py` |
| `_on_energy_harvest()` | Credita energia na bateria a cada 60 s | `network.py` |
| `battery.depleted` check | Impede device esgotado de transmitir | `network.py` |
| `compute_metrics()` | Agrega `energy_consumed` e estima lifetime | `analytics.py` |
