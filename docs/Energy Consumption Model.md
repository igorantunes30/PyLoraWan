# Energy Consumption Model — PyLoRaWAN

## Visão Geral

O consumo de energia de cada End Device é modelado por uma máquina de estados que rastreia o tempo gasto em cada estado operacional e acumula energia de acordo com a corrente característica do hardware SX1272. A cada transição de estado, a energia do período anterior é calculada como `E = I × V × t` e somada ao acumulador do device. Esse modelo permite análise detalhada do consumo por fase do protocolo e estimativa do tempo de vida da bateria.

---

## 1. Estados Operacionais

`EnergyModel` em `energymodel.py` define quatro estados físicos do rádio:

```python
class RadioState(Enum):
    SLEEP   = auto()   # ultra-low power entre ciclos
    STANDBY = auto()   # oscilador ativo, aguardando janela RX
    TX      = auto()   # transmissão ativa
    RX      = auto()   # recepção ativa
```

### 1.1 Correntes por Estado

```python
# energymodel.py — STATE_CURRENT_MA (valores SX1272 compatíveis com ns-3/FLoRa)
STATE_CURRENT_MA = {
    RadioState.SLEEP:   sleep_current_ma,   # 0.0015 mA
    RadioState.STANDBY: 1.4,                # 1.4   mA
    RadioState.RX:      rx_current_ma,      # 11.2  mA
    RadioState.TX:      38.0,               # default; sobrescrito por tx_current_table
}
```

### 1.2 Corrente de Transmissão por Potência

A corrente TX varia com a potência configurada. Valores do datasheet RN2483:

```python
# parametors.py
tx_current_table = {
    14: 38.0,   # dBm → mA
    12: 35.1,
    10: 32.4,
     8: 30.0,
     6: 27.5,
     4: 24.7,
     2: 22.3,
}
```

Para uma potência não listada, `transition()` usa o fallback `tx_current_table.get(14, 38)` = 38 mA.

### 1.3 Tensão de Alimentação

```python
# parametors.py
voltage = 3.3   # V
```

O `EnergyModel` é instanciado com `voltage=3.3` para cada device:

```python
# enddevice.py
self.energy_model = EnergyModel(voltage=3.3)
```

---

## 2. Fórmula de Energia

A energia acumulada em um estado é calculada por:

$$E_i = I_i \cdot V \cdot t_i$$

onde:
- $I_i$ — corrente do estado $i$ (mA), da tabela `STATE_CURRENT_MA` ou `tx_current_table`
- $V$ — tensão de alimentação (V) = 3.3 V
- $t_i$ — duração no estado $i$ (s)
- $E_i$ — energia consumida no estado $i$ (mJ)

A energia total do device é a soma sobre todos os estados e todos os ciclos de transmissão:

$$E_{total} = \sum_{i} I_i \cdot V \cdot t_i$$

Implementação direta em `EnergyModel.transition()`:

```python
energy = current_ma * self.voltage * duration   # mJ
self.energy_consumed                      += energy
self.energy_breakdown[self.current_state] += energy
self.state_durations[self.current_state]  += duration
```

---

## 3. Acumuladores por Device

O `EnergyModel` mantém três acumuladores independentes:

| Acumulador | Tipo | Conteúdo |
|---|---|---|
| `energy_consumed` | `float` (mJ) | Total acumulado desde o início da simulação |
| `energy_breakdown` | `dict {RadioState → float}` | mJ por estado (SLEEP, STANDBY, TX, RX) |
| `state_durations` | `dict {RadioState → float}` | Segundos acumulados por estado |

---

## 4. Ciclo de Energia Class A

`update_energy()` executa as seis transições do ciclo LoRaWAN Class A em sequência, usando as durações de `parametors.py`:

```
t₀              t₀+airtime    t₀+at+rd1   t₀+at+rd1+trx1   t₀+at+rd1+trx1+rd2_eff   t₀+...+trx2
│◄──── TX ──────►│◄─ STANDBY ─►│◄── RX1 ──►│◄─── STANDBY ───►│◄────── RX2 ────────────►│◄─ SLEEP
│  I_TX(tx_pwr) │   1.4 mA    │  11.2 mA  │     1.4 mA (0s) │        11.2 mA          │ 0.0015 mA
```

Com os defaults EU868 (`receive_delay1=1s`, `trx1=1s`, `receive_delay2=2s`, `trx2=2s`):

```python
rx_delay2_duration = max(0, receive_delay2 - receive_delay1 - trx1) = 0 s
```

As janelas RX1 e RX2 ocorrem consecutivamente sem pausa entre elas.

**Energia por fase** (exemplo SF9, tx_power=14 dBm, airtime ≈ 0.185 s):

| Fase | Duração | Corrente | Energia |
|---|---|---|---|
| TX | 0.185 s | 38.0 mA | 22.6 mJ |
| STANDBY (WAIT_RX1) | 1.0 s | 1.4 mA | 4.6 mJ |
| RX1 | 1.0 s | 11.2 mA | 37.0 mJ |
| STANDBY (WAIT_RX2) | 0.0 s | 1.4 mA | 0.0 mJ |
| RX2 | 2.0 s | 11.2 mA | 73.9 mJ |
| SLEEP | até próx. TX | 0.0015 mA | ≈ 0.0 mJ/s |

O SLEEP entre ciclos é contabilizado retroativamente no início de cada novo TX pelo gap `sim_time - _sim_time_last_tx_end`.

---

## 5. Estado Sleep — Contabilização Retroativa

O sleep entre ciclos consecutivos é calculado no início de cada `update_energy()`:

```python
if sim_time is not None and sim_time > self._sim_time_last_tx_end:
    sleep_duration = sim_time - self._sim_time_last_tx_end
    sleep_energy   = STATE_CURRENT_MA[RadioState.SLEEP] * self.voltage * sleep_duration
    # 0.0015 mA × 3.3 V × duration_s = mJ
    self.energy_consumed                    += sleep_energy
    self.energy_breakdown[RadioState.SLEEP] += sleep_energy
    self.state_durations[RadioState.SLEEP]  += sleep_duration
```

Após cada ciclo, o marcador é atualizado:

```python
self._sim_time_last_tx_end = sim_time + airtime + receive_delay1 + trx1 + rx_delay2_duration + trx2
```

Desde o instante zero até o primeiro TX, `_sim_time_last_tx_end = 0.0`, portanto o sleep inicial também é contabilizado.

---

## 6. Corrente Média e Estimativa de Lifetime

### 6.1 Corrente Média

`get_avg_current_ua(total_time_s)` deriva a corrente média da energia total acumulada:

```python
def get_avg_current_ua(self, total_time_s):
    # E = I × V × t  →  I = E / (V × t)
    # mJ / (V × s) = mA  →  × 1000 = µA
    return (self.energy_consumed / (self.voltage * total_time_s)) * 1000
```

### 6.2 Modelo de Bateria — `BatteryModel`

Quando uma bateria está associada ao device, cada chamada a `transition()` que gera energia positiva debita diretamente:

```python
# battery.py
class BatteryModel:
    def __init__(self, capacity_mah=2400, voltage=3.3):
        self.capacity_mj  = capacity_mah * voltage * 3.6   # mAh → mJ
        self.remaining_mj = self.capacity_mj
        self.depleted     = False
```

Conversão de capacidade:
$$C_{mJ} = C_{mAh} \times V \times 3600 \text{ s/h} \times 10^{-3} \text{ (mJ/mWs)}$$

Para a bateria padrão de 2400 mAh a 3.3 V: $C_{mJ} = 2400 \times 3.3 \times 3.6 = 28512 \text{ mJ}$.

`consume(energy_mj)` subtrai de `remaining_mj`. Quando `remaining_mj ≤ 0`, seta `depleted = True`. Devices com `battery.depleted = True` param de transmitir:

```python
# network.py — _on_device_send()
if device.battery is not None and device.battery.depleted:
    return   # device sem energia — ciclo encerrado
```

### 6.3 State of Charge

```python
def soc_percent(self):
    return (self.remaining_mj / self.capacity_mj) * 100
```

### 6.4 Estimativa de Lifetime

`analytics.py` extrapola linearmente a taxa de consumo observada durante a simulação:

```python
consumed_pct   = 100.0 - device.battery.soc_percent()
rate_pct_per_s = consumed_pct / network.simulation_time
lifetime_s     = 100.0 / rate_pct_per_s
battery_lifetime_days = lifetime_s / 86400
```

Isso equivale a: se o device consumiu $X\%$ da bateria em $T$ segundos de simulação, o lifetime estimado é $T / X\%$.

---

## 7. Energy Harvesting

`EnergyHarvester` recarrega a bateria periodicamente sem afetar `energy_consumed` no `EnergyModel` — representa energia de entrada, não consumo:

```python
class EnergyHarvester:
    def get_power(self, time_of_day_hours):
        if self.model == "solar":
            if 6 <= hour <= 18:
                return peak_power_mw × sin(π × (hour − 6) / 12)
            return 0
        elif self.model == "constant":
            return peak_power_mw
```

O evento `ENERGY_HARVEST` em `network.py` chama `harvest_energy(battery, sim_time, duration_s=60)` a cada 60 s:

```python
battery.remaining_mj = min(remaining_mj + power × 60, capacity_mj)
if remaining_mj > 0:
    battery.depleted = False   # reativa device se estava esgotado
```

---

## 8. Acesso às Métricas de Energia

| Método / Atributo | Tipo | Conteúdo |
|---|---|---|
| `device.energy_model.energy_consumed` | `float` (mJ) | Total acumulado |
| `device.energy_model.get_energy_breakdown()` | `dict {str → float}` | mJ por estado (strings) |
| `device.energy_model.get_avg_current_ua(t)` | `float` (µA) | Corrente média no período t |
| `device.energy_model.stats()` | `dict` | Total + estado atual + durations + breakdown |
| `device.battery.soc_percent()` | `float` (%) | State of Charge |
| `device.battery.estimate_lifetime_days(mj_per_h)` | `float` | Lifetime restante em dias |

`compute_metrics()` em `analytics.py` agrega os acumuladores de todos os devices ao final da simulação:

```python
total_energy_mj   = sum(d.energy_model.energy_consumed for d in network.devices)
energy_per_device = [d.energy_model.energy_consumed for d in network.devices]
energy_per_sf     = {sf: mean(...) for sf in range(7, 13)}
breakdown_percent = {state: total_state_mj / total_mj * 100 for state in RadioState}
```

---

## 9. Parâmetros Configuráveis

| Parâmetro | Arquivo | Valor | Descrição |
|---|---|---|---|
| `voltage` | `parametors.py` | 3.3 V | Tensão de alimentação do device |
| `sleep_current_ma` | `parametors.py` | 0.0015 mA | Corrente sleep (SX1272) |
| `rx_current_ma` | `parametors.py` | 11.2 mA | Corrente recepção (SX1272) |
| `rx_delay_current_ma` | `parametors.py` | 1.4 mA | Corrente standby |
| `tx_current_table` | `parametors.py` | 22.3–38.0 mA | Corrente TX por potência (RN2483) |
| `capacity_mah` | `BatteryModel.__init__` | 2400 mAh | Capacidade da bateria |
| `peak_power_mw` | `EnergyHarvester.__init__` | 100 mW | Potência de pico do harvester solar |
