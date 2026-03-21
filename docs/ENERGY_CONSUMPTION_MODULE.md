# PyLoRaWAN — Energy Consumption Module

> Documentação técnica completa do modelo de consumo de energia.
> Cobre EnergyModel, BatteryModel, EnergyHarvester, integração com a FSM do EndDevice,
> cálculo de ciclo TX e estimativa de tempo de vida.

---

## Sumário

1. [Visão Geral da Arquitetura de Energia](#1-visão-geral-da-arquitetura-de-energia)
2. [Parâmetros de Energia (parametors.py)](#2-parâmetros-de-energia-parametorspy)
3. [EnergyModel — Máquina de Estados](#3-energymodel--máquina-de-estados)
   - 3.1 [RadioState (4 estados)](#31-radiostate-4-estados)
   - 3.2 [STATE_CURRENT_MA](#32-state_current_ma)
   - 3.3 [Atributos e Estruturas](#33-atributos-e-estruturas)
   - 3.4 [transition() — Núcleo do Modelo](#34-transition--núcleo-do-modelo)
   - 3.5 [update_energy() — Ciclo TX Completo](#35-update_energy--ciclo-tx-completo)
   - 3.6 [Métodos Auxiliares](#36-métodos-auxiliares)
4. [Integração com a FSM do EndDevice](#4-integração-com-a-fsm-do-enddevice)
   - 4.1 [Mapeamento RadioState MAC → EnergyState](#41-mapeamento-radiostate-mac--energystate)
   - 4.2 [transition_state() no EndDevice](#42-transition_state-no-enddevice)
   - 4.3 [update_energy() vs transition_state()](#43-update_energy-vs-transition_state)
5. [BatteryModel](#5-batterymodel)
   - 5.1 [Atributos e Inicialização](#51-atributos-e-inicialização)
   - 5.2 [consume() e Depleção](#52-consume-e-depleção)
   - 5.3 [harvest() e Recarga](#53-harvest-e-recarga)
   - 5.4 [State of Charge e Lifetime](#54-state-of-charge-e-lifetime)
6. [EnergyHarvester](#6-energyharvester)
   - 6.1 [Modelo Solar](#61-modelo-solar)
   - 6.2 [Modelo Constante](#62-modelo-constante)
   - 6.3 [harvest_energy() — Integração na Simulação](#63-harvest_energy--integração-na-simulação)
7. [Cálculo de Energia por Ciclo TX](#7-cálculo-de-energia-por-ciclo-tx)
   - 7.1 [Sequência de Estados](#71-sequência-de-estados)
   - 7.2 [Fórmulas e Tabelas](#72-fórmulas-e-tabelas)
   - 7.3 [Dominância do RX Idle](#73-dominância-do-rx-idle)
8. [Energia de Sleep entre Transmissões (Sprint 5)](#8-energia-de-sleep-entre-transmissões-sprint-5)
9. [Breakdown de Energia por Estado](#9-breakdown-de-energia-por-estado)
10. [Estimativa de Tempo de Vida da Bateria](#10-estimativa-de-tempo-de-vida-da-bateria)
11. [Energy Harvesting na Simulação](#11-energy-harvesting-na-simulação)
12. [Métricas de Energia em analytics.py](#12-métricas-de-energia-em-analyticspy)
13. [Análise Quantitativa por SF](#13-análise-quantitativa-por-sf)
14. [Comparação com ns-3 e FLoRa](#14-comparação-com-ns-3-e-flora)
15. [Exemplos Numéricos Completos](#15-exemplos-numéricos-completos)

---

## 1. Visão Geral da Arquitetura de Energia

```
parametors.py
  tx_current_table, rx_current_ma, sleep_current_ma
         │
         ▼
┌─────────────────────────────────────────┐
│            EnergyModel                  │
│  (energymodel.py)                       │
│                                         │
│  RadioState: SLEEP│STANDBY│RX│TX        │
│  transition(new_state, time, tx_power)  │
│  update_energy(device, airtime, sim_t)  │
│                                         │
│  energy_consumed   [mJ total]           │
│  energy_breakdown  {state → mJ}         │
│  state_durations   {state → s}          │
└────────────┬────────────────────────────┘
             │ consume(energy_mj)
             ▼
┌─────────────────────────────────────────┐
│            BatteryModel                 │
│  (battery.py)                           │
│                                         │
│  remaining_mj, capacity_mj             │
│  depleted flag                          │
│  soc_percent()                          │
│  estimate_lifetime_days()               │
└────────────┬────────────────────────────┘
             │ harvest(power_mw, duration_s)
             ▲
┌─────────────────────────────────────────┐
│          EnergyHarvester                │
│  (battery.py)                           │
│                                         │
│  modelo solar: perfil senoidal 6h–18h  │
│  modelo constant: potência fixa         │
│  harvest_energy(battery, sim_t, dur)    │
└─────────────────────────────────────────┘

Integração no EndDevice:
  RadioState (7 estados MAC) → _ENERGY_STATE_MAP → EnergyState (4 estados)
  enddevice.transition_state(new_state, time) chama energy_model.transition()
```

### Responsabilidade por Arquivo

| Arquivo         | Responsabilidade                                              |
|-----------------|---------------------------------------------------------------|
| `energymodel.py`| Máquina de estados, cálculo de energia, tracking por estado  |
| `battery.py`    | BatteryModel (capacidade, depleção) + EnergyHarvester        |
| `enddevice.py`  | FSM MAC, mapeamento para EnergyModel, `transition_state()`   |
| `parametors.py` | Correntes datasheet (SX1272, RN2483), tensão, noise_figure   |
| `network.py`    | Orquestra `update_energy()`, `energy_log`, harvesting events |

---

## 2. Parâmetros de Energia (parametors.py)

### Correntes por Potência TX — RN2483 Datasheet

```python
tx_current_table = {
    14: 38.0,   # mA @ 14 dBm
    12: 35.1,   # mA @ 12 dBm
    10: 32.4,   # mA @ 10 dBm
     8: 30.0,   # mA @ 8 dBm
     6: 27.5,   # mA @ 6 dBm
     4: 24.7,   # mA @ 4 dBm
     2: 22.3,   # mA @ 2 dBm
}
```

### Correntes de Recepção e Sleep — SX1272 Datasheet

```python
rx_current_ma      = 11.2     # mA — modo RX ativo
rx_delay_current_ma = 1.4     # mA — standby (entre TX e RX)
sleep_current_ma   = 0.0015   # mA — deep sleep
voltage            = 3.3      # V
```

### Janelas de Recepção Class A

```python
receive_delay1 = 1    # s — delay antes de RX1 (standby)
receive_delay2 = 2    # s — delay antes de RX2 (standby total)
trx1           = 1    # s — duração da janela RX1
trx2           = 2    # s — duração da janela RX2
```

---

## 3. EnergyModel — Máquina de Estados

**Arquivo:** `energymodel.py`

Baseado no `ns-3 LoraRadioEnergyModel`.

### 3.1 RadioState (4 estados)

```python
class RadioState(Enum):
    SLEEP   = auto()   # deep sleep — corrente mínima
    STANDBY = auto()   # idle/RX delay — aguardando janela
    TX      = auto()   # transmissão ativa
    RX      = auto()   # recepção ativa (janela RX1 ou RX2)
```

### 3.2 STATE_CURRENT_MA

```python
STATE_CURRENT_MA = {
    RadioState.SLEEP:   sleep_current_ma,   # 0.0015 mA
    RadioState.STANDBY: 1.4,                # mA
    RadioState.RX:      rx_current_ma,      # 11.2 mA
    RadioState.TX:      38.0,               # mA (default, sobrescrito por tx_power)
}
```

A corrente TX real é buscada na `tx_current_table[tx_power]`, não no `STATE_CURRENT_MA`.

### 3.3 Atributos e Estruturas

| Atributo                | Tipo   | Inicialização | Descrição                                    |
|-------------------------|--------|---------------|----------------------------------------------|
| `voltage`               | float  | 3.3 V         | Tensão de alimentação                        |
| `energy_consumed`       | float  | 0.0 mJ        | Energia total acumulada                      |
| `current_state`         | RadioState | SLEEP    | Estado atual do rádio                        |
| `state_entry_time`      | float  | 0.0 s         | Timestamp de entrada no estado atual         |
| `_current_tx_power`     | int    | 14 dBm        | Potência TX ativa                            |
| `state_durations`       | dict   | `{state: 0.0}`| Segundos acumulados por estado               |
| `energy_breakdown`      | dict   | `{state: 0.0}`| mJ acumulados por estado                     |
| `_battery`              | BatteryModel | None  | Referência opcional para depleção           |
| `_sim_time_last_tx_end` | float  | 0.0 s         | Fim do último ciclo TX (Sprint 5 — sleep tracking) |

### 3.4 transition() — Núcleo do Modelo

```python
def transition(self, new_state, time, tx_power=None):
    """Registra energia do estado anterior e transiciona para novo estado."""

    duration = time - self.state_entry_time
    if duration < 0:
        duration = 0   # proteção contra eventos fora de ordem

    # Corrente do estado que está SAINDO
    if self.current_state == RadioState.TX:
        current_ma = tx_current_table.get(
            self._current_tx_power,
            tx_current_table.get(14, 38)   # fallback para 14 dBm
        )
    else:
        current_ma = STATE_CURRENT_MA[self.current_state]

    # E = I × V × t   (mA × V × s = mJ)
    energy = current_ma * self.voltage * duration

    # Acumula no total e no breakdown
    self.energy_consumed             += energy
    self.energy_breakdown[self.current_state] += energy
    self.state_durations[self.current_state]  += duration

    # Debita da bateria (se disponível)
    if self._battery is not None and energy > 0:
        self._battery.consume(energy)

    # Transiciona
    self.current_state  = new_state
    self.state_entry_time = time
    if tx_power is not None:
        self._current_tx_power = tx_power

    return energy   # energia do estado anterior (mJ)
```

#### Fórmula de Energia

```
E [mJ] = I [mA] × V [V] × t [s]

Unidades: mA × V × s = mW × s = mJ ✓
```

#### Exemplo de Transição

```
Estado anterior: STANDBY (1.4 mA) por 1.0 s
E = 1.4 × 3.3 × 1.0 = 4.62 mJ

Estado anterior: TX (14 dBm → 38 mA) por 0.144 s
E = 38 × 3.3 × 0.144 = 18.09 mJ

Estado anterior: RX (11.2 mA) por 1.0 s
E = 11.2 × 3.3 × 1.0 = 36.96 mJ

Estado anterior: SLEEP (0.0015 mA) por 290 s
E = 0.0015 × 3.3 × 290 = 1.44 mJ
```

### 3.5 update_energy() — Ciclo TX Completo

```python
def update_energy(self, device, airtime, sim_time=None):
    """Calcula energia de um ciclo TX completo (Class A).

    Ciclo: SLEEP(gap) → TX → STANDBY(rd1) → RX1 → STANDBY(rd2) → RX2 → SLEEP
    """
    if airtime <= 0:
        return

    # Sprint 5: contabiliza sleep desde o último ciclo TX
    if sim_time is not None and sim_time > self._sim_time_last_tx_end:
        sleep_duration = sim_time - self._sim_time_last_tx_end
        sleep_energy   = STATE_CURRENT_MA[RadioState.SLEEP] * self.voltage * sleep_duration
        self.energy_consumed                    += sleep_energy
        self.energy_breakdown[RadioState.SLEEP] += sleep_energy
        self.state_durations[RadioState.SLEEP]  += sleep_duration
        if self._battery is not None:
            self._battery.consume(sleep_energy)

    t = self.state_entry_time

    # 1. TX (airtime)
    self.transition(RadioState.TX, t, tx_power=device.tx_power)
    t += airtime

    # 2. STANDBY — RX delay 1 (receive_delay1 = 1s)
    self.transition(RadioState.STANDBY, t)
    t += receive_delay1

    # 3. RX1 (trx1 = 1s)
    self.transition(RadioState.RX, t)
    t += trx1

    # 4. STANDBY — RX delay 2 (max(0, receive_delay2 - receive_delay1 - trx1))
    self.transition(RadioState.STANDBY, t)
    rx_delay2_duration = max(0, receive_delay2 - receive_delay1 - trx1)
    t += rx_delay2_duration

    # 5. RX2 (trx2 = 2s)
    self.transition(RadioState.RX, t)
    t += trx2

    # 6. SLEEP
    self.transition(RadioState.SLEEP, t)

    # Registra fim do ciclo para próximo cálculo de sleep
    if sim_time is not None:
        self._sim_time_last_tx_end = (sim_time + airtime
                                      + receive_delay1 + trx1
                                      + rx_delay2_duration + trx2)
```

#### Timeline do Ciclo Class A

```
t=0                         t=airtime
│◄────────── TX ─────────►│◄─RD1─►│◄── RX1 ──►│◄─RD2─►│◄──── RX2 ────►│◄── SLEEP ──►
│   38 mA (14dBm)         │ 1.4mA │  11.2 mA  │ 1.4mA │   11.2 mA     │  0.0015 mA
│                         │  1s   │    1s     │  0s   │      2s       │   até próx TX
```

**Parâmetros EU868 Class A (padrão):**

| Fase          | Duração           | Corrente  | Código              |
|---------------|-------------------|-----------|---------------------|
| TX            | airtime (SF-dep.) | tx_table  | `receive_delay1`    |
| RX delay 1    | 1 s               | 1.4 mA    | `rd1`               |
| RX1           | 1 s               | 11.2 mA   | `trx1`              |
| RX delay 2    | 0 s (= rd2-rd1-trx1=0) | 1.4 mA | `rx_delay2_duration`|
| RX2           | 2 s               | 11.2 mA   | `trx2`              |
| SLEEP         | até próx TX       | 0.0015 mA | variável            |

> `rx_delay2_duration = max(0, 2 - 1 - 1) = 0` com os defaults EU868.
> As janelas RX1 e RX2 ocorrem consecutivamente (sem standby extra entre elas).

### 3.6 Métodos Auxiliares

| Método                              | Retorno  | Descrição                                   |
|-------------------------------------|----------|---------------------------------------------|
| `get_total_consumption_mj()`        | float    | `energy_consumed` em mJ                    |
| `get_total_consumption_j()`         | float    | `energy_consumed / 1000`                   |
| `get_energy_breakdown()`            | dict     | `{state_name: round(mJ, 3)}`              |
| `get_avg_current_ua(total_time_s)`  | float    | `(energy_mJ / (V × t)) × 1000` em µA      |
| `log_energy()`                      | None     | Print formatado do total                   |
| `reset_energy()`                    | None     | Zera todos os acumuladores                 |
| `set_battery(battery)`              | None     | Associa BatteryModel                       |
| `stats()`                           | dict     | Estado completo para debug                 |

#### `stats()` — Saída Completa

```python
{
    "total_energy_mj": 1234.567,
    "current_state": "SLEEP",
    "state_durations_s": {
        "SLEEP":   3200.0,
        "STANDBY":  300.0,
        "TX":        36.0,
        "RX":        64.0,
    },
    "energy_breakdown_mj": {
        "SLEEP":   15.84,
        "STANDBY": 1386.0,
        "TX":      507.6,
        "RX":      2365.4,
    }
}
```

---

## 4. Integração com a FSM do EndDevice

**Arquivo:** `enddevice.py`

### 4.1 Mapeamento RadioState MAC → EnergyState

O `EndDevice` tem 7 estados MAC (`RadioState`). O `EnergyModel` tem 4 estados físicos. O mapeamento é feito via `_ENERGY_STATE_MAP`:

```python
_ENERGY_STATE_MAP = {
    "IDLE":     "STANDBY",   # aguardando próximo ciclo → 1.4 mA
    "TX":       "TX",        # transmitindo → 22–38 mA (por potência)
    "WAIT_RX1": "STANDBY",   # aguardando janela RX1 → 1.4 mA
    "RX1":      "RX",        # recebendo RX1 → 11.2 mA
    "WAIT_RX2": "STANDBY",   # aguardando janela RX2 → 1.4 mA
    "RX2":      "RX",        # recebendo RX2 → 11.2 mA
    "SLEEP":    "SLEEP",     # deep sleep → 0.0015 mA
}
```

**Decisão de Design:** `IDLE`, `WAIT_RX1` e `WAIT_RX2` mapeiam para `STANDBY` (1.4 mA), não para `SLEEP`. Isso modela o rádio em modo de escuta leve — aguardando a janela de recepção sem desligar completamente o oscilador.

### 4.2 transition_state() no EndDevice

```python
def transition_state(self, new_state: RadioState, time: float) -> RadioState:
    """Transiciona FSM MAC e registra energia no EnergyModel."""
    from energymodel import RadioState as EnergyState

    old_state = self.radio_state
    energy_state_name = self._ENERGY_STATE_MAP.get(new_state.name, "SLEEP")

    # Delega o cálculo de energia para o EnergyModel
    self.energy_model.transition(EnergyState[energy_state_name], time)

    self.radio_state = new_state
    return old_state
```

#### Chamadas a transition_state() durante o Ciclo

```
_on_device_send():    transition_state(TX, t)
_on_tx_end():         transition_state(WAIT_RX1, t)
_on_rx1_open():       transition_state(RX1, t)
_on_rx1_close():      transition_state(WAIT_RX2, t)   # ou IDLE se DL recebido
_on_rx2_open():       transition_state(RX2, t)
_on_rx2_close():      transition_state(IDLE, t)        # ou SLEEP
```

### 4.3 update_energy() vs transition_state()

Existem **dois caminhos** para registrar energia no simulador:

| Caminho                  | Quando usar                           | Precisão          |
|--------------------------|---------------------------------------|-------------------|
| `update_energy()`        | Ciclo TX completo em batch (simples)  | Aproximada — usa durations fixas |
| `transition_state()`     | Eventos individuais da FSM             | Exata — baseada em timestamps reais |

Na prática, `_on_device_send()` chama `update_energy()` **uma vez por transmissão** antes de registrar no scheduler. As transições `transition_state()` são chamadas separadamente pelos handlers de evento.

> **Atenção:** Não combinar `update_energy()` e `transition_state()` no mesmo ciclo para o mesmo dispositivo — isso duplicaria a contabilização.

---

## 5. BatteryModel

**Arquivo:** `battery.py`

Compatível com `ns-3 BasicEnergySource`.

### 5.1 Atributos e Inicialização

```python
class BatteryModel:
    def __init__(self, capacity_mah=2400, voltage=3.3):
        self.capacity_mah  = capacity_mah          # mAh
        self.voltage       = voltage               # V
        self.capacity_mj   = capacity_mah * voltage * 3.6   # mJ total
        self.remaining_mj  = self.capacity_mj      # começa cheio
        self.depleted      = False                 # flag de esgotamento
        self.harvester     = None                  # EnergyHarvester opcional
        self._consumption_log = []                 # histórico de consumos
```

#### Conversão mAh → mJ

```
capacity_mj = capacity_mah × V × 3.6
            = 2400 × 3.3 × 3.6
            = 28,512 mJ  (≈ 28.5 J)

Intuição: bateria AA alcalina ≈ 2400–3000 mAh ≈ 28–35 kJ
```

### 5.2 consume() e Depleção

```python
def consume(self, energy_mj):
    """Consome energia da bateria. Retorna True se OK, False se esgotada."""
    if self.depleted:
        return False

    self.remaining_mj -= energy_mj
    self._consumption_log.append(energy_mj)

    if self.remaining_mj <= 0:
        self.remaining_mj = 0
        self.depleted = True
        return False

    return True
```

#### Comportamento na Depleção

Quando `depleted = True`:
- `consume()` retorna False imediatamente (sem subtrair)
- `network.py` verifica `device.battery.depleted` antes de cada transmissão
- Dispositivo para de transmitir — removido da simulação efetivamente

```python
# network.py — _on_device_send()
if device.battery is not None and device.battery.depleted:
    return   # device sem energia — não transmite
```

### 5.3 harvest() e Recarga

```python
def harvest(self, power_mw, duration_s):
    """Adiciona energia colhida (clamped à capacidade máxima)."""
    energy_mj = power_mw * duration_s
    self.remaining_mj = min(self.remaining_mj + energy_mj, self.capacity_mj)
    if self.remaining_mj > 0:
        self.depleted = False   # "ressuscita" se havia esgotado
```

### 5.4 State of Charge e Lifetime

```python
def soc_percent(self):
    """State of Charge em percentual (0–100%)."""
    return (self.remaining_mj / self.capacity_mj) * 100

def remaining_mah(self):
    """Capacidade restante em mAh."""
    return self.remaining_mj / (self.voltage * 3.6)

def estimate_lifetime_days(self, avg_consumption_mj_per_hour):
    """Estima dias restantes de operação."""
    if avg_consumption_mj_per_hour <= 0:
        return float('inf')
    hours = self.remaining_mj / avg_consumption_mj_per_hour
    return hours / 24

def stats(self):
    total_consumed = self.capacity_mj - self.remaining_mj
    return {
        "capacity_mah":     self.capacity_mah,
        "remaining_mah":    self.remaining_mah(),
        "soc_percent":      self.soc_percent(),
        "total_consumed_mj": total_consumed,
        "depleted":         self.depleted,
    }
```

---

## 6. EnergyHarvester

**Arquivo:** `battery.py`

Diferencial único: nenhum dos 4 simuladores comparados (FLoRa, ns-3, LoRaWANSim, LR-FHSS-sim) implementa energy harvesting.

### 6.1 Modelo Solar

```python
def get_power(self, time_of_day_hours):
    if self.model == "solar":
        hour = time_of_day_hours % 24
        if 6 <= hour <= 18:
            return self.peak_power_mw * np.sin(np.pi * (hour - 6) / 12)
        return 0   # sem coleta à noite
```

#### Perfil de Potência Solar

```
Hora:  0h   6h   9h   12h   15h   18h   24h
       │    │    │     │     │     │     │
Power: 0    0   71mW  100mW  71mW   0    0
       (com peak_power_mw = 100 mW)

Fórmula: P(h) = P_peak × sin(π × (h − 6) / 12)  para 6 ≤ h ≤ 18
```

A integral da potência solar ao longo de 1 dia:
```
E_day = ∫(6→18) P_peak × sin(π(h−6)/12) dh
       = P_peak × 12/π  [× 3600 s/h × 1 mW·s = 1 mJ]
       = 100 × 12/π × 3600
       ≈ 1,375,099 mJ/dia  (= 1375 J/dia para painel de 100mW pico)
```

### 6.2 Modelo Constante

```python
elif self.model == "constant":
    return self.peak_power_mw   # potência fixa, independente do horário
```

Modela harvesting por vibração, diferença de temperatura (TEG), ou fonte de RF constante.

### 6.3 harvest_energy() — Integração na Simulação

```python
def harvest_energy(self, battery, sim_time_s, duration_s, day_length_s=86400):
    """Colhe energia no intervalo [sim_time_s, sim_time_s + duration_s]."""
    time_of_day = (sim_time_s % day_length_s) / 3600.0   # converte para horas
    power       = self.get_power(time_of_day)
    energy      = power * duration_s                       # mW × s = mJ
    self.total_harvested_mj += energy
    battery.harvest(power, duration_s)
    return energy
```

**Chamado a cada 60s** pelo evento `ENERGY_HARVEST` no scheduler:

```python
# network.py — _on_energy_harvest()
for device in devices:
    if device.battery and device.battery.harvester:
        device.battery.harvester.harvest_energy(
            device.battery,
            sim_time_s = current_time,
            duration_s = 60.0
        )
scheduler.schedule(60, ENERGY_HARVEST, _on_energy_harvest)   # repete a cada 60s
```

---

## 7. Cálculo de Energia por Ciclo TX

### 7.1 Sequência de Estados

```
                    TX          STANDBY  RX1  STANDBY   RX2        SLEEP
                ┌─────────┐     ┌──┐   ┌──┐   ┌──┐   ┌────┐   ┌─────────┐
Corrente (mA):  │  38.0   │     │1.4│   │11.2│ │1.4│  │11.2│   │ 0.0015 │
                └─────────┘     └──┘   └──┘   └──┘   └────┘   └─────────┘
Duração (s):     airtime         1s     1s     0s      2s      gap_to_next
```

### 7.2 Fórmulas e Tabelas

#### Energia por Fase (TX power = 14 dBm, V = 3.3V)

```
E_TX      = 38.0  × 3.3 × airtime      = 125.4 mJ/s × airtime
E_STANDBY = 1.4   × 3.3 × 1.0          = 4.62  mJ  (RX delay 1)
E_RX1     = 11.2  × 3.3 × 1.0          = 36.96 mJ  (janela RX1)
E_STANDBY2= 1.4   × 3.3 × 0.0          = 0.00  mJ  (delay2 = 0 com defaults)
E_RX2     = 11.2  × 3.3 × 2.0          = 73.92 mJ  (janela RX2)
E_WINDOWS = E_STANDBY + E_RX1 + E_STANDBY2 + E_RX2 = 115.5 mJ (fixo por ciclo)
```

#### Energia de TX por SF (14 dBm, BW=125kHz, PL=20B)

| SF  | Airtime (ms) | E_TX (mJ) | E_Windows (mJ) | E_Total_ciclo (mJ) |
|-----|-------------|-----------|---------------|---------------------|
| SF7 | 36.1        | 4.53      | 115.5         | **120.0**           |
| SF8 | 72.2        | 9.05      | 115.5         | **124.6**           |
| SF9 | 144.4       | 18.10     | 115.5         | **133.6**           |
| SF10| 288.8       | 36.20     | 115.5         | **151.7**           |
| SF11| 577.5       | 72.40     | 115.5         | **187.9**           |
| SF12| 1154.9      | 144.82    | 115.5         | **260.3**           |

#### Energia de TX por TX Power (SF9, airtime=144ms)

| TX Power | Corrente | E_TX (mJ) | E_Total_ciclo (mJ) |
|----------|----------|-----------|---------------------|
| 14 dBm   | 38.0 mA  | 18.10     | **133.6**           |
| 12 dBm   | 35.1 mA  | 16.72     | **132.2**           |
| 10 dBm   | 32.4 mA  | 15.43     | **130.9**           |
| 8 dBm    | 30.0 mA  | 14.29     | **129.8**           |
| 6 dBm    | 27.5 mA  | 13.10     | **128.6**           |
| 2 dBm    | 22.3 mA  | 10.62     | **126.1**           |

### 7.3 Dominância do RX Idle

A tabela acima revela um fato crítico: **para SFs baixos, a energia das janelas RX domina sobre a energia de TX**.

```
SF7:  E_TX = 4.53 mJ   E_Windows = 115.5 mJ  → RX representa 96% do ciclo!
SF9:  E_TX = 18.1 mJ   E_Windows = 115.5 mJ  → RX representa 86%
SF12: E_TX = 144.8 mJ  E_Windows = 115.5 mJ  → RX representa 44%
```

**Implicação:** Reduzir SF (via ADR) **aumenta** a eficiência energética relativa, pois E_TX cai muito mais rápido que E_Windows:

```
SF7 vs SF12: E_TX cai de 144.8 → 4.5 mJ (fator 32×)
             E_Total cai de 260.3 → 120.0 mJ (fator 2.2×)
→ Ganho real de energia por ciclo: 2.2×, não 32×
```

A janela RX2 (2s × 11.2mA = 73.9 mJ) é o maior consumidor individual em todos os SFs.

---

## 8. Energia de Sleep entre Transmissões (Sprint 5)

O Sprint 5 corrigiu uma falha: a energia consumida durante o sleep entre transmissões não estava sendo contabilizada.

### Mecanismo

```python
# energymodel.py — update_energy()

if sim_time is not None and sim_time > self._sim_time_last_tx_end:
    sleep_duration = sim_time - self._sim_time_last_tx_end
    sleep_energy   = 0.0015 × 3.3 × sleep_duration   # mJ
    self.energy_consumed += sleep_energy
    ...

# Registra fim do ciclo atual
self._sim_time_last_tx_end = sim_time + airtime + receive_delay1 + trx1 + rx_delay2_duration + trx2
```

### Importância do Sleep Tracking

Com λ=1/300s (uma transmissão a cada 300s):
```
Duração do ciclo ativo = airtime + 1s + 1s + 0s + 2s ≈ 4s (para SF9)
Duração do sleep = 300 - 4 = 296s por ciclo

E_sleep por ciclo = 0.0015 × 3.3 × 296 = 1.47 mJ
E_ciclo ativo     = 133.6 mJ (TX + janelas)

→ Sleep representa apenas 1.47 / (133.6 + 1.47) ≈ 1.1% do consumo total

Mas ao longo de 1 hora (12 ciclos):
  E_sleep_total = 1.47 × 12 = 17.6 mJ
  E_total       = (133.6 + 1.47) × 12 = 1621 mJ
  → Sleep ainda é <1.1% — porém fisicamente correto
```

O sleep é mais significativo em cenários com λ muito baixo (ex: λ=1/3600s → 1 TX/hora):
```
E_sleep = 0.0015 × 3.3 × 3596 = 17.8 mJ
E_ciclo = 133.6 mJ
→ Sleep representa 11.7% — não negligível
```

---

## 9. Breakdown de Energia por Estado

`energy_breakdown` é um dict `{RadioState → mJ}` acumulado ao longo de toda a simulação.

### Distribuição Típica (SF9, λ=1/300s, 1 hora, 12 ciclos)

```
E_TX      = 18.1  × 12 =  217.2 mJ  (16%)
E_STANDBY = 4.62  × 12 =   55.4 mJ  ( 4%)
E_RX      = (37.0 + 73.9) × 12 = 1306.8 mJ  (79%)  ← domina
E_SLEEP   = 1.47  × 12 =   17.6 mJ  ( 1%)
─────────────────────────────────────────
Total:                   = 1597.0 mJ
```

### Distribuição Típica (SF7, λ=1/300s, 1 hora, 12 ciclos)

```
E_TX      =  4.5  × 12 =   54.4 mJ  ( 4%)
E_STANDBY =  4.62 × 12 =   55.4 mJ  ( 4%)
E_RX      = 110.9 × 12 = 1330.7 mJ  (92%)  ← domina ainda mais
E_SLEEP   =  1.47 × 12 =   17.6 mJ  ( 1%)
─────────────────────────────────────────
Total:                   = 1458.1 mJ
```

**Observação:** SF7 consome **menos** por ciclo (120 mJ) do que SF9 (133 mJ), mas a diferença é pequena (9%) porque ambos compartilham as mesmas janelas RX (115.5 mJ).

### Acesso ao Breakdown

```python
# Por EnergyModel
breakdown = device.energy_model.get_energy_breakdown()
# {'SLEEP': 17.6, 'STANDBY': 55.4, 'TX': 217.2, 'RX': 1306.8}

# Por analytics.compute_metrics()
# energy_breakdown_percent: {'TX': 14%, 'RX': 82%, 'STANDBY': 3%, 'SLEEP': 1%}
```

---

## 10. Estimativa de Tempo de Vida da Bateria

### Cálculo Direto

```python
# battery.py
def estimate_lifetime_days(self, avg_consumption_mj_per_hour):
    hours = self.remaining_mj / avg_consumption_mj_per_hour
    return hours / 24
```

### Estimativa Manual

```
Bateria 2400 mAh @ 3.3V:
  capacity = 2400 × 3.3 × 3.6 = 28,512 mJ

Consumo por hora (SF9, λ=1/300s, 12 TX/hora):
  E_hora = 1597 mJ/hora

Lifetime = 28,512 / 1597 = 17.9 horas
```

Este resultado óbvio mostra que **sem sleep profundo** (com janelas RX sempre abertas), a bateria esgota em < 1 dia.

### Lifetime Realista (sem trx2 desnecessário)

Se o dispositivo só abre RX2 quando não recebeu DL em RX1 (90% das vezes sem DL):
```
E_ciclo_com_DL = 120 + 37 + 4.62 = 161.6 mJ  (RX1 com sucesso, sem RX2)
E_ciclo_sem_DL = 120 + 37 + 74 + 4.62 = 235.6 mJ  (RX2 também)

Com 10% confirmados + 10% DLs:
  E_média ≈ 0.9 × 120 + 0.1 × 235.6 ≈ 131.6 mJ/ciclo

Lifetime = 28512 / (131.6 × 12) ≈ 18.1 horas (similar)
```

### Comparação por SF (capacidade 2400 mAh, λ=1/300s)

| SF  | E_ciclo (mJ) | E_hora (12 TX) | Lifetime (h) | Lifetime (dias) |
|-----|-------------|----------------|--------------|-----------------|
| SF7 | 120.0       | 1440           | 19.8 h       | 0.83 d          |
| SF8 | 124.6       | 1495           | 19.1 h       | 0.79 d          |
| SF9 | 133.6       | 1603           | 17.8 h       | 0.74 d          |
| SF10| 151.7       | 1820           | 15.7 h       | 0.65 d          |
| SF11| 187.9       | 2255           | 12.6 h       | 0.53 d          |
| SF12| 260.3       | 3124           | 9.1  h       | 0.38 d          |

> **Problema:** Com trx2=2s (janela RX2 sempre aberta), o lifetime é muito baixo.
> Em implementações reais, dispositivos LoRaWAN fecham RX2 se receberam DL em RX1,
> e usam SF mais eficientes quando ADR otimiza a configuração.

### Lifetime com λ Baixo (1 TX por hora)

| SF  | E_ciclo (mJ) | E_sleep (mJ/hora) | E_hora total | Lifetime |
|-----|-------------|-------------------|--------------|----------|
| SF7 | 120.0       | 17.7              | 137.7        | 208 h = 8.7 d |
| SF9 | 133.6       | 17.7              | 151.3        | 189 h = 7.9 d |
| SF12| 260.3       | 17.7              | 278.0        | 103 h = 4.3 d |

Com λ baixo, o sleep é mais relevante mas ainda pequeno comparado às janelas RX.

---

## 11. Energy Harvesting na Simulação

### Configuração

```python
# Em Network.__init__() / parametors
energy_harvesting = {
    "model": "solar",
    "peak_power_mw": 100
}
battery_capacity_mah = 2400
```

### Evento ENERGY_HARVEST

```python
# network.py — agendado a cada 60s
def _on_energy_harvest(self):
    for device in self.devices:
        if device.battery and device.battery.harvester:
            energy = device.battery.harvester.harvest_energy(
                battery    = device.battery,
                sim_time_s = self.current_time,
                duration_s = 60.0
            )
    self.scheduler.schedule(60, ENERGY_HARVEST, self._on_energy_harvest)
```

### Energia Colhida por Intervalo (modelo solar, peak=100mW)

| Hora do dia | P_solar (mW) | E em 60s (mJ) |
|-------------|-------------|----------------|
| 0–6h (noite)| 0           | 0              |
| 6h          | 0           | 0              |
| 9h          | 70.7 mW     | 4,242          |
| 12h (pico)  | 100.0 mW    | 6,000          |
| 15h         | 70.7 mW     | 4,242          |
| 18h         | 0           | 0              |

**Comparação com consumo:**
```
Consumo SF9 (1 TX/300s) ≈ 1597 mJ/hora
Coleta solar pico (12h) ≈ 6000 × 60/60 = 6000 mJ/hora pico
                   média ≈ 1375 mJ/hora ao longo do dia

→ No horário de pico solar, a coleta supera o consumo (net positive)
→ Na média diária, coleta ≈ consumo → device potencialmente self-sustaining com λ=1/300s
```

### Efeito no SoC ao Longo da Simulação

Com simulação de 1 hora (3600s) iniciando às 12h (pico solar):
```
Consumo em 1h: ~1597 mJ
Coleta em 1h:  P_media(12h→13h) ≈ 97 mW × 3600s ≈ 349,200 mW·s → impossível
                                                                  (painel ≠ painel inteiro)

Na prática: peak_power_mw = 100mW (painel pequeno, ~100cm²)
  Coleta em 1h (de 12h a 13h):
    E = ∫(12→13) 100 × sin(π(h−6)/12) dh × 3600
    ≈ 100 × 0.966 × 3600 = 347,760 mJ → muito alto

  Isso significa que o EnergyHarvester modela ~100 mW constante ao longo de 60s.
  Com peak=1mW (microcélula solar): coleta ≈ 3477 mJ/hora vs consumo 1597 → sustentável!
```

---

## 12. Métricas de Energia em analytics.py

O `compute_metrics()` extrai as seguintes métricas energéticas:

```python
"energy": {
    "total_network_mj":       sum(d.energy_model.energy_consumed for d in devices),
    "avg_device_mj":          mean(energy per device),
    "min_device_mj":          min(energy per device),
    "max_device_mj":          max(energy per device),

    "avg_per_sf_mj": {
        sf: mean(energy for devices with this SF)
    },

    "breakdown_percent": {
        "TX":      (E_TX / E_total) × 100,
        "RX":      (E_RX / E_total) × 100,
        "STANDBY": (E_STANDBY / E_total) × 100,
        "SLEEP":   (E_SLEEP / E_total) × 100,
    },

    "avg_battery_lifetime_days":  # se battery_capacity_mah configurado
        mean(battery.estimate_lifetime_days(avg_consumption) for all devices)
}
```

### export_device_summary() — Por Dispositivo

```
CSV: device_id, sf, tx_power, freq, class, x, y, coverage, energy_mj, packets_sent, packets_collided, pdr_percent
```

### export_npz() — Arrays Numpy

```python
"device_energy_mj"   # array float64 [num_devices]
"device_sf"          # array int [num_devices]
# Permite análise numpy de correlação SF × energia
```

---

## 13. Análise Quantitativa por SF

### Consumo por Transmissão (TX power=14dBm, λ=1/300s)

```
E_TX_por_SF [mJ]    = [4.53, 9.05, 18.10, 36.20, 72.40, 144.82]
E_ciclo_por_SF [mJ] = [120.0, 124.6, 133.6, 151.7, 187.9, 260.3]
                       (SF7 → SF12)
```

### Corrente Média por SF (1 hora, λ=1/300s)

```
I_avg [µA] = E_hora [mJ] / (V × 3600s) × 1000

SF7:  1440 / (3.3 × 3600) × 1000 = 121 µA
SF9:  1597 / (3.3 × 3600) × 1000 = 134 µA
SF12: 3124 / (3.3 × 3600) × 1000 = 263 µA
```

> **Referência:** LoRaWAN baixa potência típico: 10–50 µA médio. Os valores acima são altos porque as janelas RX estão sempre abertas (comportamento ideal de Class A com trx2=2s).

### Redução de Corrente com ADR

Se ADR move um dispositivo de SF12 para SF7:
```
Redução de E_ciclo: (260.3 - 120.0) / 260.3 = 54%
Redução de corrente média: (263 - 121) / 263 = 54%

Ganho de lifetime: 0.38 dias → 0.83 dias (2.2×)
```

### Impacto das Janelas RX no Lifetime

Se `trx2 = 0` (dispositivo nunca abre RX2, apenas RX1):
```
E_ciclo(SF9, sem RX2) = 18.1 + 4.62 + 36.96 = 59.7 mJ
E_hora = 59.7 × 12 = 716 mJ/h
Lifetime = 28512 / 716 = 39.8 horas = 1.66 dias

vs. com RX2: 17.8 horas = 0.74 dias
Ganho: 2.2× de lifetime eliminando RX2 quando não há DL
```

---

## 14. Comparação com ns-3 e FLoRa

| Aspecto                    | PyLoRaWAN                    | ns-3 LoRaWAN               | FLoRa                      |
|----------------------------|------------------------------|----------------------------|----------------------------|
| Modelo base                | `LoraRadioEnergyModel`       | `LoraRadioEnergyModel`     | Próprio (similar)          |
| Estados                    | SLEEP, STANDBY, RX, TX       | SLEEP, STANDBY, RX, TX     | TX, RX, Sleep              |
| Corrente TX               | Por potência (tx_current_table) | Fixo por estado           | Fixo                       |
| STANDBY (RX delay)        | 1.4 mA ✓                    | 1.4 mA ✓                   | Não separado               |
| Sleep corrente            | 0.0015 mA ✓                  | 0.0015 mA ✓                | ~0.001 mA                  |
| RX corrente               | 11.2 mA ✓                   | 11.2 mA ✓                  | ~10 mA                     |
| Sleep entre TXs           | ✓ (Sprint 5)                 | ✓                          | Parcial                    |
| Battery model             | ✓ (BatteryModel)             | ✓ (BasicEnergySource)      | ✗                          |
| Energy harvesting         | ✓ (solar + constante)        | ✗                          | ✗                          |
| Breakdown por estado      | ✓ (energy_breakdown)         | ✓                          | Parcial                    |
| Corrente média (µA)       | ✓ (get_avg_current_ua)       | ✓                          | ✗                          |

---

## 15. Exemplos Numéricos Completos

### Exemplo 1: Ciclo TX completo (SF9, 14 dBm)

```
device.sf = 9, device.tx_power = 14, device.bw = 125000
airtime = 0.1444 s, sim_time = 300.0 s

update_energy(device, 0.1444, sim_time=300.0):

  [Sleep desde último ciclo]
  _sim_time_last_tx_end = 0.0 (primeiro ciclo)
  sleep_duration = 300.0 - 0.0 = 300.0 s
  E_sleep = 0.0015 × 3.3 × 300.0 = 1.485 mJ

  t = state_entry_time (interno)

  [TX @ 14 dBm]
  transition(TX, t, tx_power=14)
  → corrente_anterior(SLEEP) × V × duration → já acumulado no sleep acima
  corrente TX = tx_current_table[14] = 38.0 mA
  t += 0.1444

  [STANDBY — RD1]
  transition(STANDBY, t)
  → E_TX = 38.0 × 3.3 × 0.1444 = 18.10 mJ
  t += 1.0

  [RX1]
  transition(RX, t)
  → E_STANDBY1 = 1.4 × 3.3 × 1.0 = 4.62 mJ
  t += 1.0

  [STANDBY — RD2 = 0s]
  transition(STANDBY, t)
  → E_RX1 = 11.2 × 3.3 × 1.0 = 36.96 mJ
  t += 0.0   (rx_delay2_duration = max(0, 2-1-1) = 0)

  [RX2]
  transition(RX, t)
  → E_STANDBY2 = 1.4 × 3.3 × 0.0 = 0.00 mJ
  t += 2.0

  [SLEEP]
  transition(SLEEP, t)
  → E_RX2 = 11.2 × 3.3 × 2.0 = 73.92 mJ

  _sim_time_last_tx_end = 300.0 + 0.1444 + 1 + 1 + 0 + 2 = 304.14 s

TOTAL CICLO:
  E_sleep      =  1.485 mJ
  E_TX         = 18.100 mJ
  E_STANDBY1   =  4.620 mJ
  E_RX1        = 36.960 mJ
  E_STANDBY2   =  0.000 mJ
  E_RX2        = 73.920 mJ
  ─────────────────────────
  Total        = 135.085 mJ
```

### Exemplo 2: Depleção de Bateria

```
BatteryModel(capacity_mah=2400, voltage=3.3)
capacity_mj = 2400 × 3.3 × 3.6 = 28,512 mJ

Consumo SF9, λ=1/300s, 12 TX/hora:
  E/hora = 135.1 × 12 = 1621.2 mJ/hora

Lifetime = 28512 / 1621.2 = 17.59 horas

Após 18 horas (18 × 12 = 216 ciclos):
  Consumido = 135.1 × 216 = 29,181 mJ > 28,512 mJ
  → battery.depleted = True durante o 211º ciclo
  → device para de transmitir na simulação
```

### Exemplo 3: ADR melhora lifetime

```
Dispositivo a 1km, outdoor, SNR_médio = 20 dB (excelente cobertura)
ADR ativo:
  required_snr[SF9] = -12.5 dB
  margin = 20 - (-12.5) - 10 = 22.5 dB
  n_steps = int(22.5 / 3) = 7

  Passo 1: SF reduz SF9→SF8→SF7 (−2 steps = 2 SF)
  Passo 2: TX power reduz 14→12→10→8→6 (-5 steps × 2 dBm = -10 dBm)

  Novo SF = 7, TX power = 4 dBm
  Nova corrente TX = tx_current_table[4] = 24.7 mA
  Novo airtime = 36.1ms
  E_TX = 24.7 × 3.3 × 0.0361 = 2.94 mJ  (vs 18.1 mJ antes)
  E_ciclo = 2.94 + 115.5 = 118.5 mJ  (vs 133.6 mJ)

  Ganho de lifetime por ADR: 133.6 / 118.5 = 1.13×  (13% de ganho)
  (limitado porque janelas RX dominam)
```

### Exemplo 4: Energy Harvesting — SoC ao longo do dia

```
Dispositivo com bateria 2400mAh, solar peak=100mW, SF9, λ=1/300s
Simulação: 12 horas (de 6h às 18h)

A cada 60s:
  t=0 (6h):   P=0mW,    E=0 mJ colhidos
  t=3600 (7h): P=25.9mW, E=1556 mJ colhidos (em 1h)
  t=7200 (8h): P=50.0mW, E=3000 mJ colhidos
  t=10800 (9h): P=70.7mW, E=4242 mJ colhidos
  t=18000 (11h): P=93.3mW, E=5598 mJ
  t=21600 (12h): P=100mW,  E=6000 mJ

Consumo em 12h: 1621.2 mJ/h × 12h = 19,454 mJ
Coleta total (6h→18h): ≈ 1,375,099 × (12/24) ≈ 687,549 mJ → muito alto pois P_peak é alto

Com P_peak=1mW (realista para micro painel IoT):
  Coleta total = 687,549 / 100 = 6875 mJ em 12h
  Consumo = 19,454 mJ
  Net = -12,579 mJ (bateria ainda descarregando, mas a taxa é menor)

SoC ao longo do dia:
  6h:  SoC = 100%
  12h: SoC = (28512 - 6000*) / 28512 ≈ 79%   *consumo - coleta na manhã
  18h: SoC ≈ 32%
  6h+1dia: SoC ≈ dependente do perfil noturno (sem coleta, só consumo)
```

---

*Documentação gerada a partir do código-fonte — Sprint 7 concluído.*
