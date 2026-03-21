# Mobility and Traffic Modeling — PyLoRaWAN

## Visão Geral

O simulador suporta mobilidade de devices e dois padrões de geração de tráfego. Quando a mobilidade está habilitada, as posições são atualizadas periodicamente a cada 1 s via evento `MOBILITY_UPDATE`, recalculando distâncias e condições de canal após cada passo. A geração de tráfego usa chegadas de Poisson com taxa configurável, com um campo de tipo de tráfego (`traffic_type`) por device para extensões analíticas.

---

## 1. Modelos de Mobilidade

### 1.1 Estado de Implementação

O atributo `model` em `parametors.py` lista quatro modelos:

```python
model = "random_walk"   # "gauss_markov" "levy_walk" "random_walk" "matrix"
```

Apenas `"random_walk"` está implementado. Os demais (`"gauss_markov"`, `"levy_walk"`, `"matrix"`) constam como comentários em `parametors.py` e o atributo `device.model` é armazenado em cada device para extensão futura, mas não altera o comportamento de `move()`.

### 1.2 Random Walk — `move()` em `enddevice.py`

```python
def move(self, area_size, mobility_enabled, model):
    if not mobility_enabled:
        return 0, (self.x, self.y)

    time_interval = np.random.exponential(1 / self.lambda_rate)  # não usado pelo handler
    angle = random.uniform(0, 2 * np.pi)                         # direção uniforme
    dx = self.speed * np.cos(angle)
    dy = self.speed * np.sin(angle)
    self.x = max(0, min(area_size, self.x + dx))                 # fronteira reflexiva
    self.y = max(0, min(area_size, self.y + dy))
    return time_interval, (self.x, self.y)
```

**Propriedades do passo:**

| Propriedade | Valor |
|---|---|
| Direção | `θ ~ U(0, 2π)` — uniforme, sem memória de direção anterior |
| Magnitude | `speed × Δt = 1.5 m/s × 1 s = 1.5 m` por passo |
| Fronteira | Reflexiva — `clamp` a `[0, area_size]` |

O campo `time_interval` (amostrado de `Exponential(1/λ)`) é retornado por `move()` mas **não é usado** pelo handler — o reagendamento é sempre fixo em 1,0 s.

---

## 2. Atualização Periódica de Posição — `_on_mobility_update()`

A mobilidade é habilitada pelo flag `mobility_enabled` em `parametors.py`. Em `simulate_transmissions()`, o primeiro evento é agendado antes do loop:

```python
if self.mobility_enabled:
    self.scheduler.schedule(1.0, EventType.MOBILITY_UPDATE, self._on_mobility_update)
```

O handler se reagenda a cada execução, criando um loop fixo de 1,0 s:

```python
def _on_mobility_update(self):
    for device in self.devices:
        if device.mobility_enabled:                    # flag por device
            _, new_position = device.move(
                self.area_size, device.mobility_enabled, device.model)
            self.positions["devices"][device.device_id] = new_position
            device.update_coverage_status()

    total_energy = sum(d.energy_model.energy_consumed for d in self.devices) / 1000
    self.energy_log.append(total_energy)              # snapshot de energia (J)

    self.scheduler.schedule(1.0, EventType.MOBILITY_UPDATE, self._on_mobility_update)
```

A verificação `device.mobility_enabled` é por device — `set_mobility(False)` desabilita todos os devices sem remover o evento do scheduler:

```python
def set_mobility(self, mobility_enabled):
    for device in self.devices:
        device.mobility_enabled = mobility_enabled
```

---

## 3. Recálculo de Canal após Movimento

Após cada passo, `update_coverage_status()` reavalia o link budget com a nova posição:

```python
def update_coverage_status(self):
    best_gateway, _, _ = self.network.find_best_gateway(self)
    self.coverage_status = best_gateway is not None
```

`find_best_gateway()` recalcula distância 3D e RSSI para cada gateway:

```python
distance = max(sqrt((device.x - gw.x)² + (device.y - gw.y)² + (ht_m - hr_m)²), 1.0)
path_loss = pathloss(distance, freq, model) + building_penetration(device)
rssi      = tx_power + G_ed + G_gw - path_loss

if rssi > gw_sensitivity_table[(sf, bw)]:
    available_gateways.append((gw, distance, rssi))
```

Se nenhum gateway supera o limiar de sensibilidade SX1301, `coverage_status = False` e o próximo evento `DEVICE_SEND` adia a transmissão com um novo delay Poisson.

---

## 4. Deslocamento Esperado (Random Walk 2D)

Para a simulação de referência (`speed=1.5 m/s`, `T=3600 s`, `Δt=1 s`):

```
Passos por device: N = 3600
Tamanho do passo: Δ = 1.5 m

Comprimento total do caminho:   N × Δ = 5400 m
Deslocamento RMS da origem:     Δ × √N = 1.5 × 60 = 90 m
```

Para uma área de 10 km × 10 km, o desvio de 90 m em relação à posição inicial é pequeno. O efeito da mobilidade torna-se significativo com velocidades ≥ 10 m/s ou simulações mais longas.

**Probabilidade de sair do raio R da posição inicial:**

```
P(r > R) ≈ exp(−R² / (N × Δ²))

R = 500 m → exp(−500²/(3600×2.25)) = exp(−30.9) ≈ 0 %
R = 100 m → exp(−100²/(3600×2.25)) = exp(−1.23) ≈ 29 %
R =  50 m → exp( −50²/(3600×2.25)) = exp(−0.31) ≈ 73 %
```

---

## 5. Geração de Tráfego

### 5.1 Chegadas de Poisson — Scheduler

Todo o tráfego usa chegadas de Poisson com taxa `λ = lambda_rate` (pacotes/s). O inter-arrival é amostrado da distribuição exponencial:

```python
# network.py — _on_device_send()
next_delay = np.random.exponential(1.0 / max(device.lambda_rate, 0.001))
self.scheduler.schedule(next_delay, EventType.DEVICE_SEND, self._on_device_send, device)
```

O delay inicial de cada device é uniformemente distribuído no intervalo `[0, 1/λ]` para dispersar os primeiros eventos:

```python
# simulate_transmissions()
initial_delay = random.uniform(0, 1.0 / max(device.lambda_rate, 0.001))
```

**Relação entre `lambda_rate` e inter-arrival médio:**

| `lambda_rate` (Hz) | Inter-arrival médio | Pacotes/hora |
|---|---|---|
| 0.1 | 10 s | 360 |
| 0.033 | 30 s | 120 |
| 0.01 | 100 s | 36 |

### 5.2 Campo `traffic_type`

O atributo `device.traffic_type` é atribuído aleatoriamente no construtor:

```python
self.traffic_type = random.choice(["periodic", "sporadic", "critical"])
```

Este campo não altera o mecanismo de agendamento — todos os devices usam chegadas de Poisson independentemente do `traffic_type`. O campo é armazenado no objeto device e exportado nas métricas por device para análise e segmentação externa.

### 5.3 Pacotes Confirmados

A fração de pacotes confirmados (que abrem janelas RX1/RX2 e aguardam ACK) é controlada por:

```python
self.confirmed_ratio = 0.3    # 30% dos pacotes são confirmed uplinks
```

Por transmissão:

```python
packet.confirmed = (random.random() < device.confirmed_ratio)
```

Pacotes não confirmados não geram retransmissão em caso de ausência de ACK.

---

## 6. Parâmetros Configuráveis

| Parâmetro | Arquivo | Default | Efeito |
|---|---|---|---|
| `mobility_enabled` | `parametors.py` | `True` | Habilita mobilidade globalmente |
| `speed` | `parametors.py` | 1.5 m/s | Magnitude do passo por segundo |
| `model` | `parametors.py` | `"random_walk"` | Modelo de mobilidade (somente random_walk implementado) |
| `lambda_rate` | `parametors.py` | 0.1 Hz | Taxa de transmissão (inter-arrival = 1/λ s em média) |
| `simulation_time` | `parametors.py` | 3600 s | Duração total da simulação |
| `device.confirmed_ratio` | `enddevice.py` | 0.3 | Fração de pacotes confirmed uplink |
| Intervalo de mobilidade | `network.py` | 1.0 s fixo | Frequência de atualização de posição |
