# Mobility Updates — PyLoRaWAN

## Visão Geral

Quando a mobilidade está habilitada, o simulador atualiza periodicamente a posição de cada End Device móvel de acordo com o modelo de mobilidade selecionado. Após cada passo de movimento, o módulo de propagação recalcula a distância entre o device e os gateways, o que pode afetar o RSSI, o SF necessário e a cobertura. Esse mecanismo permite avaliar o desempenho da rede em cenários dinâmicos como sensores móveis ou nós IoT em movimento.

---

## 1. Arquivos Envolvidos

| Arquivo | Responsabilidade |
|---|---|
| `enddevice.py` | `move()` — aplica deslocamento; `update_coverage_status()` — reavalia link budget |
| `network.py` | `_on_mobility_update()` — handler periódico; `find_best_gateway()` — recalcula RSSI/distância |
| `parametors.py` | `speed`, `lambda_rate`, `mobility_enabled`, `model` |

---

## 2. Ativação e Agendamento

A mobilidade é ativada pelo flag global `mobility_enabled` de `parametors.py`. Em `simulate_transmissions()`, o primeiro evento é agendado antes de entrar no loop:

```python
# network.py — simulate_transmissions()
if self.mobility_enabled:
    self.scheduler.schedule(1.0, EventType.MOBILITY_UPDATE,
                            self._on_mobility_update)
```

O handler se reagenda a cada execução, criando um loop periódico com intervalo fixo de **1,0 s**:

```python
def _on_mobility_update(self):
    for device in self.devices:
        if device.mobility_enabled:
            _, new_position = device.move(self.area_size,
                                          device.mobility_enabled,
                                          device.model)
            self.positions["devices"][device.device_id] = new_position
            device.update_coverage_status()

    # amostra energia total para energy_log
    total_energy = sum(d.energy_model.energy_consumed for d in self.devices) / 1000
    self.energy_log.append(total_energy)

    self.scheduler.schedule(1.0, EventType.MOBILITY_UPDATE,
                            self._on_mobility_update)
```

**Verificação por device:** o flag `device.mobility_enabled` é verificado individualmente. `set_mobility()` pode alterar a mobilidade de todos os devices sem remover o evento do scheduler — devices com flag `False` são simplesmente ignorados no loop.

---

## 3. Modelo de Mobilidade — Random Walk

### 3.1 `move()` em `EndDevice`

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
| Direção | `θ ~ U(0, 2π)` — uniforme em todas as direções |
| Magnitude | `speed × 1 s` (intervalo fixo do handler) |
| Fronteira | Reflexiva — `clamp` a `[0, area_size]` |
| Magnitude padrão | `1.5 m/s × 1 s = 1.5 m` por passo |

**`time_interval`** (retorno de `move()`) é amostrado de `Exponential(1/λ)` mas **não é usado** pelo handler — o reagendamento é sempre fixo em 1,0 s. O valor existe para extensões customizadas.

### 3.2 Outros modelos declarados

`parametors.py` lista `model = "random_walk"` com comentário `"gauss_markov" "levy_walk" "matrix"`. Somente `"random_walk"` está implementado. O atributo `device.model` é armazenado para extensão futura.

---

## 4. Recálculo de Propagação após Movimento

Após cada passo, `update_coverage_status()` é chamado para reavliar o link budget com a nova posição:

```python
# enddevice.py
def update_coverage_status(self):
    best_gateway, _, _ = self.network.find_best_gateway(self)
    self.coverage_status = best_gateway is not None
```

`find_best_gateway()` recalcula, para cada gateway, a distância euclidiana 3D e o RSSI:

```python
# network.py — find_best_gateway()
distance = max(
    sqrt((device.x - gw.x)² + (device.y - gw.y)² + (ht_m - hr_m)²),
    1.0                   # mínimo de 1m para evitar log(0)
)

path_loss  = self.pathloss(distance, device.freq, model_pathloss, device.x, device.y)
path_loss += self.get_building_penetration(device)   # +perda indoor se is_indoor

rssi = device.tx_power + ed_antenna_gain + gw_antenna_gain - path_loss
```

O RSSI calculado é comparado à sensibilidade do receptor do gateway (SX1301):

```python
gw_sensitivity = gw_sensitivity_table.get((device.sf, device.bw), fallback)

if rssi > gw_sensitivity:
    available_gateways.append((gateway, distance, rssi))
```

Se nenhum gateway apresentar RSSI acima do limiar, `device.coverage_status = False` e o device entra em modo de espera no próximo evento de transmissão.

### 4.1 Sensibilidade SX1301 (referência de cobertura)

| SF | BW 125 kHz | BW 250 kHz |
|---|---|---|
| SF7  | −130.0 dBm | −127.0 dBm |
| SF8  | −132.5 dBm | −129.5 dBm |
| SF9  | −135.0 dBm | −132.0 dBm |
| SF10 | −137.5 dBm | −134.5 dBm |
| SF11 | −140.0 dBm | −137.0 dBm |
| SF12 | −142.5 dBm | −139.0 dBm |

### 4.2 Componente 3D da distância

A altura relativa entre device (`ht_m = 1.5 m`) e gateway (`hr_m = 30 m`) é incorporada:

```
distance = sqrt(d_horizontal² + (ht_m − hr_m)²)
         = sqrt(d_horizontal² + (1.5 − 30)² )
         = sqrt(d_horizontal² + 812.25)
```

Para distâncias horizontais acima de ~100 m, o componente vertical (±28.5 m) é desprezível. Para devices próximos ao gateway a diferença de altura pode ser relevante.

---

## 5. Impacto na Transmissão

`coverage_status` é verificado em `_on_device_send()` antes de cada transmissão:

```python
# network.py — _on_device_send()
best_gateway, _, best_rssi = self.find_best_gateway(device)
if best_gateway is None or not device.coverage_status:
    next_delay = np.random.exponential(1.0 / max(device.lambda_rate, 0.001))
    self.scheduler.schedule(next_delay, EventType.DEVICE_SEND,
                            self._on_device_send, device)
    return   # adia transmissão — device fora de cobertura
```

O device não transmite e agenda novo evento com delay Poisson normal — reavaliando cobertura na próxima tentativa (que pode coincidir com o device tendo se movido de volta para a cobertura).

---

## 6. Atualização de Posição no Registro

Após cada `move()`, a posição é atualizada no dicionário centralizado:

```python
self.positions["devices"][device.device_id] = new_position  # (x, y) em metros
```

`self.positions` é usado por:
- `visualize_network()` — renderiza trajetória de cada device no plot
- `find_best_gateway()` — usa `device.x` e `device.y` diretamente (não `positions`)

Os plots gerados por `visualize_network()` mostram a **última posição** de cada device; devices fora de cobertura aparecem em cinza.

---

## 7. Fluxo Completo por Passo de Mobilidade

```
MOBILITY_UPDATE @ t = k × 1.0 s
  │
  ├── Para cada device com mobility_enabled=True:
  │     │
  │     ├── move(area_size, True, "random_walk")
  │     │     ├── θ = U(0, 2π)
  │     │     ├── dx = 1.5 × cos(θ),  dy = 1.5 × sin(θ)
  │     │     ├── device.x = clamp(device.x + dx, 0, area_size)
  │     │     └── device.y = clamp(device.y + dy, 0, area_size)
  │     │
  │     ├── positions["devices"][id] = (device.x, device.y)
  │     │
  │     └── update_coverage_status()
  │           └── find_best_gateway(device)
  │                 ├── Para cada gateway:
  │                 │     distance = sqrt(Δx² + Δy² + Δh²)
  │                 │     path_loss = pathloss(distance, freq, model)
  │                 │               + building_penetration(device)
  │                 │     rssi = tx_power + G_ed + G_gw - path_loss
  │                 │     if rssi > gw_sensitivity[sf, bw]:
  │                 │         available_gateways.append(...)
  │                 └── device.coverage_status = len(available) > 0
  │
  ├── energy_log.append(Σ energy_consumed / 1000)   ← snapshot J
  │
  └── scheduler.schedule(1.0, MOBILITY_UPDATE, _on_mobility_update)


Próximo DEVICE_SEND do device:
  └── find_best_gateway(device)   ← segundo recálculo com posição atual
      if None or coverage_status=False → adia transmissão
      else → prossegue TX com best_gateway e rssi atualizados
```

---

## 8. Deslocamento Esperado (Random Walk 2D)

Para a simulação de referência (`speed=1.5 m/s`, `T=3600 s`, `Δt=1 s`):

```
Passos por device: N = 3600
Tamanho do passo: Δ = 1.5 m

Comprimento total do caminho: N × Δ = 5400 m
Deslocamento RMS (distância da origem): Δ × √N = 1.5 × 60 = 90 m
```

Para uma área de 10 km × 10 km, o desvio de 90 m em relação à posição inicial é pequeno — devices permanecem essencialmente fixos em relação ao gateway. O efeito da mobilidade torna-se significativo com velocidades maiores (≥ 10 m/s) ou simulações mais longas.

**Probabilidade de sair de raio R da posição inicial:**

```
P(r > R) ≈ exp(−R² / (N × Δ²))

Para R = 500 m: exp(−500² / (3600 × 1.5²)) = exp(−30.9) ≈ 0
Para R = 100 m: exp(−100² / (3600 × 1.5²)) = exp(−1.23) ≈ 29%
Para R = 50 m:  exp(−50²  / (3600 × 1.5²)) = exp(−0.31) ≈ 73%
```

---

## 9. Parâmetros Configuráveis

| Parâmetro | Arquivo | Default | Efeito |
|---|---|---|---|
| `mobility_enabled` | `parametors.py` | `True` | Liga/desliga mobilidade globalmente |
| `speed` | `parametors.py` | `1.5 m/s` | Magnitude do passo por segundo |
| `model` | `parametors.py` | `"random_walk"` | Modelo de mobilidade (somente random_walk implementado) |
| `device.mobility_enabled` | `enddevice.py` | `True` | Flag por device; alterável via `set_mobility()` |
| Intervalo do handler | `network.py` | `1.0 s` fixo | Frequência de atualização |
