# Reference Scenario — PyLoRaWAN

## Visão Geral

O cenário de referência modela um deployment LoRaWAN típico com um único gateway posicionado no centro da área e 50 end devices distribuídos em uma região circular de raio 5 km. Esse cenário representa cobertura IoT de longo alcance em ambiente suburbano ou misto, e serve de base para comparação com estudos de simulação publicados (FLoRa, ns-3, LoRaWANSim).

---

## 1. Tabela de Parâmetros

| Parâmetro | Valor | Origem no Código |
|---|---|---|
| End Devices | 50 | `num_devices = 50` (`parametors.py`) |
| Gateways | 1 | `num_gateways = 1` (`parametors.py`) |
| Raio de deployment | 5 km | `deployment_radius = area_size/2 = 5000 m` |
| Duração da simulação | 3600 s | `simulation_time = 3600` (`parametors.py`) |
| Payload | 20 bytes | `pl = 20` (`parametors.py`) |
| Intervalo médio de TX | 300 s | `lambda_rate = 1/300 ≈ 0.00333 Hz` |
| TX Power | 14 dBm | `tx_power = 14` (`parametors.py`) |
| Largura de banda | 125 kHz | `bw = [125000]` (`parametors.py`) |
| Coding Rate | 4/5 | `cr = 1` (`parametors.py`) |
| SF range | SF7–SF12 | `sf_range = [7,8,9,10,11,12]` (`parametors.py`) |
| Banda de frequência | EU868 | `region_name = "EU868"` |
| Noise Figure | 6 dB | `noise_figure = 6` (`parametors.py`) |
| Modelo de sensibilidade | SX1272 | `sensitivity_table` (`parametors.py`) |

---

## 2. Topologia e Deployment

### 2.1 Posição do Gateway

Para `num_gateways = 1`, `initialize_gateways()` em `network.py` posiciona o gateway no centro da área via grade 1×1:

```python
x_grid = linspace(0, area_size, 1, endpoint=False) + area_size / (2 × 1) = 5000 m
y_grid = linspace(0, area_size, 1, endpoint=False) + area_size / (2 × 1) = 5000 m
```

Gateway posicionado em `(5000, 5000)` — centro exato da área 10 km × 10 km.

### 2.2 Distribuição dos Devices

O cenário de referência usa `deployment_type="circular"` com raio 5 km:

```python
# network.py — initialize_devices()
center = area_size / 2   # = 5000 m
positions = deploy_circular(num_devices, deployment_radius, center, center)
```

`deploy_circular()` em `deployment.py` usa a transformação `r = R × √U` para garantir distribuição uniforme na área do disco:

```python
def deploy_circular(n_devices, radius, center_x, center_y):
    for _ in range(n_devices):
        r     = radius * np.sqrt(np.random.uniform(0, 1))
        theta = np.random.uniform(0, 2 * np.pi)
        x     = center_x + r * cos(theta)
        y     = center_y + r * sin(theta)
```

A raiz quadrada corrige a tendência de concentração no centro que ocorreria com `r = R × U` diretamente — a densidade é uniforme em área (não em raio).

**Distância máxima** de qualquer device ao gateway: 5000 m.
**Distância média esperada** para distribuição uniforme em disco: `2R/3 ≈ 3333 m`.

---

## 3. Geração de Tráfego

O intervalo médio entre transmissões é de 300 s. A geração segue um processo de Poisson com taxa:

```
λ = 1 / 300 s = 0.00333 Hz
```

Configuração correspondente em `parametors.py`:

```python
lambda_rate = 1.0 / 300   # ≈ 0.00333 Hz
```

O inter-arrival de cada transmissão é amostrado da distribuição exponencial em `network.py`:

```python
next_delay = np.random.exponential(1.0 / max(device.lambda_rate, 0.001))
```

O delay inicial de cada device é uniformemente distribuído em `[0, 300 s]` para dispersar os eventos de primeira transmissão e evitar bursts iniciais:

```python
initial_delay = random.uniform(0, 1.0 / max(device.lambda_rate, 0.001))
```

**Transmissões esperadas por device** em 3600 s: `3600 / 300 = 12`.
**Total esperado na rede**: `50 × 12 = 600` pacotes.

---

## 4. Parâmetros de Rádio

### 4.1 Airtime por SF (BW 125 kHz, PL 20 B, CR 4/5)

Calculado por `calculate_airtime()` em `enddevice.py`:

| SF | Airtime (ms) | Noise Floor (dBm) | Sensibilidade SX1272 (dBm) |
|---|---|---|---|
| SF7 | 71.9 | −116.95 | −124.0 |
| SF8 | 133.6 | −116.95 | −127.0 |
| SF9 | 246.8 | −116.95 | −130.0 |
| SF10 | 452.6 | −116.95 | −133.0 |
| SF11 | 987.1 | −116.95 | −135.5 |
| SF12 | 1974.1 | −116.95 | −137.0 |

### 4.2 Alcance Máximo por SF

Link budget com `tx_power=14 dBm`, `G_ed=0 dBi`, `G_gw=3 dBi`, modelo `log_normal_shadowing` (γ=3.76):

```
RSSI_min = sensibilidade_SX1272(SF) = −124 a −137 dBm
Link margin = 14 + 0 + 3 − PL(d) > sensibilidade

PL(d) = 7.7 + 10 × 3.76 × log₁₀(d)
```

O gateway usa sensibilidade SX1301 (mais sensível ~6 dB):

| SF | SX1301 (dBm) | Alcance máximo estimado (γ=3.76) |
|---|---|---|
| SF7 | −130.0 | ~2.5 km |
| SF9 | −135.0 | ~4.0 km |
| SF12 | −142.5 | ~7.5 km |

Dentro do raio de 5 km, SF7 e SF8 podem não cobrir todos os devices; SF9–SF12 cobrem a área completa para devices sem obstrução.

---

## 5. Propagação no Cenário de Referência

| Componente | Configuração |
|---|---|
| Modelo de path loss | `log_normal_shadowing` (d₀=1m, PL₀=7.7, γ=3.76, σ=3.57 dB) |
| Shadowing | Independente por pacote (`X_σ ~ N(0, 3.57)`) |
| Penetração predial | 30% dos devices `is_indoor` — perda adicional `Lognormal(ln(20), 0.5)` |
| Ruído térmico | −116.95 dBm @ 125 kHz, T=294.15 K, NF=6 dB |
| Interferência | Modelo Semtech AN1200.18 (co-SF threshold: 1 dB) |

---

## 6. Reproducibilidade

A semente aleatória fixa garante que posições de devices, SFs iniciais, sequências de tráfego e valores de shadowing sejam idênticos entre execuções:

```python
random_seed = 42

# network.py — Network.__init__()
if random_seed is not None:
    random.seed(random_seed)
    np.random.seed(random_seed)
```

Para varreduras de parâmetros multi-seed, a semente pode ser passada via argumento ao instanciar `Network`, permitindo análise estatística com múltiplas realizações independentes.

---

## 7. Instanciação do Cenário de Referência

```python
from network import Network

network = Network(
    num_devices      = 50,
    num_gateways     = 1,
    area_size        = 10000,          # 10 km × 10 km
    lambda_rate      = 1.0 / 300,      # 300 s inter-arrival médio
    speed            = 1.5,
    sf_range         = [7, 8, 9, 10, 11, 12],
    tx_power         = 14,
    frequency_mhz    = [868.1, 868.3, 868.5],
    ht_m             = 1.5,
    hr_m             = 30,
    bw               = [125000],
    cr               = 1,
    pl               = 20,
    simulation_time  = 3600,
    adr_enabled      = True,
    model_pathloss   = "log_normal_shadowing",
    region_name      = "EU868",
    deployment_type  = "circular",
    deployment_radius = 5000,          # 5 km
    indoor_ratio     = 0.3,
)

network.simulate_transmissions()
```
