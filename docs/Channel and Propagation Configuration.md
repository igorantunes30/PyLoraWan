# Channel and Propagation Configuration — PyLoRaWAN

## Visão Geral

O ambiente de propagação do cenário de referência é modelado pelo Log-Normal Shadowing, que introduz variabilidade realista na potência recebida devida a fatores ambientais. O ruído térmico é calculado pela fórmula padrão `N = kTB + NF`, garantindo consistência física nos cálculos de SINR.

---

## 1. Modelo Log-Normal Shadowing

Implementado em `pathloss()` em `network.py` sob `model_pathloss = "log_normal_shadowing"`:

```python
elif model_pathloss == "log_normal_shadowing":
    d0     = 1.0          # distância de referência (m)
    PL_d0  = 7.7          # perda na distância de referência (dB)
    gamma  = 3.76         # expoente de path loss
    sigma  = 3.57         # desvio padrão do shadowing (dB)
    X_sigma = np.random.normal(0, sigma)
    path_loss = PL_d0 + 10 * gamma * np.log10(distance / d0) + X_sigma
```

**Equação de path loss:**

$$PL(d) = PL(d_0) + 10\,\gamma\,\log_{10}\!\left(\frac{d}{d_0}\right) + X_\sigma$$

onde:
- $d_0 = 1\,\text{m}$ — distância de referência
- $PL(d_0) = 7.7\,\text{dB}$ — perda de referência
- $\gamma = 3.76$ — expoente de path loss (ambiente suburbano/urbano)
- $X_\sigma \sim \mathcal{N}(0,\,3.57^2)$ — componente de shadowing lognormal

**Parâmetros calibrados** para compatibilidade com FLoRa e ns-3 (`network_logger`, `parametors.py`).

### 1.1 Componente 3D da Distância

Antes de chamar `pathloss()`, a distância é calculada em 3D incluindo a diferença de altura entre device e gateway:

```python
distance = max(
    sqrt((device.x - gw.x)² + (device.y - gw.y)² + (ht_m - hr_m)²),
    1.0       # mínimo de 1 m para evitar log(0)
)
```

Com `ht_m = 1.5 m` e `hr_m = 30 m`: componente vertical = `(1.5 − 30)² = 812.25 m²`. Para distâncias horizontais acima de ~100 m, este termo é desprezível.

### 1.2 Shadowing por Amostra

O shadowing `X_σ` é amostrado independentemente por chamada — cada avaliação de `pathloss()` gera um valor diferente. Em cenários com mobilidade, isso implica variação de RSSI a cada transmissão, independentemente do deslocamento físico.

---

## 2. Shadowing Espacialmente Correlacionado (Opcional)

Quando `correlated_shadowing=True`, `CorrelatedShadowingMap` substitui a amostragem independente por um campo espacialmente contínuo:

```python
# network.py
class CorrelatedShadowingMap:
    def __init__(self, area_size, grid_step=50, sigma=3.57):
        self.grid = np.random.normal(0, sigma, (grid_points, grid_points))

    def get_shadowing(self, x, y):
        # Interpolação bilinear entre pontos do grid
        return (1-dx)*(1-dy)*v00 + dx*(1-dy)*v10 + (1-dx)*dy*v01 + dx*dy*v11
```

| Parâmetro | Valor | Descrição |
|---|---|---|
| `grid_step` | 50 m | Resolução espacial do grid |
| `sigma` | 3.57 dB | Desvio padrão (igual ao modelo independente) |
| Correlação | Implícita | Pontos adjacentes (≤50 m) recebem o mesmo valor de shadowing |

A interpolação bilinear garante continuidade — devices próximos experimentam shadowing similar, reproduzindo a correlação espacial observada em medições de campo.

---

## 3. Ruído Térmico — `calculate_noise_floor()`

Implementado em `network.py`:

```python
def calculate_noise_floor(self, bw):
    noise_floor = 10 * np.log10(k * temperatura * bw) + 30 + noise_figure
    return noise_floor   # dBm
```

**Derivação da fórmula:**

$$N\,[\text{dBm}] = 10\,\log_{10}(k \cdot T \cdot B) + 30 + NF$$

onde:
- $k = 1.38 \times 10^{-23}$ J/K — constante de Boltzmann (`parametors.py`)
- $T = 294.15$ K — temperatura de referência (~21 °C, `parametors.py`)
- $B$ — largura de banda do canal (Hz), passada como argumento
- $+30$ — conversão de W para mW (dBW → dBm)
- $NF = 6$ dB — fator de ruído do receptor SX1272 (`parametors.py`)

**Cálculo numérico para BW = 125 kHz:**

```
kTB = 1.38e-23 × 294.15 × 125000 = 5.075e-16 W
10 × log10(5.075e-16) = −152.95 dBW = −122.95 dBm
N  = −122.95 + 6 = −116.95 dBm
```

**Noise floor por largura de banda:**

| BW (kHz) | Noise Floor (dBm) |
|---|---|
| 125 | −116.95 |
| 250 | −113.94 |
| 500 | −110.93 |

---

## 4. Link Budget e RSSI

O RSSI recebido no gateway é calculado em `gateway.py` (`process_uplink()`) e em `network.py` (`find_best_gateway()`):

```python
rssi = device.tx_power + ed_antenna_gain + gw_antenna_gain - path_loss
```

**Parâmetros:**

| Termo | Valor | Descrição |
|---|---|---|
| `device.tx_power` | 14 dBm (inicial) | Reduzido pelo ADR |
| `ed_antenna_gain` | 0 dBi | Antena chip end device |
| `gw_antenna_gain` | 3 dBi | Antena omnidirecional gateway |
| `path_loss` | `PL(d) + X_σ + BPL` | Propagação + shadowing + indoor |

**BPL (Building Penetration Loss)** — adicionado para `device.is_indoor = True`:

```python
building_penetration = np.random.lognormal(mean=log(20), sigma=0.5)   # dB
```

Valor central: 20 dB; distribuição lognormal com σ=0.5 representa variabilidade entre tipos de construção.

---

## 5. SNR e SINR

### 5.1 SNR

Calculado em `gateway.py` após obter RSSI e noise floor:

```python
packet.snr = packet.rssi - packet.noise_floor   # dB
```

### 5.2 SINR — Modelo Energy-Based (G14)

O SINR incorpora a interferência acumulada de todos os pacotes em transmissão simultânea na mesma frequência:

```python
# gateway.py — process_uplink()
interference_power_linear = sum(
    other_rssi_linear × overlap_ratio
    for other_pkt if same_freq and temporal_overlap
)

sinr_linear = signal_linear / (interference_power_linear + noise_linear)
packet.sinr = 10 × log10(sinr_linear)
```

Quando não há interferência (`interference_power_linear = 0`): `SINR = SNR`.

**Sem interferência (isolado):**
```
SINR = RSSI − noise_floor = SNR
```

**Com interferência:**
```
SINR = RSSI_dBm − 10×log10(I_linear + N_linear)   <  SNR
```

---

## 6. Sensibilidade do Receptor

Dois conjuntos de tabelas são usados para verificação de cobertura:

**SX1272** (end device — `sensitivity_table` em `parametors.py`):

| SF | BW 125 kHz (dBm) | BW 250 kHz (dBm) |
|---|---|---|
| SF7 | −124.0 | −121.0 |
| SF8 | −127.0 | −124.0 |
| SF9 | −130.0 | −127.0 |
| SF10 | −133.0 | −130.0 |
| SF11 | −135.5 | −132.5 |
| SF12 | −137.0 | −134.0 |

**SX1301** (gateway — `gw_sensitivity_table` em `parametors.py`, ~6 dB melhor):

| SF | BW 125 kHz (dBm) | BW 250 kHz (dBm) |
|---|---|---|
| SF7 | −130.0 | −127.0 |
| SF8 | −132.5 | −129.5 |
| SF9 | −135.0 | −132.0 |
| SF10 | −137.5 | −134.5 |
| SF11 | −140.0 | −137.0 |
| SF12 | −142.5 | −139.0 |

A cobertura de uplink (`find_best_gateway()`) usa a tabela SX1301. A sensibilidade SX1272 é usada como fallback quando o par (SF, BW) não consta na tabela SX1301.

---

## 7. Configuração do Cenário de Referência

```python
# parametors.py
model_pathloss = "log_normal_shadowing"   # γ=3.76, σ=3.57 dB
noise_figure   = 6                        # dB — SX1272
temperatura    = 294.15                   # K
k              = 1.38e-23                 # J/K
gw_antenna_gain = 3                       # dBi
ed_antenna_gain = 0                       # dBi
indoor_ratio    = 0.3                     # 30% dos devices indoor

# network.py
Network(...,
    model_pathloss      = "log_normal_shadowing",
    correlated_shadowing = False,   # shadowing independente por amostra
    indoor_ratio        = 0.3,
)
```

Noise floor resultante: **−116.95 dBm** @ 125 kHz — referência para cálculo de SNR e SINR em todos os pacotes da simulação.
