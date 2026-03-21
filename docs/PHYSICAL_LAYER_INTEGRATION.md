# PyLoRaWAN — Physical Layer Integration

> Documentação técnica completa da camada física do simulador.
> Cobre channel model, interferência, propagação, LR-FHSS e toda a cadeia PHY de recepção.

---

## Sumário

1. [Visão Geral da Cadeia PHY](#1-visão-geral-da-cadeia-phy)
2. [Parâmetros PHY (parametors.py)](#2-parâmetros-phy-parametorspy)
3. [Cálculo de Airtime (ToA)](#3-cálculo-de-airtime-toa)
4. [Ruído Térmico e Noise Floor](#4-ruído-térmico-e-noise-floor)
5. [Tabelas de Sensibilidade](#5-tabelas-de-sensibilidade)
6. [Modelos de Propagação (9 modelos)](#6-modelos-de-propagação-9-modelos)
7. [Shadowing Correlacionado (G2)](#7-shadowing-correlacionado-g2)
8. [Penetração Predial (G3)](#8-penetração-predial-g3)
9. [Link Budget Completo](#9-link-budget-completo)
10. [ChannelModel — Rastreamento On-Air](#10-channelmodel--rastreamento-on-air)
11. [Interferência: SINR e Matrizes](#11-interferência-sinr-e-matrizes)
12. [Capture Effect e Modelo de Energia (G14)](#12-capture-effect-e-modelo-de-energia-g14)
13. [Preamble Locking (G13)](#13-preamble-locking-g13)
14. [DL-UL Interference (G8)](#14-dl-ul-interference-g8)
15. [Gateway Saturation — SX1301](#15-gateway-saturation--sx1301)
16. [LR-FHSS — Camada Física Alternativa](#16-lr-fhss--camada-física-alternativa)
17. [ACRDA — Successive Interference Cancellation](#17-acrda--successive-interference-cancellation)
18. [MRC — Maximal Ratio Combining (Sprint 9)](#18-mrc--maximal-ratio-combining-sprint-9)
19. [Fluxo PHY Completo por Transmissão](#19-fluxo-phy-completo-por-transmissão)
20. [Comparação com Referências (ns-3, FLoRa)](#20-comparação-com-referências-ns-3-flora)

---

## 1. Visão Geral da Cadeia PHY

```
EndDevice
│
│  1. Seleciona (SF, BW, freq, tx_power) e calcula ToA
│  2. Aplica duty cycle
│  3. Cria Packet
│
▼
Gateway.process_uplink(packet)
│
│  4. Verifica DL-UL interference (G8)
│  5. Aloca ReceptionPath (SX1301 — 8 paths)
│  6. Calcula distance + path_loss (modelo selecionado)
│  7. Aplica building penetration (G3)
│  8. Calcula RSSI = tx_power + gains - path_loss
│  9. Acumula interferência on-air por SF (G14 energy-based)
│  10. Calcula SINR = S / (I + N)
│
▼
ChannelModel.evaluate_reception(packet)
│
│  11. Verifica SNR >= SNR_min[SF] (sinal vs. ruído térmico)
│  12. Preamble locking (G13) — exclui interferentes tardios
│  13. Seleciona matriz (Semtech ou Goursaud)
│  14. Para cada interferente: energy_ratio_db >= threshold?
│       → SUCESSO ou COLISÃO
│
▼
Resultado: packet.collided = True/False
           packet.rssi, .snr, .sinr populados
```

---

## 2. Parâmetros PHY (parametors.py)

### LoRa PHY Básico

| Parâmetro          | Valor                       | Descrição                            |
|--------------------|-----------------------------|--------------------------------------|
| `sf_range`         | `[7, 8, 9, 10, 11, 12]`    | Spreading Factors disponíveis        |
| `tx_power`         | `14 dBm`                    | Potência TX inicial (máx EU868)      |
| `frequency_mhz`    | `[868.1, 868.3, 868.5]`     | Canais EU868 padrão                  |
| `bw`               | `[125000]` Hz               | Bandwidths disponíveis               |
| `cr`               | `1` (= 4/5)                 | Coding Rate                          |
| `pl`               | `20` bytes                  | Payload útil                         |
| `ht_m`             | `1.5 m`                     | Altura do end device                 |
| `hr_m`             | `30 m`                      | Altura do gateway (torre/telhado)    |

### Ganhos de Antena

| Componente          | Ganho   | Tipo                    |
|---------------------|---------|-------------------------|
| `ed_antenna_gain`   | 0 dBi   | Chip antenna (ED)       |
| `gw_antenna_gain`   | 3 dBi   | Omnidirecional (GW)     |

### Correntes TX por Potência (RN2483 datasheet)

| TX Power | Corrente |
|----------|----------|
| 14 dBm   | 38.0 mA  |
| 12 dBm   | 35.1 mA  |
| 10 dBm   | 32.4 mA  |
| 8 dBm    | 30.0 mA  |
| 6 dBm    | 27.5 mA  |
| 4 dBm    | 24.7 mA  |
| 2 dBm    | 22.3 mA  |

### Correntes de Recepção (SX1272 datasheet)

| Estado   | Corrente    |
|----------|------------|
| RX       | 11.2 mA    |
| Standby  | 1.4 mA     |
| Sleep    | 0.0015 mA  |

---

## 3. Cálculo de Airtime (ToA)

**Arquivo:** `enddevice.py` — método `calculate_airtime()`

Implementa a fórmula oficial do datasheet SX1272/SX1276 (Semtech AN1200.13):

### Fórmula Completa

```
Tsym = 2^SF / BW                         [segundos por símbolo]

Tpream = (8 + 4.25) * Tsym               [duração do preâmbulo]

DE = 1  se  Tsym > 16ms  (low data rate optimization)
DE = 0  caso contrário

H   = 0  (header presente)
CRC = 1  (CRC habilitado)

numerator   = 8 * PL - 4 * SF + 28 + 16 * CRC - 20 * H
denominator = 4 * (SF - 2 * DE)

payloadSymbNB = 8 + max(⌈ numerator / denominator ⌉ × (CR + 4), 0)

ToA = Tpream + payloadSymbNB * Tsym
```

### ToA por SF (PL=20B, BW=125kHz, CR=4/5, CRC=1)

| SF  | Tsym (ms) | DE | ToA (ms)  | Throughput efetivo |
|-----|-----------|----|-----------|--------------------|
| SF7 | 1.024     | 0  | ~36.1 ms  | ~4.4 kbps          |
| SF8 | 2.048     | 0  | ~72.2 ms  | ~2.2 kbps          |
| SF9 | 4.096     | 0  | ~144.4 ms | ~1.1 kbps          |
| SF10| 8.192     | 0  | ~288.8 ms | ~554 bps           |
| SF11| 16.384    | 1  | ~577.5 ms | ~277 bps           |
| SF12| 32.768    | 1  | ~1154.9 ms| ~138 bps           |

> **DE=1** para SF11 e SF12 (BW=125kHz) porque Tsym > 16ms.

---

## 4. Ruído Térmico e Noise Floor

**Arquivo:** `network.py` — método `calculate_noise_floor(bw)`

```python
noise_floor_dBm = 10 * log10(k * T * BW) + 30 + NF

onde:
  k  = 1.38 × 10⁻²³ J/K   (constante de Boltzmann)
  T  = 294.15 K             (temperatura de referência, ~21°C)
  BW = largura de banda (Hz)
  NF = 6 dB                 (noise figure SX1272/SX1301, compatível com ns-3)
  +30 = conversão W → mW → dBm
```

### Valores por Bandwidth

| BW      | Noise Floor  |
|---------|-------------|
| 125 kHz | ≈ −120.0 dBm |
| 250 kHz | ≈ −117.0 dBm |
| 500 kHz | ≈ −114.0 dBm |

### SNR Mínimo por SF (LoRa Design Guide / ns-3 / FLoRa)

| SF  | SNR mínimo |
|-----|------------|
| SF7 | −7.5 dB    |
| SF8 | −10.0 dB   |
| SF9 | −12.5 dB   |
| SF10| −15.0 dB   |
| SF11| −17.5 dB   |
| SF12| −20.0 dB   |

> Limiar inicial de recepção: `RSSI - noise_floor >= SNR_min[SF]`

---

## 5. Tabelas de Sensibilidade

### End Device — SX1272

| SF  | BW 125kHz | BW 250kHz | BW 500kHz |
|-----|----------|----------|----------|
| SF7 | −124.0 dBm | −121.0 dBm | −118.0 dBm |
| SF8 | −127.0 dBm | −124.0 dBm | −121.0 dBm |
| SF9 | −130.0 dBm | −127.0 dBm | −124.0 dBm |
| SF10| −133.0 dBm | −130.0 dBm | −127.0 dBm |
| SF11| −135.5 dBm | −132.5 dBm | −129.5 dBm |
| SF12| −137.0 dBm | −134.0 dBm | −131.0 dBm |

### Gateway — SX1301 (~6 dB melhor que SX1272)

Referência: `ns-3 gateway-lora-phy.cc` (G10 Sprint 2)

| SF  | BW 125kHz | BW 250kHz | BW 500kHz |
|-----|----------|----------|----------|
| SF7 | −130.0 dBm | −127.0 dBm | −124.0 dBm |
| SF8 | −132.5 dBm | −129.5 dBm | −126.5 dBm |
| SF9 | −135.0 dBm | −132.0 dBm | −129.0 dBm |
| SF10| −137.5 dBm | −134.5 dBm | −131.5 dBm |
| SF11| −140.0 dBm | −137.0 dBm | −134.0 dBm |
| SF12| −142.5 dBm | −139.0 dBm | −136.0 dBm |

> Diferença de 6 dB consistente com a vantagem do SX1301 multichannel demodulator.

---

## 6. Modelos de Propagação (9 modelos)

**Arquivo:** `network.py` — método `pathloss(distance, frequency_mhz, model_pathloss, ...)`

### Seleção do Modelo

```python
model_pathloss = "log_normal_shadowing"  # padrão em parametors.py
```

Opções válidas: `okumura_hata`, `log_distance`, `fspl`, `cost_hata`,
`log_normal_shadowing`, `correlated_shadowing`, `building_penetration`,
`fading`, `oulu`

---

### Modelo 1: `log_normal_shadowing` (padrão — FLoRa / ns-3)

```
PL(d) = PL_d0 + 10 * γ * log10(d / d0) + X_σ

PL_d0 = 7.7 dB   (perda a d0=1m)
γ     = 3.76      (expoente de propagação urbano)
d0    = 1.0 m
σ     = 3.57 dB   (desvio padrão do shadowing)
X_σ   ~ N(0, σ)   (nova amostra a cada transmissão — independente)
```

Idêntico ao modelo padrão do FLoRa e do ns-3 LoRaWAN (Aerts et al. 2017, Magrin et al. 2017).

---

### Modelo 2: `correlated_shadowing` (G2 Sprint 2)

```
PL(d, x, y) = PL_d0 + 10 * γ * log10(d / d0) + S(x, y)

S(x, y) = interpolação bilinear do CorrelatedShadowingMap
```

O mapa usa um grid 50m × 50m de amostras gaussianas `N(0, 3.57 dB)`.
Posições intermediárias são interpoladas — introduz correlação espacial
equivalente ao `ns-3 CorrelatedShadowingPropagationLossModel` (distância de correlação ~40 m).

Ver [Seção 7](#7-shadowing-correlacionado-g2) para detalhes da implementação.

---

### Modelo 3: `okumura_hata` (macro urbano)

```
CF = (1.1 * log10(f) - 0.7) * hr - (1.56 * log10(f) - 0.8)

PL = 69.55 + 26.16 * log10(f_MHz)
         - 13.82 * log10(ht_m)
         - CF
         + (44.9 - 6.55 * log10(ht_m)) * log10(d_km)

ht_m = 1.5 m (ED)
hr_m = 30 m  (GW)
```

Válido para: 150–1500 MHz, 1–20 km, ambiente urbano.

---

### Modelo 4: `fspl` (Free-Space Path Loss)

```
PL = 20 * log10(d_km) + 20 * log10(f_MHz) + 32.45
```

Modelo de espaço livre. Subestima perda em ambientes reais — usado apenas para comparação.

---

### Modelo 5: `log_distance`

```
PL = 20 * log10(f_MHz) + 10 * γ * log10(d_km) - 28

γ = 3.5  (expoente)
```

Versão sem shadowing estocástico — totalmente determinístico para dada distância.

---

### Modelo 6: `cost_hata` (suburbano/rural)

```
CF = (1.1 * log10(f) - 0.7) * hr - (1.56 * log10(f) - 0.8)

PL = 46.3 + 33.9 * log10(f_MHz)
         - 13.82 * log10(ht_m)
         - CF
         + (44.9 - 6.55 * log10(ht_m)) * log10(d_km)
```

Variante COST-231 Hata para faixas acima de 1500 MHz. Usada como base para o modelo `fading`.

---

### Modelo 7: `building_penetration`

```
PL = log_normal_shadowing(d, f) + BPL

BPL ~ LogNormal(μ=ln(20), σ=0.5)  dB   (se device.is_indoor)
BPL = 0                                  (se device.is_outdoor)
```

Perda de penetração predial aplicada como adicional ao modelo log-normal.
O valor `BPL` é cacheado por dispositivo (`_building_penetration_cache`) — constante para a simulação.

Ver [Seção 8](#8-penetração-predial-g3) para detalhes.

---

### Modelo 8: `fading`

```
PL = cost_hata(d, f) - fading_dB

Rayleigh:  fading_dB = 20 * log10(sample),  sample ~ Rayleigh(scale=1)
Rician:    fading_dB = 20 * log10(sample),  sample ~ √(χ²(2, 2K) / (2(1+K))),  K=3
Nakagami:  fading_dB = 20 * log10(sample),  sample ~ √(Gamma(m, 1/m)),  m=3
```

> Fading **reduz** a perda (pode melhorar recepção) — representa o efeito de multipath constructivo.

---

### Modelo 9: `oulu` (LoRaSim / FLoRa — Bor et al. 2016)

```
PL(d) = PL_d0 + 10 * γ * log10(d / d0) + X_σ

PL_d0 = 127.41 dB   (perda a d0=40m)
γ     = 2.08         (expoente — ambiente semi-aberto)
d0    = 40.0 m
σ     = 3.57 dB
X_σ   ~ N(0, σ)
```

Compatível com o modelo original do simulador LoRaSim (Bor et al. 2016) e FLoRa.

---

### Comparação de Perda por Distância (868 MHz, ht=1.5m, hr=30m)

| Modelo             | 100m    | 500m    | 2000m   | 5000m   |
|--------------------|---------|---------|---------|---------|
| fspl               | 71 dB   | 85 dB   | 97 dB   | 106 dB  |
| log_normal (γ=3.76)| ~87 dB  | ~108 dB | ~121 dB | ~130 dB |
| oulu (γ=2.08)      | ~128 dB | ~134 dB | ~141 dB | ~146 dB |
| okumura_hata       | ~100 dB | ~117 dB | ~130 dB | ~140 dB |

> Valores aproximados sem shadowing estocástico.

---

## 7. Shadowing Correlacionado (G2)

**Arquivo:** `network.py` — classe `CorrelatedShadowingMap`

### Motivação

O shadowing log-normal padrão gera amostras independentes por transmissão.
Na realidade, dispositivos próximos sofrem shadowing similar (obstáculos comuns).
O modelo correlacionado captura esta dependência espacial.

### Implementação

```python
class CorrelatedShadowingMap:
    grid_step  = 50 m       # resolução do grid
    grid_points = area/50+1 # pontos por dimensão (10km → 201×201)
    grid[i,j]  ~ N(0, 3.57 dB)   # amostra independente por nó do grid
```

### Interpolação Bilinear

Para dispositivo em posição `(x, y)`:

```
gx = x / grid_step              # posição fracionária no grid
gy = y / grid_step

x0, y0 = ⌊gx⌋, ⌊gy⌋            # nó inferior-esquerdo
x1, y1 = x0+1, y0+1             # nó superior-direito
dx = gx - x0                    # peso fracionário x
dy = gy - y0                    # peso fracionário y

S(x,y) = (1-dx)(1-dy)·v[x0,y0]
        + dx(1-dy)·v[x1,y0]
        + (1-dx)dy·v[x0,y1]
        + dx·dy·v[x1,y1]
```

### Propriedades

| Propriedade         | Valor              |
|---------------------|--------------------|
| Distribuição        | N(0, 3.57 dB)      |
| Distância de correlação | ~40–50 m (grid_step) |
| Dispositivos a > 100m | shadowing praticamente independente |
| Equivalência        | ns-3 CorrelatedShadowingPropagationLossModel |

---

## 8. Penetração Predial (G3)

**Arquivo:** `network.py` — `get_building_penetration(device)`

### Configuração

```python
indoor_ratio = 0.3   # 30% dos devices são indoor (padrão)
# Definido em initialize_devices():
device.is_indoor = (random.random() < indoor_ratio)
```

### Cálculo da Perda

```python
# Na inicialização (Sprint 2) — calculado uma vez e cacheado
BPL ~ LogNormal(μ=ln(20), σ=0.5) dB

# Valor típico: ~20 dB (mediana), com variação lognormal
# Cache: _building_penetration_cache[device_id] = BPL
```

### Aplicação no Link Budget

```python
path_loss = pathloss(distance, freq, model, x, y)
path_loss += get_building_penetration(device)   # +0 dB outdoor, +~20 dB indoor
RSSI = tx_power + ed_antenna_gain + gw_antenna_gain - path_loss
```

---

## 9. Link Budget Completo

Calculado em `gateway.process_uplink()`:

```
RSSI [dBm] = tx_power
           + ed_antenna_gain        (+0 dBi — chip antenna)
           + gw_antenna_gain        (+3 dBi — omnidirecional)
           - path_loss              (modelo selecionado, inclui shadowing)
           - building_penetration   (+0 outdoor / +~20 dB indoor)

Margem de link = RSSI - gw_sensitivity[SF, BW]

Ex. (SF9, BW=125kHz, d=2km, modelo log_normal):
  path_loss ≈ 121 dB
  RSSI = 14 + 0 + 3 - 121 = −104 dBm
  gw_sensitivity[SF9] = −135 dBm
  Margem = −104 − (−135) = +31 dB  ✓ bem acima do limiar
```

### Distância Máxima por SF (cenário sem shadowing)

Com `log_normal_shadowing` (σ=3.57 dB), margem ~10 dB, ht=1.5m, hr=30m:

| SF  | GW Sensitivity | TX Power | Margem alvo | d_max estimado |
|-----|---------------|----------|-------------|----------------|
| SF7 | −130.0 dBm    | 14 dBm   | 10 dB       | ~2.5 km        |
| SF9 | −135.0 dBm    | 14 dBm   | 10 dB       | ~4.0 km        |
| SF12| −142.5 dBm    | 14 dBm   | 10 dB       | ~7.5 km        |

---

## 10. ChannelModel — Rastreamento On-Air

**Arquivo:** `channel.py`

Gerencia todas as transmissões ativas e avalia colisões.

### Estrutura de Dados

```python
on_air: list[tuple[Packet, tx_start: float, tx_end: float]]
```

### Métodos

| Método                              | Descrição                                         |
|-------------------------------------|---------------------------------------------------|
| `add_transmission(pkt, start, end)` | Registra pacote como on-air                      |
| `evaluate_reception(pkt, gw)`       | Avalia colisão via SINR + capture effect         |
| `cleanup_expired(current_time)`     | Remove `tx_end <= current_time` da lista        |
| `get_active_on_frequency(freq, t)`  | Filtra on-air por frequência e tempo             |
| `get_on_air_count()`                | Total de transmissões ativas                     |
| `stats()`                           | `{total_collisions, total_receptions, on_air_count}` |

### Ciclo de Vida no On-Air

```
_on_device_send:     channel.add_transmission(packet, tx_start, tx_end)
                     → pacote entra no on_air

_on_tx_end:          channel.evaluate_reception(packet, gateway)
                     → colisão avaliada com todos os outros on-air
                     channel.cleanup_expired(current_time)
                     → pacotes expirados removidos
```

---

## 11. Interferência: SINR e Matrizes

### Cálculo de SINR (no Gateway)

```
Signal_linear    = 10^(RSSI_signal / 10)           [mW]
Noise_linear     = 10^(noise_floor / 10)            [mW]

Para cada interferente com sobreposição em mesma frequência:
  overlap_ratio = overlap_duration / packet.rectime
  I_i_linear   = 10^(RSSI_interferer_i / 10) × overlap_ratio

Interference_linear = Σ I_i_linear

SINR_linear = Signal / (Interference + Noise)
SINR_dB     = 10 × log10(SINR_linear)
```

> `overlap_ratio` pondera a interferência proporcionalmente ao tempo de sobreposição — implementação do modelo de energia acumulada do ns-3.

### Matriz de Interferência Semtech AN1200.18

Índice: `SF12 → idx 0, SF7 → idx 5`

```
          SF12  SF11  SF10   SF9   SF8   SF7     ← SF interferente
SF12  [ +1,   -23,  -24,  -25,  -25,  -25]
SF11  [-20,    +1,  -20,  -21,  -22,  -22]
SF10  [-18,   -17,   +1,  -17,  -18,  -19]
SF9   [-15,   -14,  -13,   +1,  -13,  -15]
SF8   [-13,   -13,  -12,  -11,   +1,  -11]
SF7   [ -9,    -9,   -9,   -9,   -8,   +1]
↑ SF alvo
```

- **Diagonal (co-SF)**: threshold = **+1 dB** — sinal precisa ser apenas 1 dB mais forte
- **Off-diagonal (cross-SF)**: thresholds muito negativos — SFs ortogonais são tolerantes a interferências cruzadas

### Matriz de Interferência Goursaud (ns-3 padrão)

```
          SF12  SF11  SF10   SF9   SF8   SF7
SF12  [ +6,   -36,  -36,  -36,  -36,  -36]
SF11  [-29,    +6,  -33,  -33,  -33,  -33]
SF10  [-28,   -26,   +6,  -30,  -30,  -30]
SF9   [-27,   -25,  -23,   +6,  -27,  -27]
SF8   [-24,   -22,  -20,  -19,   +6,  -24]
SF7   [-20,   -16,  -18,  -19,  -16,   +6]
```

- **Co-SF**: threshold = **+6 dB** — requer sinal dominante (mais conservador)
- **Cross-SF**: thresholds ainda mais negativos (mais permissivo com SFs cruzados)

### Comparação dos Modelos

| Aspecto             | Semtech AN1200.18       | Goursaud (ns-3)         |
|---------------------|-------------------------|-------------------------|
| Co-SF threshold     | +1 dB                   | +6 dB                   |
| Colisões co-SF      | Menos colisões          | Mais colisões           |
| Cross-SF            | Thresholds maiores (−9 a −25) | Thresholds menores (−16 a −36) |
| PDR resultante      | Mais alto (~92%)        | Mais baixo (~88%)       |
| Referência          | Semtech AN1200.18       | Goursaud et al. 2016    |
| Configuração        | `interference_model = "semtech"` | `= "goursaud"` |

---

## 12. Capture Effect e Modelo de Energia (G14)

**Arquivo:** `channel.py` — `evaluate_reception()`

Sprint 6 implementou o modelo de energia acumulada do ns-3 `LoraInterferenceHelper`, substituindo a simples comparação de RSSI.

### Modelo de Energia Acumulada

A razão de energia entre sinal e interferente leva em conta **quanto tempo** cada um se sobrepõe ao pacote alvo:

```
E_signal     = P_signal × T_packet       [energia total do sinal]
E_interferer = P_interferer × T_overlap  [energia do interferente durante sobreposição]

Razão de energia:
  E_signal / E_interferer = (P_signal / P_interferer) × (T_packet / T_overlap)

Em dB:
  energy_ratio_dB = (RSSI_signal - RSSI_interferer)
                  + 10 × log10(T_packet / T_overlap)
```

### Decisão de Captura

```python
threshold_dB = matrix[sf_target_idx][sf_interferer_idx]

if energy_ratio_dB >= threshold_dB:
    → sinal "captura" o canal → SUCESSO
else:
    → interferência domina → COLISÃO
    break  # um interferente que colide é suficiente
```

### Efeito da Correção de Energia

| Cenário                       | Correto sem correção? | Com correção G14 |
|-------------------------------|----------------------|-----------------|
| Interferente curto e tardio   | ❌ Superestima impacto | ✓ overlap_ratio < 1 → mais favorável |
| Interferente longo (co-durando) | ✓ Funciona bem      | ✓ overlap_ratio ≈ 1 → similar |
| Interferente que começa antes | ✓ Funciona bem       | ✓ considera energia total |

### Exemplo Numérico

```
packet.rectime = 144 ms (SF9)
interferer_overlap = 50 ms
RSSI_signal = -105 dBm
RSSI_interferer = -110 dBm  (5 dB mais fraco)

energy_ratio_dB = (−105 − (−110)) + 10*log10(144/50)
               = 5 + 10*log10(2.88)
               = 5 + 4.6
               = 9.6 dB

threshold (co-SF SF9 Semtech) = +1 dB

9.6 dB >= 1 dB → SUCESSO (sinal captura mesmo com interferência)
```

---

## 13. Preamble Locking (G13)

**Arquivo:** `channel.py` — `evaluate_reception()`

Sprint 2 implementou o modelo de travamento de preâmbulo do ns-3.

### Conceito

O receptor LoRa precisa sincronizar no preâmbulo (primeiros símbolos) para demodular o pacote. Interferentes que chegam **depois que o receptor já travou** no preâmbulo não podem desestabilizar a sincronização.

### Cálculo

```python
Tsym = 2^SF / BW                         # duração de 1 símbolo

preamble_lock_time = tx_start + 6 * Tsym # receptor trava após 6 símbolos
```

### Aplicação

```python
for (other_pkt, other_start, other_end) in on_air:
    # Interferente chegou APÓS o lock → não conta
    if other_start >= preamble_lock_time:
        continue
    # Interferente chegou ANTES do lock → avaliado
    interferers.append(...)
```

### Valores de Preamble Lock Time por SF

| SF  | Tsym    | 6 × Tsym (lock) |
|-----|---------|-----------------|
| SF7 | 1.0 ms  | 6.1 ms          |
| SF9 | 4.1 ms  | 24.6 ms         |
| SF12| 32.8 ms | 196.6 ms        |

> SF12 tem janela de vulnerabilidade ao preâmbulo muito maior (~197 ms).

---

## 14. DL-UL Interference (G8)

**Arquivo:** `gateway.py` — `process_uplink()` e `dl_busy_until`

Sprint 4 implementou o bloqueio de recepção durante transmissão de downlink.

### Mecanismo

Quando o gateway transmite um downlink, fica half-duplex na frequência usada:

```python
# Ao enviar downlink (em _on_rx1_open/_on_rx2_open):
gateway.dl_busy_until[freq] = current_time + dl_airtime

# No início de process_uplink():
if dl_busy_until.get(packet.freq, 0) > current_time:
    packet.collided = True
    return   # UL ignorado — GW ocupado transmitindo DL
```

### Impacto

- **Frequência bloqueada**: apenas a frequência do DL ativo
- **Duração**: airtime do downlink (função do SF do DL)
- **Efeito no PDR**: pequeno (~0.5–1%), mas fisicamente correto

---

## 15. Gateway Saturation — SX1301

**Arquivo:** `gateway.py` — `try_allocate_path()`

### Capacidade do SX1301

O chip SX1301 possui **8 demoduladores independentes** (reception paths). Permite recepção simultânea de até 8 pacotes em qualquer combinação de SF/canal.

```python
reception_paths = [ReceptionPath() for _ in range(8)]
```

### Lógica de Saturação

```python
def try_allocate_path(packet, current_time):
    tx_end = current_time + packet.rectime
    for path in reception_paths:
        if path.is_free(current_time):
            path.assign(packet, tx_end)
            return True           # path alocado
    saturation_events += 1
    return False                  # todos os 8 paths ocupados → descartado
```

### Probabilidade de Saturação

Com 50 EDs, λ=1/300s, ToA médio ~300ms (SF9):
- Taxa de chegada ao GW: ~50/300 = 0.17 pacotes/s
- Janela de risco: ~300ms → ~0.05 colisões de path por chegada
- Na prática: saturação <0.1% das vezes no cenário de referência

---

## 16. LR-FHSS — Camada Física Alternativa

**Arquivo:** `lrfhss.py`

Diferencial único do PyLoRaWAN: nenhum dos 4 simuladores comparados combina LoRa CSS + LR-FHSS.

### Conceito LR-FHSS

- Pacote fragmentado em múltiplos sub-pacotes
- Cada fragmento transmitido em canal aleatório diferente (frequency hopping)
- Receptor reconstrói com decodificação parcial (FEC — Forward Error Correction)
- Resiste a colisões parciais: basta que fragmentos suficientes sobrevivam

### LRFHSSFragment

```python
@dataclass
class LRFHSSFragment:
    frag_type: str    # 'header' ou 'payload'
    channel: int      # canal de hopping (0 a obw-1)
    duration: float   # duração em segundos
    packet_id: UUID   # UUID do pacote pai
    collided: bool    # colisão detectada
    tx_start: float   # timestamp início
    tx_end: float     # timestamp fim
```

### LRFHSS_PHY — Parâmetros

| Parâmetro           | Valor       | Fonte                  |
|---------------------|-------------|------------------------|
| `header_duration`   | 233.472 ms  | Especificação Semtech  |
| `payload_duration`  | 102.4 ms    | Especificação Semtech  |
| `transceiver_wait`  | 6.472 ms    | Overhead entre fragmentos |
| `obw` (OBW canais)  | 35          | Occupied Bandwidth padrão |
| `num_headers`       | 3           | Headers redundantes    |

### Code Rates

| Code Rate | bytes/fragmento | Divisor threshold |
|-----------|----------------|-------------------|
| `1/3`     | 2              | ÷3                |
| `2/3`     | 4              | ÷3                |
| `1/2`     | 3              | ÷2                |

### Fragmentação — `fragment_packet(payload_size)`

```python
n_payloads = ceil((payload_size + 3) / bytes_per_frag)
threshold  = ceil(n_payloads / threshold_div)
return (num_headers=3, n_payloads, threshold)
```

Exemplo (PL=20B, CR=1/3):
```
bytes_per_frag = 2
n_payloads = ceil(23 / 2) = 12
threshold  = ceil(12 / 3) = 4   # mínimo 4 payloads para decodificar

Total de fragmentos = 3 headers + 12 payloads = 15
```

### Cálculo de ToA LR-FHSS

```python
ToA = n_headers × header_duration
    + n_payloads × payload_duration
    + (n_headers + n_payloads) × transceiver_wait

= 3 × 0.233 + 12 × 0.102 + 15 × 0.006
≈ 0.700 + 1.229 + 0.097
≈ 2.03 s  (para PL=20B, CR=1/3)
```

> Comparação: SF9 CSS ≈ 144ms. LR-FHSS é ~14× mais lento, mas usa 35 canais.

### Sequência de Hopping — `generate_hopping_sequence(n_fragments)`

```python
[random.randint(0, obw-1) for _ in range(n_fragments)]
# Ex: [7, 23, 1, 15, 31, 0, 8, 22, ...]
```

Cada fragmento salta para um canal aleatório independente do anterior.

### `create_fragments(packet_id, payload_size, tx_start)`

```python
fragments = []
t = tx_start

# Headers primeiro (3 headers)
for i in range(n_headers):
    frag = LRFHSSFragment('header', hopping[i], header_duration, packet_id)
    frag.tx_start = t
    frag.tx_end   = t + header_duration
    t += header_duration + transceiver_wait

# Payloads (n_payloads)
for i in range(n_payloads):
    frag = LRFHSSFragment('payload', hopping[n_headers + i], payload_duration, packet_id)
    frag.tx_start = t
    frag.tx_end   = t + payload_duration
    t += payload_duration + transceiver_wait
```

### LRFHSS_Channel — Colisão por Fragmento

```python
def check_fragment_collisions(fragments):
    for frag in fragments:
        for (other, other_start, other_end) in active_fragments:
            if other.packet_id == frag.packet_id: continue
            if other.channel != frag.channel: continue       # mesma frequência?
            if no_overlap(frag, other): continue              # sobreposição temporal?
            frag.collided = True; break
```

### Critério de Decodificação — `evaluate_packet(fragments, threshold)`

```python
h_success = sum(1 for f in fragments if f.frag_type=='header' and not f.collided)
p_success = sum(1 for f in fragments if f.frag_type=='payload' and not f.collided)

success = (h_success >= 1) AND (p_success >= threshold)
```

> Precisa de **pelo menos 1 header** E **pelo menos `threshold` payloads** sem colisão.

### Por que PDR ≈ 100% com LR-FHSS?

Com 35 canais OBW e 15 fragmentos por pacote:
- Probabilidade de qualquer fragmento colidir: `n_devices × 15 / 35 / λ_agg` → muito baixa
- Mesmo com algumas colisões parciais: threshold=4/12 → basta 33% dos payloads
- ACRDA/SIC recupera ainda mais (ver Seção 17)

---

## 17. ACRDA — Successive Interference Cancellation

**Arquivo:** `lrfhss.py` — classe `ACRDA`

### Conceito

ACRDA (Asynchronous Coded Random Access with Diversity Aloha) aplica cancelamento iterativo de interferência após a simulação:

1. Pacotes que já podem ser decodificados são identificados
2. Seus fragmentos são "removidos" do canal (interferência cancelada)
3. Outros pacotes, antes não decodificáveis, podem agora ser decodificados
4. Processo repete até convergência

### Atributos

| Atributo           | Descrição                                           |
|--------------------|-----------------------------------------------------|
| `window_size`      | Janela ACRDA (múltiplos do período de TX), padrão 3 |
| `decoded_packets`  | Set de UUIDs já decodificados                       |
| `packet_fragments` | `{packet_id → [LRFHSSFragment, ...]}`              |

### Algoritmo `process_window(channel)`

```python
while new_recovery:
    new_recovery = False

    for packet_id, fragments in packet_fragments.items():
        if packet_id in decoded_packets: continue

        h_ok = sum(f.frag_type=='header' and not f.collided)
        p_ok = sum(f.frag_type=='payload' and not f.collided)
        threshold = max(1, n_payloads // 3)

        if h_ok >= 1 and p_ok >= threshold:
            decoded_packets.add(packet_id)
            _cancel_interference(packet_id, channel)  # SIC
            new_recovery = True   # pode haver mais pacotes recuperáveis

return (n_decoded, n_iterations)
```

### `_cancel_interference(packet_id, channel)`

```python
for dec_frag in decoded_frags:
    for other_frag in outros_pacotes:
        if other_frag.collided == False: continue
        if other_frag.channel != dec_frag.channel: continue
        if not overlap(dec_frag, other_frag): continue
        other_frag.collided = False  # interferência removida!
```

### Efeito Prático

| Fase            | Pacotes decodificados |
|-----------------|-----------------------|
| Sem ACRDA       | ~85–90% (colisões de fragmento)   |
| Iteração 1 SIC  | ~95%+                 |
| Convergência    | ~100% (cenário ref.)  |

---

## 18. MRC — Maximal Ratio Combining (Sprint 9)

**Arquivo:** `channel.py` — `evaluate_reception()` + `packet.snr_mrc`

### Conceito

Com múltiplos gateways, o mesmo pacote pode ser recebido por mais de um GW. O MRC combina os sinais para melhorar o SNR efetivo:

```
SNR_MRC = Σ SNR_i_linear  →  SNR_MRC_dB = 10 * log10(Σ 10^(SNR_i/10))
```

### Uso no evaluate_reception

```python
# Sprint 9: usa SNR_MRC se disponível (melhor do que SNR de um único GW)
if packet.snr_mrc is not None:
    snr = packet.snr_mrc
else:
    snr = packet.rssi - noise_floor
```

O `snr_mrc` é calculado externamente (por função MRC no `network.py`) e populado no Packet antes da avaliação.

---

## 19. Fluxo PHY Completo por Transmissão

```
t=0: EndDevice seleciona SF=9, freq=868.3, BW=125kHz, tx_power=14dBm
     → calculate_airtime() = 144.4 ms
     → duty_cycle OK (dc_release_time < t)

t=0: Packet criado (UUID, rectime=0.144, tx_start=0, tx_end=0.144)
     gateway.process_uplink(packet):
       ① dl_busy_until[868.3] = 0 → OK, sem DL-UL interference
       ② try_allocate_path() → Path #3 alocado (path.tx_end = 0.144)
       ③ distance = sqrt(dx² + dy² + (1.5-30)²) = 3200 m
       ④ path_loss = log_normal_shadowing(3200, 868.3) = 118.7 dB
       ⑤ building_pen = 0 dB (outdoor)
       ⑥ RSSI = 14 + 0 + 3 - 118.7 = -101.7 dBm
       ⑦ on_air: outros 2 pacotes em 868.3 com overlap
          → interference_linear = 10^(-110/10)*0.8 + 10^(-115/10)*0.3 = 1.06e-11 mW
       ⑧ noise_floor = -120 dBm = 10^(-12) mW
       ⑨ SINR = 10^(-10.17) / (1.06e-11 + 1e-12) = 7.3 (linear)
          SINR = 10*log10(7.3) = 8.6 dB
       ⑩ packet.rssi=-101.7, snr=18.3, sinr=8.6 armazenados

t=0: channel.add_transmission(packet, 0, 0.144)
     → packet entra em on_air

t=0.144: _on_tx_end:
     channel.evaluate_reception(packet, gw):
       ① noise_floor = -120 dBm, SNR = 18.3 dB
          SNR_min[SF9] = -12.5 dB → OK ✓
       ② preamble_lock_time = 0 + 6 * (2^9/125000) = 0 + 24.6ms = 0.025s
       ③ matriz = interference_matrix (Semtech)
       ④ interferentes com tx_start < 0.025:
          - Pkt_B: SF9 (co-SF), RSSI=-110 dBm, overlap=80ms
            energy_ratio = (-101.7 - (-110)) + 10*log10(144/80)
                         = 8.3 + 2.56 = 10.86 dB
            threshold = matrix[3][3] = +1 dB   (SF9 vs SF9)
            10.86 >= 1 → SOBREVIVE ✓
          - Pkt_C: SF7 (cross-SF), RSSI=-112 dBm, overlap=30ms
            threshold = matrix[3][5] = -15 dB  (SF9 vs SF7)
            ratio = (-101.7 - (-112)) + 10*log10(144/30) = 10.3 + 6.8 = 17.1 dB
            17.1 >= -15 → SOBREVIVE ✓
       ⑤ packet.collided = False ✓

     gateway.release_paths(0.144) → Path #3 liberado
     channel.cleanup_expired(0.144) → on_air limpo

t=0.144 + 1s: _on_rx1_open
     → RX1 aberta, NS verifica DL pendente
```

---

## 20. Comparação com Referências (ns-3, FLoRa)

| Aspecto                      | PyLoRaWAN                    | ns-3 LoRaWAN              | FLoRa                    |
|------------------------------|------------------------------|---------------------------|--------------------------|
| **Modelo de propagação**     | 9 modelos (log_normal padrão)| log_normal_shadowing       | log_normal / oulu        |
| **Shadowing**                | Independente + correlacionado| Correlacionado (exp.)     | Independente             |
| **Interferência**            | Energy-based (G14)           | LoraInterferenceHelper    | Simples (RSSI threshold) |
| **Matrizes**                 | Semtech + Goursaud           | Goursaud (padrão)         | Semtech                  |
| **Preamble locking**         | ✓ (6 símbolos)               | ✓                         | ✗                        |
| **Co-SF threshold**          | +1 dB (Semtech) / +6 dB (Goursaud) | +6 dB               | +6 dB                    |
| **SX1301 paths**             | ✓ (8 paths)                  | ✓                         | ✓                        |
| **DL-UL interference**       | ✓ (por frequência)           | ✓                         | ✗                        |
| **Building penetration**     | ✓ (lognormal, 30% indoor)    | ✓                         | ✗                        |
| **LR-FHSS**                  | ✓ (único)                    | ✗                         | ✗                        |
| **ACRDA / SIC**              | ✓ (único)                    | ✗                         | ✗                        |
| **MRC multi-GW**             | ✓ (Sprint 9)                 | ✓                         | ✗                        |
| **Noise figure**             | 6 dB (SX1272)                | 6 dB                      | 6 dB                     |
| **SNR mínimo por SF**        | ✓ (−7.5 a −20 dB)           | ✓                         | ✓                        |

---

*Documentação gerada a partir do código-fonte — Sprint 7 concluído.*
