# Simulation Parameters and Experimental Configuration — PyLoRaWAN

## Visão Geral

Os experimentos de avaliação de desempenho são conduzidos em um cenário de referência reproduzível, configurado em `parametors.py`. Os parâmetros seguem as convenções adotadas por estudos comparativos de referência (FLoRa, ns-3, LoRaWANSim) para permitir comparação direta com resultados publicados. A semente aleatória fixa (`random_seed = 42`) garante reproducibilidade entre execuções.

---

## 1. Cenário de Referência

| Parâmetro | Valor | Justificativa |
|---|---|---|
| End Devices | 50 | Carga moderada, comparável ao cenário de referência FLoRa |
| Gateways | 1 | Topologia single-GW — isola o comportamento do protocolo |
| Área | 10 km × 10 km | Compatível com alcance LoRa em área urbana/suburbana |
| Duração | 3600 s (1 h) | Cenário de referência FLoRa/ns-3 |
| Semente aleatória | 42 | Reproducibilidade e comparação entre configurações |

---

## 2. Parâmetros de Rádio (PHY)

| Parâmetro | Valor | Descrição |
|---|---|---|
| SFs disponíveis | [7, 8, 9, 10, 11, 12] | SF atribuído aleatoriamente; ajustado pelo ADR |
| TX Power inicial | 14 dBm | Máximo EU868; reduzido pelo ADR se margem disponível |
| Canais EU868 | 868.1, 868.3, 868.5 MHz | 3 canais default; seleção aleatória por transmissão |
| Largura de banda | 125 kHz | BW padrão LoRaWAN EU868 |
| Coding Rate | 4/5 (CR=1) | Overhead mínimo; compatível com FLoRa/ns-3 |
| Payload | 20 bytes | Compatível com cenário de referência FLoRa/ns-3 |
| Altura device (ht_m) | 1.5 m | Nível do chão |
| Altura gateway (hr_m) | 30 m | Torre / telhado |
| Ganho antena GW | 3 dBi | Antena omnidirecional típica |
| Ganho antena ED | 0 dBi | Antena chip |

---

## 3. Modelo de Propagação

O modelo padrão é `log_normal_shadowing`, calibrado com parâmetros compatíveis com FLoRa e ns-3:

```python
model_pathloss = "log_normal_shadowing"

# Parâmetros do modelo (network.py — pathloss())
d0     = 1.0 m          # distância de referência
PL_d0  = 7.7 dB         # perda na distância de referência
γ      = 3.76           # expoente de path loss
σ      = 3.57 dB        # desvio padrão do shadowing

PL(d) = 7.7 + 10 × 3.76 × log₁₀(d / 1.0) + X_σ
X_σ ~ N(0, 3.57)
```

**Modelos alternativos disponíveis:**

| `model_pathloss` | Parâmetros principais | Referência |
|---|---|---|
| `log_normal_shadowing` | d₀=1m, PL₀=7.7, γ=3.76, σ=3.57 | FLoRa/ns-3 (padrão) |
| `oulu` | d₀=40m, PL₀=127.41, γ=2.08, σ=3.57 | LoRaSim (Bor et al. 2016) |
| `log_distance` | γ=3.5 | Modelo simplificado |
| `fspl` | — | Free-Space Path Loss |
| `okumura_hata` | ht=1.5m, hm=30m | Urbano/suburbano |
| `cost_hata` | COST 231 extension | Urbano |
| `correlated_shadowing` | grid_step=50m, σ=3.57 | ns-3 CorrelatedShadowing |
| `building_penetration` | Lognormal(ln(20), 0.5) | ITU-R P.2109 |
| `fading` | Rayleigh/Rician/Nakagami | Multpath fading |

### 3.1 Shadowing Espacialmente Correlacionado

Quando `correlated_shadowing=True`, `CorrelatedShadowingMap` é criado com `grid_step=50 m` e σ=3.57 dB. A interpolação bilinear garante continuidade espacial do campo de shadowing.

### 3.2 Perda de Penetração Predial

30% dos devices são marcados como `is_indoor` (`indoor_ratio=0.3`). Para eles, uma perda adicional é amostrada no construtor:

```python
device.is_indoor = (random.random() < indoor_ratio)
building_penetration = np.random.lognormal(mean=log(20), sigma=0.5)   # mJ
```

---

## 4. Ruído e Sensibilidade

| Parâmetro | Valor | Descrição |
|---|---|---|
| Constante de Boltzmann (k) | 1.38 × 10⁻²³ J/K | — |
| Temperatura de referência | 294.15 K (~21 °C) | — |
| Noise Figure | 6 dB | SX1272 típico |

**Noise floor** para BW = 125 kHz:
```
N = k × T × BW × NF_linear
  = 1.38e-23 × 294.15 × 125000 × 10^(6/10)
  ≈ −116.95 dBm
```

**Sensibilidade SX1301 (gateway):**

| SF | BW 125 kHz |
|---|---|
| SF7 | −130.0 dBm |
| SF8 | −132.5 dBm |
| SF9 | −135.0 dBm |
| SF10 | −137.5 dBm |
| SF11 | −140.0 dBm |
| SF12 | −142.5 dBm |

---

## 5. Parâmetros MAC e Protocolo

| Parâmetro | Valor | Descrição |
|---|---|---|
| Região | EU868 | Sub-bandas, canais default, RX2 @ 869.525 MHz |
| RX1 delay | 1 s | Abertura da janela RX1 após TX_END |
| RX2 delay | 2 s | Abertura da janela RX2 após TX_END |
| Duty cycle ED | 1% | Sub-banda G/G1 (863–868.6 MHz) |
| Duty cycle GW RX1 | 1% | Downlink janela RX1 |
| Duty cycle GW RX2 | 10% | Downlink janela RX2 (sub-banda G3, 869.525 MHz) |
| Confirmed ratio | 30% | Fração de uplinks confirmados por device |
| Max retransmissões | 10 | Por pacote confirmado sem ACK |
| `lambda_rate` | 0.1 Hz | Taxa de transmissão; inter-arrival médio = 10 s |

---

## 6. Adaptive Data Rate

| Parâmetro | Valor | Descrição |
|---|---|---|
| ADR habilitado | `True` | Server-side ADR ativo por default |
| Método de agregação | `"average"` | Média aritmética do histórico SNR |
| Tamanho do histórico | 20 pacotes | Mínimo de amostras antes de agir |
| Margem de segurança | 10 dB | Sobre o SNR mínimo do SF atual |
| Passo de ajuste | 3 dB | Por passo de redução/aumento |
| ADR_ACK_LIMIT | 64 frames | Uplinks sem DL → solicita backoff |
| ADR_ACK_DELAY | 32 frames | Frames adicionais antes do backoff |
| Interferência | `"semtech"` | Matriz Semtech AN1200.18 (default) |

---

## 7. Modelo de Energia

| Parâmetro | Valor | Referência |
|---|---|---|
| Tensão de alimentação | 3.3 V | — |
| Corrente SLEEP | 0.0015 mA | SX1272 datasheet |
| Corrente STANDBY | 1.4 mA | SX1272 datasheet |
| Corrente RX | 11.2 mA | SX1272 / ns-3 / FLoRa |
| Corrente TX (14 dBm) | 38.0 mA | RN2483 datasheet |
| Corrente TX (2 dBm) | 22.3 mA | RN2483 datasheet |

---

## 8. Mobilidade

| Parâmetro | Valor | Descrição |
|---|---|---|
| Mobilidade | `True` | Habilitada por default |
| Modelo | `random_walk` | Direção uniforme θ~U(0, 2π) |
| Velocidade | 1.5 m/s | Passo de 1.5 m por segundo |
| Intervalo de atualização | 1.0 s fixo | Handler `MOBILITY_UPDATE` |
| Área de confinamento | 10 000 m | Fronteira reflexiva (clamp) |

Deslocamento RMS esperado em 3600 s: `1.5 × √3600 = 90 m` — pequeno em relação à área de 10 km × 10 km.

---

## 9. Deployment

O deployment padrão é `"grid"` (posições uniformemente espaçadas na área). Alternativas disponíveis:

| `deployment_type` | Descrição |
|---|---|
| `grid` | Grade uniforme — padrão |
| `random_uniform` | Posições aleatórias uniformes |
| `circular` | Anel a `deployment_radius` do centro |
| `hexagonal` | Grade hexagonal |
| `annular` | Anel com raio interno e externo |
| `clustered` | Agrupamentos (clusters) com dispersão interna |
| `from_file` | Posições lidas de arquivo JSON/CSV externo |

---

## 10. Configuração Completa de Referência

```python
# parametors.py — Cenário de referência
num_devices        = 50
num_gateways       = 1
area_size          = 10000          # 10 km × 10 km
simulation_time    = 3600           # 1 hora
lambda_rate        = 0.1            # 10 s entre transmissões (média)
random_seed        = 42

# PHY
sf_range           = [7, 8, 9, 10, 11, 12]
tx_power           = 14             # dBm
frequency_mhz      = [868.1, 868.3, 868.5]
bw                 = [125000]       # Hz
cr                 = 1              # 4/5
pl                 = 20             # bytes
ht_m               = 1.5            # m
hr_m               = 30             # m

# Propagação
model_pathloss     = "log_normal_shadowing"
# d0=1m, PL0=7.7dB, γ=3.76, σ=3.57dB

# Ruído
noise_figure       = 6              # dB
temperatura        = 294.15         # K

# ADR
adr_enabled        = True
ADR_HISTORY_SIZE   = 20
ADR_SNR_MARGIN_DB  = 10
ADR_STEP_DB        = 3

# Energia
voltage            = 3.3            # V
```