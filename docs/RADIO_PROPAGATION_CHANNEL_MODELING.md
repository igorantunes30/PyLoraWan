# Radio Propagation and Channel Modeling — PyLoRaWAN

## Visão Geral

O simulador modela o canal sem fio por meio de um framework de propagação modular que suporta múltiplos efeitos de propagação em larga e pequena escala. Esse design permite avaliar o desempenho do LoRaWAN em ambientes heterogêneos, incluindo áreas abertas, implantações suburbanas, regiões urbanas densas e cenários com obstruções.

---

## 1. Arquivos e Responsabilidades

| Arquivo | Responsabilidade |
|---|---|
| `network.py` | `pathloss()` — 9 modelos de path loss; `CorrelatedShadowingMap`; `calculate_noise_floor()`; `find_best_gateway()` |
| `channel.py` | `ChannelModel` — rastreamento on-air; `evaluate_reception()` — avaliação de SNR, preamble lock, capture effect |
| `parametors.py` | `snr_min_per_sf`, `sensitivity_table`, `gw_sensitivity_table`, `gw_antenna_gain`, `ed_antenna_gain` |

---

## 2. Modelos de Path Loss

A função `pathloss(distance, frequency_mhz, model_pathloss, ...)` em `network.py` suporta nove modelos selecionáveis por configuração.

### 2.1 Log-Normal Shadowing (padrão)

Modelo padrão nas avaliações principais. Oferece equilíbrio realista entre simplicidade analítica e variabilidade ambiental:

```
PL(d) = PL(d₀) + 10·γ·log₁₀(d / d₀) + X_σ
```

Onde `X_σ ~ N(0, σ²)` representa o shadowing — variável aleatória gaussiana de média zero que captura a variabilidade de pacote a pacote causada por obstáculos e irregularidades do terreno.

**Parâmetros implementados (`network.py`):**

| Parâmetro | Valor | Descrição |
|---|---|---|
| `d₀` | 1.0 m | Distância de referência |
| `PL(d₀)` | 7.7 dB | Path loss na distância de referência |
| `γ` | 3.76 | Expoente de path loss |
| `σ` | 3.57 dB | Desvio padrão do shadowing |

```python
# network.py
d0, PL_d0, gamma, sigma = 1.0, 7.7, 3.76, 3.57
X_sigma = np.random.normal(0, sigma)
path_loss = PL_d0 + 10 * gamma * np.log10(distance / d0) + X_sigma
```

### 2.2 Correlated Shadowing

Melhora o realismo em cenários onde devices próximos experimentam condições similares de atenuação em larga escala. Em vez de valores de shadowing independentes por enlace, a posição espacial do device é usada para obter um valor correlacionado a partir de um mapa pré-gerado (`CorrelatedShadowingMap`):

```python
# network.py
d0, PL_d0, gamma = 1.0, 7.7, 3.76
base_loss = PL_d0 + 10 * gamma * np.log10(distance / d0)
shadowing  = self.shadowing_map.get_shadowing(device_x, device_y)
path_loss  = base_loss + shadowing
```

O mapa é gerado uma vez na inicialização e consultado por interpolação bilinear (ver Seção 3).

**Ativação:**
```python
Network(..., correlated_shadowing=True)
```

### 2.3 Okumura-Hata

Modelo empírico para ambientes urbanos e suburbanos. Formulação padrão com fator de correção de antena de terminal:

```
PL = 69.55 + 26.16·log₁₀(f) - 13.82·log₁₀(hₜ) - aH(hᵣ)
   + (44.9 - 6.55·log₁₀(hₜ))·log₁₀(d)
```

```python
# network.py
correction = (1.1 * np.log10(f) - 0.7) * hr_m - (1.56 * np.log10(f) - 0.8)
path_loss  = (69.55 + 26.16 * np.log10(f)
              - 13.82 * np.log10(ht_m) - correction
              + (44.9 - 6.55 * np.log10(ht_m)) * np.log10(distance_km))
```

Onde `f` = frequência (MHz), `hₜ` = altura do gateway (`hr_m = 30 m`), `hᵣ` = altura do device (`ht_m = 1.5 m`).

### 2.4 COST-Hata

Extensão do Okumura-Hata com coeficientes revisados para frequências acima de 1.5 GHz, compatível com o intervalo EU868:

```
PL = 46.3 + 33.9·log₁₀(f) - 13.82·log₁₀(hₜ) - aH(hᵣ)
   + (44.9 - 6.55·log₁₀(hₜ))·log₁₀(d)
```

```python
path_loss = (46.3 + 33.9 * np.log10(f)
             - 13.82 * np.log10(ht_m) - correction
             + (44.9 - 6.55 * np.log10(ht_m)) * np.log10(distance_km))
```

Também usado como modelo base para o modelo `fading`.

### 2.5 Free-Space Path Loss (FSPL)

Modelo de espaço livre para avaliações sem obstruções. Expresso em dB:

```
PL = 20·log₁₀(d) + 20·log₁₀(f) + 32.45
```

```python
path_loss = 20 * np.log10(distance_km) + 20 * np.log10(frequency_mhz) + 32.45
```

### 2.6 Log-Distance

Simplificação do modelo log-normal sem componente de shadowing. Expoente fixo `γ = 3.5`:

```
PL = 20·log₁₀(f) + 10·3.5·log₁₀(d) - 28
```

```python
path_loss = 20 * np.log10(f) + 10 * 3.5 * np.log10(distance_km) - 28
```

### 2.7 Building Penetration

Modelo log-normal shadowing com perda adicional de penetração predial amostrada de distribuição log-normal (ITU-R P.2109):

```
PL(d) = PL_lns(d) + BPL
BPL ~ LogNormal(ln(20), 0.5)     → mediana ≈ 20 dB
```

```python
# network.py
X_sigma   = np.random.normal(0, 3.57)
path_loss = PL_d0 + 10 * gamma * np.log10(distance / d0) + X_sigma
bpl       = np.random.lognormal(mean=np.log(20), sigma=0.5)
path_loss += bpl
```

Nota: este é o modelo de path loss `"building_penetration"` aplicado a todos os pacotes do enlace. O mecanismo de **penetração por device** (perda adicional para devices `is_indoor`) é implementado separadamente via `get_building_penetration()` e é independente do modelo selecionado (ver Seção 4).

### 2.8 Fading (Rayleigh / Rician / Nakagami)

Modela efeitos de propagação em pequena escala, sobrepostos ao modelo COST-Hata. Permite emular condições de propagação multipercurso em ambientes dinâmicos ou com muitos obstáculos:

```
PL_fading(d) = PL_COST-Hata(d) - fading_dB
```

```python
# Rayleigh: sem linha de visada
fading_db = 20 * np.log10(np.random.rayleigh(scale=1))

# Rician: linha de visada dominante, K=3
sample    = np.sqrt(np.random.noncentral_chi2(2, 2*K) / (2*(1+K)))
fading_db = 20 * np.log10(sample)

# Nakagami: ambiente com múltiplos clusters, m=3
sample    = np.sqrt(np.random.gamma(shape=m, scale=1.0/m))
fading_db = 20 * np.log10(sample)
```

**Parâmetros fixos:** K = 3 (Rician), m = 3 (Nakagami).

### 2.9 Oulu (Compatibilidade FLoRa/LoRaSim)

Modelo usado em LoRaSim (Bor et al. 2016) e FLoRa. Referência padrão para comparação:

```
PL(d) = 127.41 + 10·2.08·log₁₀(d / 40) + N(0, 3.57²)
```

```python
# network.py
d0, PL_d0, gamma, sigma = 40.0, 127.41, 2.08, 3.57
X_sigma   = np.random.normal(0, sigma)
path_loss = PL_d0 + 10 * gamma * np.log10(max(distance / d0, 1e-10)) + X_sigma
```

### 2.10 Comparativo de Parâmetros

| Modelo | `d₀` | `PL(d₀)` | `γ` | `σ` | Base |
|---|---|---|---|---|---|
| `log_normal_shadowing` | 1 m | 7.7 dB | 3.76 | 3.57 dB | FLoRa/ns-3 |
| `correlated_shadowing` | 1 m | 7.7 dB | 3.76 | 3.57 dB | ns-3 |
| `building_penetration` | 1 m | 7.7 dB | 3.76 | 3.57 dB | ITU-R P.2109 |
| `oulu` | 40 m | 127.41 dB | 2.08 | 3.57 dB | LoRaSim/FLoRa |
| `log_distance` | 1 km | — | 3.5 | — | Teórico |
| `fspl` | — | — | 2.0 | — | Espaço livre |
| `okumura_hata` | — | — | varia | — | Empírico urbano |
| `cost_hata` | — | — | varia | — | COST 231 |
| `fading` | — | — | COST | — | + Rayleigh/Rician/Nakagami |

---

## 3. Shadowing Espacialmente Correlacionado — `CorrelatedShadowingMap`

### 3.1 Motivação

Em implantações densas, devices próximos tendem a experimentar condições de atenuação semelhantes — obstáculos e o terreno afetam múltiplos enlaces de forma correlacionada. O modelo independente (sorteio i.i.d. por pacote) não captura essa característica.

### 3.2 Implementação

`CorrelatedShadowingMap` approxima o modelo `ns-3 CorrelatedShadowingPropagationLossModel` com correlação exponencial (`d_corr ≈ 110 m`):

```python
class CorrelatedShadowingMap:
    def __init__(self, area_size, grid_step=50, sigma=3.57):
        self.grid_step   = grid_step       # 50 m entre pontos de grid
        self.grid_points = int(area_size / grid_step) + 1
        # Grid de shadowing fixo: N(0, σ²) por ponto
        self.grid = np.random.normal(0, sigma, (self.grid_points, self.grid_points))
```

O grid é gerado **uma única vez** na inicialização com `sigma = 3.57 dB`. Isso garante que o shadowing seja espacialmente estável ao longo da simulação.

### 3.3 Interpolação Bilinear

Para qualquer posição `(x, y)` do device, o valor de shadowing é interpolado entre os quatro vértices do quadrado de grid mais próximo:

```python
def get_shadowing(self, x, y):
    gx, gy = x / grid_step, y / grid_step
    x0, y0 = int(gx), int(gy)
    dx, dy  = gx - x0, gy - y0

    # Interpolação bilinear dos quatro vizinhos
    return ((1-dx)*(1-dy)*grid[x0,y0] + dx*(1-dy)*grid[x1,y0]
           + (1-dx)*dy*grid[x0,y1]    + dx*dy*grid[x1,y1])
```

Com `grid_step = 50 m`, a correlação espacial efetiva é de ~50–100 m entre amostras adjacentes — consistente com o valor de referência `d_corr = 110 m` do ns-3.

---

## 4. Perda de Penetração Predial (G3)

Devices marcados como `is_indoor` (30% por padrão) recebem uma perda adicional de penetração na inicialização, amostrada uma vez e mantida fixa durante toda a simulação:

```python
# network.py — initialize_devices()
if device.is_indoor:
    self._building_penetration_cache[device_id] = np.random.lognormal(
        mean=np.log(20), sigma=0.5
    )
```

Esta perda é somada ao path loss em **todos** os cálculos de link budget — tanto em `find_best_gateway()` quanto em `assign_sf_by_distance()`:

```python
path_loss += self.get_building_penetration(device)
# → adiciona BPL ~ LogNormal(ln(20), 0.5) se is_indoor, 0.0 caso contrário
```

**Distribuição da perda:** mediana = 20 dB, com σ = 0.5 (escala log). Representa paredes de concreto/alvenaria típicas de edificações urbanas.

---

## 5. Link Budget — Potência Recebida

Para cada transmissão, a potência recebida no gateway é calculada combinando potência de transmissão, ganhos de antena, path loss e perda de penetração:

```
Pᵣ = Pₜ + Gₜ + Gᵣ - PL(d) - L_add     [dBm]
```

Implementação em `find_best_gateway()`:

```python
rssi = device.tx_power + ed_antenna_gain + gw_antenna_gain - path_loss
#       14 dBm            +  0 dBi         +  3 dBi          - PL(d)
```

**Parâmetros de antena (`parametors.py`):**

| Parâmetro | Valor | Descrição |
|---|---|---|
| `ed_antenna_gain` | 0 dBi | Antena do End Device (chip antenna) |
| `gw_antenna_gain` | 3 dBi | Antena do Gateway (omnidirecional) |
| `ht_m` | 1.5 m | Altura do End Device |
| `hr_m` | 30.0 m | Altura do Gateway |

A distância 3D incorpora a diferença de alturas:

```python
distance = max(sqrt((dx)² + (dy)² + (ht_m - hr_m)²), 1.0)
         = max(sqrt(d_horizontal² + 812.25), 1.0)
```

---

## 6. Sensibilidade do Receptor e Limiares SNR

### 6.1 Sensibilidade do Gateway SX1301

O gateway usa o chip SX1301 (~6 dB mais sensível que o SX1272 do ED). Um pacote é considerado recebível apenas se `RSSI > gw_sensitivity[SF, BW]`:

```python
# parametors.py — gw_sensitivity_table (SX1301)
{
    (7,  125000): -130.0,  (8,  125000): -132.5,  (9,  125000): -135.0,
    (10, 125000): -137.5,  (11, 125000): -140.0,  (12, 125000): -142.5,
}
```

### 6.2 Limiares Mínimos de SNR por SF

Determinam a condição mínima de demodulação antes da análise de interferência. Valores padrão LoRa (Semtech / ns-3 / FLoRa):

```python
# parametors.py
snr_min_per_sf = {
    12: -20.0,   11: -17.5,   10: -15.0,
     9: -12.5,    8: -10.0,    7:  -7.5,    # dB
}
```

### 6.3 Ruído Térmico

O piso de ruído é calculado a partir da largura de banda e temperatura do receptor:

```
NF [dBm] = 10·log₁₀(k·T·BW) + 30 + NF_dB
```

```python
# network.py — calculate_noise_floor()
noise_floor = 10 * np.log10(k * temperatura * bw) + 30 + noise_figure
#  k = 1.38e-23 J/K,  T = 294.15 K (~21°C),  NF = 6 dB
```

**Para BW = 125 kHz:**
```
NF = 10·log₁₀(1.38e-23 × 294.15 × 125000) + 30 + 6
   = 10·log₁₀(5.074e-16) + 30 + 6
   = -152.95 + 30 + 6 = -116.95 dBm
```

O SNR é então: `SNR = RSSI - noise_floor [dB]`

---

## 7. Pipeline de Avaliação de Recepção — `evaluate_reception()`

Para cada pacote ao final da transmissão (`_on_tx_end`), `ChannelModel.evaluate_reception()` executa quatro passos sequenciais:

### 7.1 Passo 1 — Verificação de SNR

```python
noise_floor  = network.calculate_noise_floor(packet.bw)
snr          = packet.snr_mrc if packet.snr_mrc else (packet.rssi - noise_floor)
packet.snr   = snr
snr_required = snr_min_per_sf[packet.sf]

if snr < snr_required:
    packet.collided = True
    return False             # falha por ruído — sem análise de interferência
```

Sprint 9 (MRC): se múltiplos gateways receberam o pacote, `snr_mrc = 10·log₁₀(Σ 10^(snrᵢ/10))` é usado em lugar do SNR de um único gateway.

### 7.2 Passo 2 — Preamble Locking (G13)

Interferentes que chegam **após** o receptor ter travado no preâmbulo do pacote alvo não afetam a demodulação:

```python
Tsym              = (2^SF) / BW        # duração de um símbolo LoRa
preamble_lock_time = tx_start + 6 * Tsym   # lock após 6 símbolos
```

Interferentes com `other_start >= preamble_lock_time` são excluídos da análise.

### 7.3 Passo 3 — Seleção da Matriz de Interferência

Selecionada em runtime via `parametors.interference_model`:

```python
matrix = (interference_matrix_goursaud
          if parametors.interference_model == "goursaud"
          else interference_matrix)     # default: Semtech AN1200.18
```

### 7.4 Passo 4 — Capture Effect com Razão de Energia (G14)

Para cada interferente com sobreposição temporal, é avaliada a razão de energia acumulada (modelo ns-3 `LoraInterferenceHelper`):

```
E_signal     = P_signal × T_packet
E_interferer = P_interferer × T_overlap

energy_ratio_dB = (RSSI_signal − RSSI_interferer) + 10·log₁₀(T_packet / T_overlap)
```

```python
energy_correction_db = 10.0 * math.log10(packet_duration / max(overlap_duration, 1e-9))
energy_ratio_db      = (packet.rssi - interferer.rssi) + energy_correction_db

if energy_ratio_db < threshold_db:    # threshold da matriz [SF_target][SF_inter]
    survived = False
    break
```

O pacote sobrevive apenas se `energy_ratio_db ≥ threshold` para **todos** os interferentes com sobreposição.

---

## 8. Matrizes de Interferência

### 8.1 Semtech AN1200.18 (padrão)

Limiares de razão energia sinal/interferente (dB) para sobreviver à colisão. Linhas = SF alvo, colunas = SF interferente (SF12..SF7):

```python
interference_matrix = [
    [ 1,  -23, -24, -25, -25, -25],  # SF12 alvo
    [-20,   1, -20, -21, -22, -22],  # SF11
    [-18, -17,   1, -17, -18, -19],  # SF10
    [-15, -14, -13,   1, -13, -15],  # SF9
    [-13, -13, -12, -11,   1, -11],  # SF8
    [ -9,  -9,  -9,  -9,  -8,   1],  # SF7
]
```

Diagonal `= 1 dB` (co-SF): sinal deve ser apenas 1 dB mais forte que o interferente para captura.

### 8.2 Goursaud et al. (ns-3 padrão)

Modelo mais conservador: co-SF exige 6 dB de margem (threshold maior = mais difícil capturar):

```python
interference_matrix_goursaud = [
    [  6, -36, -36, -36, -36, -36],  # SF12
    [-29,   6, -33, -33, -33, -33],  # SF11
    [-28, -26,   6, -30, -30, -30],  # SF10
    [-27, -25, -23,   6, -27, -27],  # SF9
    [-24, -22, -20, -19,   6, -24],  # SF8
    [-20, -16, -18, -19, -16,   6],  # SF7
]
```

---

## 9. Integração com o Subsistema de Mobilidade

O modelo de canal opera conjuntamente com o subsistema de mobilidade. Quando a mobilidade está habilitada, o simulador atualiza periodicamente as posições dos nós e recalcula distâncias de propagação, condições de shadowing e níveis de potência recebida.

A cada atualização de mobilidade (`_on_mobility_update`, intervalo de 1 s), `update_coverage_status()` chama `find_best_gateway()`, que:

1. Recalcula `distance = sqrt(Δx² + Δy² + Δh²)` com a nova posição
2. Aplica `pathloss(distance, freq, model_pathloss, device_x, device_y)` — no modelo `correlated_shadowing`, a posição atual do device determina o valor do mapa
3. Soma `get_building_penetration(device)` (constante por device)
4. Recalcula `rssi = tx_power + G_ed + G_gw - path_loss`
5. Compara com `gw_sensitivity[SF, BW]` — atualiza `coverage_status`

No modelo `log_normal_shadowing`, um novo valor de `X_σ` é amostrado a cada chamada — modela variação temporal do canal de pacote a pacote. No modelo `correlated_shadowing`, o valor é determinístico para uma posição fixa (consistência espacial).

---

## 10. Fluxo Completo por Transmissão

```
_on_device_send(device, t)
  │
  ├── find_best_gateway(device)
  │     ├── distance = sqrt(Δx² + Δy² + Δh²)
  │     ├── path_loss = pathloss(d, f, model)   ← 1 dos 9 modelos
  │     │               + get_building_penetration(device)
  │     ├── rssi = Pₜ + G_ed + G_gw - path_loss
  │     └── if rssi > gw_sensitivity[SF,BW]: gateway disponível
  │
  ├── Packet(rssi=rssi, ...)
  ├── channel.add_transmission(packet, t, t+airtime)
  │
  └── scheduler.schedule(airtime, TX_END)

_on_tx_end(device, packet, t+airtime)
  │
  └── channel.evaluate_reception(packet, gateway)
        │
        ├── noise_floor = 10·log₁₀(k·T·BW) + 30 + NF
        ├── snr = snr_mrc ?? (rssi - noise_floor)
        ├── if snr < snr_min_per_sf[SF]: collided=True → return False
        │
        ├── preamble_lock_time = tx_start + 6·Tsym
        ├── Filtra interferentes: mesmo canal, sobreposição, antes do lock
        │
        ├── Se nenhum interferente: collided=False → return True
        │
        └── Para cada interferente:
              threshold  = matrix[SF_target][SF_inter]
              E_ratio_dB = (rssi_signal - rssi_inter) + 10·log₁₀(T_pkt/T_ovlp)
              if E_ratio_dB < threshold: collided=True → return False
            packet.collided = False → return True
```
