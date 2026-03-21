# PyLoRaWAN — Channel and Propagation Module

> Documentação técnica detalhada dos modelos de canal e propagação.
> Cobre `channel.py`, `CorrelatedShadowingMap`, todos os 9 modelos de path loss,
> matrizes de interferência, capture effect, preamble locking e interações com o gateway.

---

## Sumário

1. [Visão Geral do Módulo](#1-visão-geral-do-módulo)
2. [ChannelModel — Estrutura e API](#2-channelmodel--estrutura-e-api)
3. [Rastreamento On-Air](#3-rastreamento-on-air)
4. [evaluate_reception — Pipeline Completo](#4-evaluate_reception--pipeline-completo)
   - 4.1 [Passo 1: SNR vs. Ruído Térmico](#41-passo-1-snr-vs-ruído-térmico)
   - 4.2 [Passo 2: Preamble Locking (G13)](#42-passo-2-preamble-locking-g13)
   - 4.3 [Passo 3: Seleção da Matriz de Interferência](#43-passo-3-seleção-da-matriz-de-interferência)
   - 4.4 [Passo 4: Capture Effect — Modelo de Energia (G14)](#44-passo-4-capture-effect--modelo-de-energia-g14)
   - 4.5 [Resultado e Estatísticas](#45-resultado-e-estatísticas)
5. [Matrizes de Interferência](#5-matrizes-de-interferência)
   - 5.1 [Semtech AN1200.18](#51-semtech-an120018)
   - 5.2 [Goursaud (ns-3 padrão)](#52-goursaud-ns-3-padrão)
   - 5.3 [Interpretação das Matrizes](#53-interpretação-das-matrizes)
   - 5.4 [Comparação e Impacto no PDR](#54-comparação-e-impacto-no-pdr)
6. [Ruído Térmico — calculate_noise_floor](#6-ruído-térmico--calculate_noise_floor)
7. [Cálculo de SINR no Gateway](#7-cálculo-de-sinr-no-gateway)
8. [Modelos de Propagação (9 modelos)](#8-modelos-de-propagação-9-modelos)
   - 8.1 [log_normal_shadowing](#81-log_normal_shadowing-padrão)
   - 8.2 [correlated_shadowing](#82-correlated_shadowing)
   - 8.3 [okumura_hata](#83-okumura_hata)
   - 8.4 [fspl](#84-fspl)
   - 8.5 [log_distance](#85-log_distance)
   - 8.6 [cost_hata](#86-cost_hata)
   - 8.7 [building_penetration](#87-building_penetration)
   - 8.8 [fading](#88-fading)
   - 8.9 [oulu](#89-oulu)
9. [CorrelatedShadowingMap](#9-correlatedshadowingmap)
10. [Building Penetration Cache (G3)](#10-building-penetration-cache-g3)
11. [Link Budget — Integração Completa](#11-link-budget--integração-completa)
12. [Interação Channel ↔ Gateway ↔ Network](#12-interação-channel--gateway--network)
13. [Análise de Sensibilidade dos Parâmetros](#13-análise-de-sensibilidade-dos-parâmetros)
14. [Exemplos Numéricos Completos](#14-exemplos-numéricos-completos)

---

## 1. Visão Geral do Módulo

O módulo de canal e propagação é composto por dois arquivos principais e elementos embutidos no orquestrador:

```
channel.py                    → ChannelModel
  ├── on_air tracking         → lista de transmissões ativas
  ├── evaluate_reception()    → decisão de colisão (SINR + capture effect)
  └── cleanup_expired()       → gerenciamento de memória

network.py                    → Network (métodos de propagação)
  ├── pathloss()              → 9 modelos de path loss
  ├── calculate_noise_floor() → ruído térmico Johnson-Nyquist
  ├── get_building_penetration() → perda indoor cacheada
  └── CorrelatedShadowingMap  → mapa de shadowing espacialmente correlacionado

parametors.py                 → Constantes PHY
  ├── interference_matrix     → Semtech AN1200.18 (6×6)
  ├── interference_matrix_goursaud → Goursaud et al. (6×6)
  ├── snr_min_per_sf          → {SF → SNR_min dB}
  ├── sensitivity_table       → {(SF, BW) → dBm} — SX1272 (ED)
  └── gw_sensitivity_table    → {(SF, BW) → dBm} — SX1301 (GW)
```

### Referências de Implementação

| Componente                | Referência                                    |
|---------------------------|-----------------------------------------------|
| Modelo log-normal         | FLoRa (Aerts et al.), ns-3 LoRaWAN            |
| Interferência energy-based| ns-3 `LoraInterferenceHelper`                 |
| Matrizes Semtech          | Semtech AN1200.18 LoRa Modem Design Guide     |
| Matrizes Goursaud         | Goursaud & Gorce 2016, ns-3 padrão            |
| Preamble locking          | ns-3 `LoraInterferenceHelper::AddPacket()`    |
| Shadowing correlacionado  | ns-3 `CorrelatedShadowingPropagationLossModel`|
| Building penetration      | ns-3 `BuildingsPropagationLossModel`          |
| Modelo Oulu               | LoRaSim (Bor et al. 2016), FLoRa              |

---

## 2. ChannelModel — Estrutura e API

**Arquivo:** `channel.py`

```python
class ChannelModel:
    network: Network          # referência ao orquestrador (para noise_floor)
    on_air: list              # [(Packet, tx_start, tx_end), ...]
    total_collisions: int     # estatística global
    total_receptions: int     # estatística global
```

### API Completa

| Método                                       | Complexidade | Descrição                                        |
|----------------------------------------------|-------------|--------------------------------------------------|
| `add_transmission(pkt, tx_start, tx_end)`    | O(1)        | Registra pacote no on_air                       |
| `evaluate_reception(pkt, gateway) → bool`    | O(N)        | Avalia colisão — N = on_air atual               |
| `cleanup_expired(current_time)`              | O(N)        | Remove tx_end ≤ current_time                    |
| `get_on_air_count() → int`                   | O(1)        | Tamanho atual do on_air                         |
| `get_active_on_frequency(freq, t) → list`    | O(N)        | Filtra por frequência e tempo                   |
| `stats() → dict`                             | O(1)        | `{total_collisions, total_receptions, on_air_count}` |

### Nota de Design

`evaluate_reception()` é chamado em `_on_tx_end()` (após o fim da transmissão), não no início. Isso garante que todos os pacotes com sobreposição temporal já estejam no `on_air` antes da avaliação — equivalente ao comportamento do ns-3.

---

## 3. Rastreamento On-Air

### Estrutura

```python
on_air: list[tuple[Packet, float, float]]
         # (packet,  tx_start,  tx_end)
```

### Ciclo de Vida

```
_on_device_send()
  ↓
  channel.add_transmission(packet, t, t + rectime)
  → pacote entra em on_air imediatamente

_on_tx_end()
  ↓
  channel.evaluate_reception(packet, gw)
  → avalia com snapshot atual de on_air
  ↓
  channel.cleanup_expired(current_time)
  → remove (p, s, e) onde e ≤ current_time
```

### Por que não remover em add_transmission?

Um pacote removido prematuramente deixaria de ser visto como interferente por transmissões que começam durante seu airtime. A remoção só ocorre em `_on_tx_end`, garantindo que o on_air seja uma foto precisa de todas as transmissões sobrepostas.

### Crescimento do on_air

Com 50 EDs, λ=1/300s, SF médio ≈ SF9 (ToA≈144ms):
- Taxa de chegada: ~50/300 ≈ 0.17 pkt/s
- Tempo médio no on_air: 144ms
- Tamanho médio: 0.17 × 0.144 ≈ **0.02 pacotes** simultâneos (baixo no cenário ref.)
- Pico com SF12 (ToA≈1155ms): até ~0.2 pacotes simultâneos

---

## 4. evaluate_reception — Pipeline Completo

**Arquivo:** `channel.py` — linhas 24–113

```python
def evaluate_reception(self, packet, gateway):
```

### Visão em Fluxograma

```
ENTRADA: packet (com rssi, sf, bw, arrival_time, rectime populados)
         gateway (referência, para acesso ao network)

PASSO 1: SNR >= SNR_min[SF] ?
         ├── NÃO → packet.collided = True → return False
         └── SIM → continua

PASSO 2: Preamble locking
         → preamble_lock_time = tx_start + 6 * Tsym
         → filtra interferentes: other_start < preamble_lock_time

PASSO 3: Seleciona matriz
         → "goursaud" se parametors.interference_model == "goursaud"
         → "semtech" caso contrário

PASSO 4: Para cada interferente na mesma freq com sobreposição:
         → energy_ratio_dB = (RSSI_signal - RSSI_interferer)
                            + 10 * log10(T_packet / T_overlap)
         → threshold_dB = matrix[sf_target_idx][sf_interferer_idx]
         → energy_ratio_dB < threshold_dB?
              ├── SIM → survived = False; break
              └── NÃO → continua

SAÍDA:   packet.collided = not survived
         packet.snr = snr calculado
         packet.noise_floor = noise_floor calculado
         total_collisions ou total_receptions incrementado
         return survived
```

---

### 4.1 Passo 1: SNR vs. Ruído Térmico

```python
noise_floor = network.calculate_noise_floor(packet.bw)

# Sprint 9: MRC — usa SNR combinado se disponível
if packet.snr_mrc is not None:
    snr = packet.snr_mrc
else:
    snr = packet.rssi - noise_floor  if packet.rssi is not None else -999

packet.snr = snr
packet.noise_floor = noise_floor

snr_required = snr_min_per_sf.get(packet.sf, -20.0)
if snr < snr_required:
    packet.collided = True
    total_collisions += 1
    return False
```

#### SNR Mínimo por SF

| SF  | SNR_min (dB) | Razão |
|-----|-------------|-------|
| SF7 | −7.5        | SF mais rápido, menos ganho de processamento |
| SF8 | −10.0       | |
| SF9 | −12.5       | |
| SF10| −15.0       | |
| SF11| −17.5       | |
| SF12| −20.0       | SF mais lento, maior ganho de processamento |

Este teste elimina pacotes que chegam com sinal muito fraco para serem demodulados mesmo em silêncio absoluto — representa o limite fundamental da sensibilidade do receptor.

---

### 4.2 Passo 2: Preamble Locking (G13)

```python
Tsym = (2.0 ** packet.sf) / packet.bw

preamble_lock_time = tx_start + 6 * Tsym
```

**Conceito:** O receptor LoRa detecta e sincroniza no preâmbulo nos primeiros 6 símbolos. Após o lock, a sincronização está estabelecida e interferentes tardios não podem desestabilizá-la.

```python
for (other_pkt, other_start, other_end) in self.on_air:
    ...
    # G13: interferente chegou APÓS o lock → não interfere
    if other_start >= preamble_lock_time:
        continue
    # interferente chegou ANTES do lock → avaliado normalmente
    interferers.append((other_pkt, overlap_duration, other_start))
```

#### Preamble Lock Time por SF (BW=125kHz)

| SF  | Tsym    | 6 × Tsym | Janela vulnerável |
|-----|---------|----------|-------------------|
| SF7 | 1.024ms | 6.1ms    | Curta — pouco exposta |
| SF8 | 2.048ms | 12.3ms   | |
| SF9 | 4.096ms | 24.6ms   | |
| SF10| 8.192ms | 49.2ms   | |
| SF11| 16.38ms | 98.3ms   | |
| SF12| 32.77ms | 196.6ms  | Longa — mais exposta |

> SF12 demora ~197ms para travar o preâmbulo — qualquer interferente que chegue durante esse período conta.

---

### 4.3 Passo 3: Seleção da Matriz de Interferência

```python
# Lido em runtime para testabilidade (não cacheado)
matrix = (interference_matrix_goursaud
          if parametors.interference_model == "goursaud"
          else interference_matrix)
```

A seleção é dinâmica — permite trocar a matriz durante a simulação sem reinicialização. Padrão: `"semtech"`.

---

### 4.4 Passo 4: Capture Effect — Modelo de Energia (G14)

```python
survived = True
for interferer, overlap_duration, _ in interferers:
    sf_target_idx   = 12 - packet.sf      # SF12→0, SF7→5
    sf_inter_idx    = 12 - interferer.sf

    threshold_db = matrix[sf_target_idx][sf_inter_idx]

    if packet.rssi is not None and interferer.rssi is not None:
        energy_correction_db = 10.0 * math.log10(
            packet_duration / max(overlap_duration, 1e-9))

        energy_ratio_db = (packet.rssi - interferer.rssi) + energy_correction_db

        if energy_ratio_db < threshold_db:
            survived = False
            break
```

#### Derivação da Fórmula

```
Energia do sinal:       E_s = P_s × T_packet
Energia interferente:   E_i = P_i × T_overlap

Razão de energia:
  E_s / E_i = (P_s / P_i) × (T_packet / T_overlap)

Em dB:
  energy_ratio_dB = 10·log10(P_s/P_i) + 10·log10(T_packet/T_overlap)
                  = (RSSI_s − RSSI_i)  + energy_correction_dB

onde:
  energy_correction_dB = 10·log10(T_packet / T_overlap)
```

#### Casos Extremos

| Cenário                      | T_overlap / T_packet | Correção    | Efeito                          |
|------------------------------|----------------------|-------------|---------------------------------|
| Sobreposição total           | 1.0                  | 0 dB        | Puro RSSI ratio                 |
| Sobreposição de 50%          | 0.5                  | +3.0 dB     | Sinal levemente favorecido      |
| Sobreposição de 10%          | 0.1                  | +10.0 dB    | Sinal muito favorecido          |
| Sobreposição de 1%           | 0.01                 | +20.0 dB    | Interferente quase irrelevante  |
| Sobreposição total + i > s   | 1.0                  | 0 dB        | Depende só do threshold         |

#### Índices das Matrizes

```
SF12 → índice 0    (12 - 12 = 0)
SF11 → índice 1
SF10 → índice 2
SF9  → índice 3
SF8  → índice 4
SF7  → índice 5    (12 - 7  = 5)

matrix[sf_target_idx][sf_interferer_idx]
  = threshold em dB que energy_ratio_dB deve SUPERAR para sobreviver
```

---

### 4.5 Resultado e Estatísticas

```python
packet.collided = not survived

if survived:
    self.total_receptions += 1
else:
    self.total_collisions += 1

return survived
```

`total_collisions` e `total_receptions` são contadores acumulativos do canal — distintos dos contadores por pacote armazenados no `Packet` e no `PacketTracker`.

---

## 5. Matrizes de Interferência

Ambas as matrizes são **6×6** onde:
- **Linhas**: SF do pacote **alvo** (SF12=0, SF7=5)
- **Colunas**: SF do pacote **interferente** (SF12=0, SF7=5)
- **Valor**: threshold em dB que `energy_ratio_dB` deve ser ≥ para o alvo sobreviver

---

### 5.1 Semtech AN1200.18

**Fonte:** Semtech Application Note AN1200.18 — LoRa Modem Design Guide

```python
interference_matrix = [
    # SF12  SF11  SF10   SF9   SF8   SF7   ← interferente
    [  1,   -23,  -24,  -25,  -25,  -25],  # SF12 alvo
    [-20,     1,  -20,  -21,  -22,  -22],  # SF11 alvo
    [-18,   -17,    1,  -17,  -18,  -19],  # SF10 alvo
    [-15,   -14,  -13,    1,  -13,  -15],  # SF9  alvo
    [-13,   -13,  -12,  -11,    1,  -11],  # SF8  alvo
    [ -9,    -9,   -9,   -9,   -8,    1],  # SF7  alvo
]
```

**Co-SF (diagonal):** threshold = **+1 dB**
- Para dois pacotes SF9 colidirem, o interferente precisa ser menos de 1 dB mais fraco
- Um sinal apenas 1 dB mais forte "captura" o canal — capture effect forte

**Cross-SF (off-diagonal):**
- SF7 (alvo) vs SF12 (interferente): threshold = −9 dB → SF7 sobrevive se for apenas 9 dB mais fraco que SF12
- SF12 (alvo) vs SF7 (interferente): threshold = −25 dB → SF12 é muito tolerante a interferência SF7

---

### 5.2 Goursaud (ns-3 padrão)

**Fonte:** Goursaud & Gorce 2016, implementado como padrão no ns-3 LoRaWAN

```python
interference_matrix_goursaud = [
    # SF12  SF11  SF10   SF9   SF8   SF7   ← interferente
    [   6,  -36,  -36,  -36,  -36,  -36],  # SF12 alvo
    [ -29,    6,  -33,  -33,  -33,  -33],  # SF11 alvo
    [ -28,  -26,    6,  -30,  -30,  -30],  # SF10 alvo
    [ -27,  -25,  -23,    6,  -27,  -27],  # SF9  alvo
    [ -24,  -22,  -20,  -19,    6,  -24],  # SF8  alvo
    [ -20,  -16,  -18,  -19,  -16,    6],  # SF7  alvo
]
```

**Co-SF (diagonal):** threshold = **+6 dB**
- O sinal precisa ser 6 dB mais forte que o interferente para capturar — critério mais exigente
- Mais conservador: mais colisões co-SF

**Cross-SF (off-diagonal):**
- Thresholds muito mais negativos (−16 a −36 dB) → SFs ainda mais ortogonais entre si
- Ex: SF7 vs SF12: threshold = −20 dB (Goursaud) vs −9 dB (Semtech) → Goursaud mais permissivo em cross-SF

---

### 5.3 Interpretação das Matrizes

#### Leitura correta

`matrix[row][col]` = energia mínima que o **pacote alvo** (SF da linha) precisa ter em relação ao **interferente** (SF da coluna) para sobreviver.

```
matrix[3][3] = 1  (Semtech)
→ SF9 alvo vs SF9 interferente: precisa de energy_ratio ≥ 1 dB
→ Se signal = -105 dBm, interferer = -107 dBm (sobreposição total):
   energy_ratio = (-105) - (-107) = 2 dB > 1 dB → SOBREVIVE

matrix[3][3] = 6  (Goursaud)
→ Mesmas condições: 2 dB < 6 dB → COLISÃO
```

#### Assimetria das Matrizes

As matrizes **não são simétricas**, refletindo a assimetria real do LoRa:
- SF12 com alta robustez: sobrevive mesmo com interferência de SF mais baixos (thresholds negativos grandes)
- SF7 com menos robustez: precisa dominar mais os interferentes co-SF

---

### 5.4 Comparação e Impacto no PDR

| Parâmetro                   | Semtech        | Goursaud       |
|-----------------------------|----------------|----------------|
| Co-SF threshold             | +1 dB          | +6 dB          |
| Cross-SF thresholds         | −8 a −25 dB    | −16 a −36 dB   |
| PDR típico (50 EDs, 1 GW)  | ~92%           | ~87%           |
| Colisões co-SF              | Menos (~8%)    | Mais (~13%)    |
| Colisões cross-SF           | Mais           | Menos          |
| Padrão no simulador         | ✓ (padrão)     | Opcional       |
| Padrão no ns-3              | ✗              | ✓ (padrão)     |
| Padrão no FLoRa             | ✓              | ✗              |

---

## 6. Ruído Térmico — calculate_noise_floor

**Arquivo:** `network.py` — método `calculate_noise_floor(bw)`

```python
def calculate_noise_floor(self, bw):
    noise_floor = 10 * np.log10(k * temperatura * bw) + 30 + noise_figure
    return noise_floor
```

### Parâmetros

| Constante          | Valor           | Fonte                     |
|--------------------|-----------------|---------------------------|
| `k`                | 1.38 × 10⁻²³ J/K | Constante de Boltzmann   |
| `temperatura`      | 294.15 K        | ~21°C (temperatura padrão)|
| `noise_figure`     | 6 dB            | SX1272 datasheet          |
| `+30`              | conversão       | W → mW → dBm             |

### Fórmula Expandida

```
noise_floor [dBm] = 10·log10(k · T · BW) + 30 + NF

= 10·log10(1.38e-23 × 294.15 × BW) + 30 + 6
```

### Valores Calculados

| BW       | k·T·BW (W)   | 10·log10(W) | +30 (dBm) | +NF    | = Noise Floor |
|----------|-------------|-------------|-----------|--------|---------------|
| 125 kHz  | 5.07 × 10⁻¹⁶ | −152.9 dBW | −122.9 dBm | +6 dB | **−116.9 dBm** |
| 250 kHz  | 1.01 × 10⁻¹⁵ | −149.9 dBW | −119.9 dBm | +6 dB | **−113.9 dBm** |
| 500 kHz  | 2.03 × 10⁻¹⁵ | −146.9 dBW | −116.9 dBm | +6 dB | **−110.9 dBm** |

> Nota: A conversão correta é +30 para W→mW (+10), depois já está em dBm. O valor típico citado na literatura para BW=125kHz é ~−120 dBm; a diferença vem do arredondamento e temperatura de referência usada.

### SNR a partir do Noise Floor

```
SNR [dB] = RSSI [dBm] - noise_floor [dBm]

Ex: RSSI = -105 dBm, BW = 125kHz
    SNR = -105 - (-116.9) = 11.9 dB
    SNR_min[SF9] = -12.5 dB
    11.9 >> -12.5 → sinal bem acima do mínimo
```

---

## 7. Cálculo de SINR no Gateway

**Arquivo:** `gateway.py` — `process_uplink()`

O SINR é calculado **antes** de `evaluate_reception()`, durante o processamento do uplink pelo gateway.

### Fórmula

```
SINR_linear = P_signal / (P_interference + P_noise)

onde cada potência é em unidade linear (mW):
  P_X_linear = 10^(X_dBm / 10)

SINR_dB = 10 · log10(SINR_linear)
```

### Acumulação de Interferência (G14)

```python
interference_power_linear = 0
interference_per_sf = {}

for (other_pkt, tx_start, tx_end) in channel.on_air:
    if other_pkt.packet_id == packet.packet_id: continue
    if other_pkt.freq != packet.freq: continue

    # Sobreposição temporal
    overlap_start    = max(current_time, tx_start)
    overlap_end      = min(current_time + packet.rectime, tx_end)
    overlap_duration = overlap_end - overlap_start
    if overlap_duration <= 0: continue

    # Peso proporcional ao overlap (modelo de energia acumulada)
    overlap_ratio = overlap_duration / packet.rectime

    # RSSI do interferente neste GW
    other_rssi_dBm    = calc_rssi(other_device, gateway)
    other_rssi_linear = 10 ** (other_rssi_dBm / 10)

    interference_power_linear          += other_rssi_linear * overlap_ratio
    interference_per_sf[other_pkt.sf]  += other_rssi_linear * overlap_ratio
```

### SINR Final

```python
noise_linear   = 10 ** (noise_floor / 10)
signal_linear  = 10 ** (packet.rssi / 10)

sinr_linear    = signal_linear / (interference_power_linear + noise_linear)
packet.sinr    = 10 * np.log10(sinr_linear)

# SIR e SNR calculados separadamente para retrocompatibilidade
if interference_power_linear > 0:
    packet.sir = packet.rssi - 10 * np.log10(interference_power_linear)
else:
    packet.sir = float('inf')

packet.snr = packet.rssi - packet.noise_floor
```

### Diferença SINR vs SNR

| Métrica | Fórmula                         | Inclui interferência? |
|---------|---------------------------------|-----------------------|
| SNR     | RSSI − noise_floor              | Não                   |
| SIR     | RSSI − 10·log10(I_linear)       | Só interferência      |
| SINR    | RSSI − 10·log10(I+N)_linear     | Sim (I e N juntos)    |

O **SINR** é a métrica fisicamente mais correta e é usada na avaliação de recepção.

---

## 8. Modelos de Propagação (9 modelos)

**Arquivo:** `network.py` — método `pathloss(distance, frequency_mhz, model_pathloss, fading_type=None, device_x=None, device_y=None)`

```python
# Conversão para km (modelos clássicos Hata usam distância em km)
distance_km = distance / 1000.0
```

---

### 8.1 `log_normal_shadowing` (padrão)

**Equivalente:** FLoRa (Aerts et al. 2017), ns-3 LoRaWAN (Magrin et al. 2017)

```python
d0    = 1.0       # distância de referência (metros)
PL_d0 = 7.7       # perda a d0 = 1m (dB)
gamma = 3.76      # expoente de propagação urbano
sigma = 3.57      # desvio padrão do shadowing (dB)

X_sigma = np.random.normal(0, sigma)   # nova amostra por transmissão

path_loss = PL_d0 + 10 * gamma * np.log10(distance / d0) + X_sigma
```

**Propriedades:**
- Shadowing **independente** a cada transmissão (sem correlação espacial)
- `gamma = 3.76` — expoente típico de ambiente urbano denso
- Compatível ponto a ponto com FLoRa e ns-3 para validação cruzada

**Perda por distância (sem shadowing):**

| d      | PL (dB) | RSSI (TX=14dBm, ganhos=3dB) |
|--------|---------|------------------------------|
| 100 m  | 83 dB   | −66 dBm                      |
| 500 m  | 103 dB  | −86 dBm                      |
| 1 km   | 116 dB  | −99 dBm                      |
| 2 km   | 127 dB  | −110 dBm                     |
| 5 km   | 140 dB  | −123 dBm                     |
| 10 km  | 151 dB  | −134 dBm                     |

---

### 8.2 `correlated_shadowing`

**Equivalente:** ns-3 `CorrelatedShadowingPropagationLossModel`

```python
d0    = 1.0
PL_d0 = 7.7
gamma = 3.76

# Shadowing do mapa espacialmente correlacionado (CorrelatedShadowingMap)
shadowing = self.shadowing_map.get_shadowing(device_x, device_y)

path_loss = PL_d0 + 10 * gamma * np.log10(distance / d0) + shadowing
```

**Diferença do log_normal padrão:**
- O shadowing é **determinístico para uma posição** — transmissões repetidas do mesmo dispositivo obtêm o mesmo shadowing
- Dispositivos próximos têm shadowing correlacionado (compartilham obstáculos similares)
- Ver [Seção 9](#9-correlatedshadowingmap) para implementação do mapa

---

### 8.3 `okumura_hata`

**Fonte:** Okumura-Hata — modelo empírico para macro células urbanas (150–1500 MHz)

```python
CF = (1.1 * np.log10(frequency_mhz) - 0.7) * self.hr_m \
   - (1.56 * np.log10(frequency_mhz) - 0.8)

path_loss = (
    69.55
    + 26.16 * np.log10(frequency_mhz)
    - 13.82 * np.log10(self.ht_m)
    - CF
    + (44.9 - 6.55 * np.log10(self.ht_m)) * np.log10(distance_km)
)
```

**Parâmetros da simulação:**
- `ht_m = 1.5 m` (altura ED)
- `hr_m = 30 m` (altura GW)
- `frequency_mhz ≈ 868 MHz` (EU868)

**Aplicabilidade:** 150–1500 MHz, distâncias 1–20 km, macro célula urbana.

---

### 8.4 `fspl`

**Fonte:** Friis Free-Space Path Loss

```python
path_loss = (20 * np.log10(distance_km)
           + 20 * np.log10(frequency_mhz)
           + 32.45)
```

**Expansão:**
```
FSPL [dB] = 20·log10(d[km]) + 20·log10(f[MHz]) + 32.45
          = 20·log10(4π d f / c)
          ≈ 20·log10(d) + 20·log10(f) + 20·log10(4π/c) + ajuste unidades
```

**Nota:** Subestima significativamente a perda em ambientes reais. Útil apenas para limite inferior teórico.

---

### 8.5 `log_distance`

```python
path_loss_exponent = 3.5

path_loss = (20 * np.log10(frequency_mhz)
           + 10 * path_loss_exponent * np.log10(distance_km)
           - 28)
```

Versão sem shadowing estocástico — totalmente determinístico. `gamma = 3.5` é intermediário entre urbano (3.76) e suburbano (3.0).

---

### 8.6 `cost_hata`

**Fonte:** COST-231 Hata — extensão para 1500–2000 MHz

```python
CF = (1.1 * np.log10(frequency_mhz) - 0.7) * self.hr_m \
   - (1.56 * np.log10(frequency_mhz) - 0.8)

path_loss = (
    46.3
    + 33.9 * np.log10(frequency_mhz)
    - 13.82 * np.log10(self.ht_m)
    - CF
    + (44.9 - 6.55 * np.log10(self.ht_m)) * np.log10(distance_km)
)
```

Difere do Okumura-Hata nos coeficientes iniciais (`69.55→46.3`, `26.16→33.9`).
Usado como base para o modelo `fading`.

---

### 8.7 `building_penetration`

```python
# Base: log_normal_shadowing
path_loss = PL_d0 + 10 * gamma * np.log10(distance / d0) + X_sigma

# Adiciona BPL se dispositivo é indoor
bpl = np.random.lognormal(mean=np.log(20), sigma=0.5)
path_loss += bpl
```

Ver [Seção 10](#10-building-penetration-cache-g3) — na prática, o BPL é **pré-calculado e cacheado** por dispositivo em `_building_penetration_cache`, não recalculado aqui. Este modelo é o alternativo de path loss completo; a versão em produção usa `get_building_penetration()` separado.

---

### 8.8 `fading`

```python
# Base: cost_hata
base_loss = self.pathloss(distance, frequency_mhz, "cost_hata")

if fading_type == "rayleigh":
    fading_db = 20 * np.log10(max(np.random.rayleigh(scale=1), 1e-10))

elif fading_type == "rician":
    K = 3
    sample = max(np.sqrt(np.random.noncentral_chi2(2, 2*K) / (2*(1+K))), 1e-10)
    fading_db = 20 * np.log10(sample)

elif fading_type == "nakagami":
    m = 3
    sample = max(np.sqrt(np.random.gamma(shape=m, scale=1.0/m)), 1e-10)
    fading_db = 20 * np.log10(sample)

else:
    fading_db = 0

path_loss = base_loss - fading_db   # fading REDUZ a perda (multipath construtivo)
```

**Distribuições de Fading:**

| Tipo      | Parâmetros | Cenário físico                         | Efeito médio   |
|-----------|-----------|----------------------------------------|----------------|
| Rayleigh  | scale=1   | Sem linha de visada (NLOS), muitos multipath | −2.5 dB |
| Rician    | K=3       | Com linha de visada dominante (LOS)    | −1.0 dB aprox. |
| Nakagami  | m=3       | Generalização — moderado LOS           | −0.5 dB aprox. |

> `fading_db` pode ser negativo (perda por fading destrutivo) ou positivo (ganho por fading construtivo). O sinal final é `path_loss = base - fading_db`.

---

### 8.9 `oulu`

**Fonte:** LoRaSim (Bor et al. 2016), FLoRa original

```python
d0    = 40.0       # distância de referência (metros)
PL_d0 = 127.41     # perda a d0 = 40m (dB)
gamma = 2.08       # expoente de propagação (medições em Oulu, Finlândia)
sigma = 3.57       # desvio padrão shadowing (dB)

X_sigma = np.random.normal(0, sigma)

path_loss = PL_d0 + 10 * gamma * np.log10(max(distance / d0, 1e-10)) + X_sigma
```

**Peculiaridades:**
- `d0 = 40m` (incomum — a maioria usa 1m ou 10m)
- `PL_d0 = 127.41 dB` — perda de referência muito alta (absorve efeitos de near-field)
- `gamma = 2.08` — expoente baixo (propagação em área aberta nórdica)
- A combinação `PL_d0 + gamma` produz resultados mais pessimistas que log_normal para distâncias curtas, similar para distâncias médias

**Comparação log_normal vs oulu (sem shadowing):**

| d      | log_normal (γ=3.76) | oulu (γ=2.08, d0=40m) |
|--------|--------------------|-----------------------|
| 100 m  | 83 dB              | 121 dB                |
| 500 m  | 103 dB             | 134 dB                |
| 2 km   | 127 dB             | 141 dB                |
| 5 km   | 140 dB             | 147 dB                |

---

### Resumo Comparativo dos 9 Modelos

| Modelo              | Tipo           | Shadowing    | Base  | Cenário alvo      | Padrão? |
|---------------------|----------------|--------------|-------|-------------------|---------|
| `log_normal_shadowing` | Estatístico | Independente | 1m   | Urbano geral      | ✓       |
| `correlated_shadowing` | Estatístico | Correlacionado | 1m | Urbano realista   |         |
| `okumura_hata`      | Empírico       | Nenhum       | 1km  | Macro urbano      |         |
| `fspl`              | Determinístico | Nenhum       | 1km  | Espaço livre      |         |
| `log_distance`      | Determinístico | Nenhum       | 1km  | Genérico          |         |
| `cost_hata`         | Empírico       | Nenhum       | 1km  | Suburbano/rural   |         |
| `building_penetration` | Estatístico | Independente + BPL | 1m | Indoor/outdoor |      |
| `fading`            | Estatístico    | + Rayleigh/Rician/Nakagami | cost_hata | Multipath |   |
| `oulu`              | Estatístico    | Independente | 40m  | LoRaSim/FLoRa     |         |

---

## 9. CorrelatedShadowingMap

**Arquivo:** `network.py` — classe `CorrelatedShadowingMap`

### Motivação

O modelo `log_normal_shadowing` usa amostras independentes por transmissão — fisicamente incorreto, pois o shadowing é causado por obstáculos fixos (edifícios, relevo). Um dispositivo numa rua estreita sempre terá shadowing alto, independente do instante.

O `CorrelatedShadowingMap` resolve isso: **mesma posição → mesmo shadowing**, e **posições próximas → shadowing similar**.

### Inicialização

```python
class CorrelatedShadowingMap:
    def __init__(self, area_size, grid_step=50, sigma=3.57):
        self.area_size   = area_size      # 10000 m
        self.grid_step   = grid_step      # resolução: 50 m
        self.grid_points = int(area_size / grid_step) + 1   # 201 × 201

        # Grid de amostras gaussianas independentes
        self.grid = np.random.normal(0, sigma, (self.grid_points, self.grid_points))
        # Cada nó (i,j) recebe shadowing ~ N(0, 3.57 dB)
```

**Tamanho do grid:** 201 × 201 = 40.401 nós para área 10km × 10km com passo 50m.

### Interpolação Bilinear

Para qualquer posição `(x, y)` dentro da área:

```python
def get_shadowing(self, x, y):
    # Clampa coordenadas para dentro da área
    x = max(0.0, min(float(self.area_size), float(x)))
    y = max(0.0, min(float(self.area_size), float(y)))

    # Posição fracionária no grid
    gx, gy = x / self.grid_step, y / self.grid_step

    # Nós vizinhos
    x0, y0 = int(gx),  int(gy)
    x1, y1 = min(x0+1, self.grid_points-1), min(y0+1, self.grid_points-1)

    # Frações de interpolação
    dx, dy = gx - x0, gy - y0

    # Valores nos 4 nós vizinhos
    v00 = self.grid[x0, y0]   # inferior-esquerdo
    v10 = self.grid[x1, y0]   # inferior-direito
    v01 = self.grid[x0, y1]   # superior-esquerdo
    v11 = self.grid[x1, y1]   # superior-direito

    # Interpolação bilinear
    return (1-dx)*(1-dy)*v00 + dx*(1-dy)*v10 + (1-dx)*dy*v01 + dx*dy*v11
```

### Fórmula da Interpolação Bilinear

```
S(x,y) = (1−dx)(1−dy)·S(x0,y0)
        + dx(1−dy)·S(x1,y0)
        + (1−dx)dy·S(x0,y1)
        + dx·dy·S(x1,y1)
```

**Visualização:**
```
(x0,y1) ──────── (x1,y1)
   |          ↑          |
   |     (x,y)           |
   |          ↓          |
(x0,y0) ──────── (x1,y0)
```

### Propriedades Resultantes

| Propriedade              | Valor                          |
|--------------------------|--------------------------------|
| Distribuição             | ~N(0, 3.57 dB)                 |
| Distância de correlação  | ~50 m (= grid_step)           |
| Dispositivos em mesma célula | Shadowing idêntico          |
| Dispositivos a 50–100 m  | Shadowing moderadamente correlacionado |
| Dispositivos a > 200 m   | Praticamente independentes     |
| Equivalência             | ns-3 CorrelatedShadowingPropagationLossModel |

**Vantagem sobre ns-3:** A implementação do ns-3 usa correlação exponencial contínua `C(d) = exp(-d/d_corr)`. A interpolação bilinear é uma aproximação que produz resultados similares com custo O(1) por consulta.

---

## 10. Building Penetration Cache (G3)

**Arquivo:** `network.py`

### Design de Cache

```python
# Em __init__:
self._building_penetration_cache = {}

# Em initialize_devices():
for device in devices:
    if device.is_indoor:
        # Calcula BPL uma vez e armazena
        bpl = np.random.lognormal(mean=np.log(20), sigma=0.5)
        self._building_penetration_cache[device.device_id] = bpl

# Em get_building_penetration():
def get_building_penetration(self, device):
    if device.is_indoor:
        return self._building_penetration_cache.get(device.device_id, 20.0)
    return 0.0
```

**Motivação do cache:** Se o BPL fosse amostrado a cada transmissão, um dispositivo indoor teria perda variável — fisicamente incorreto. O prédio é um obstáculo fixo. O cache garante que cada dispositivo tenha uma perda de penetração **constante** durante toda a simulação.

### Distribuição LogNormal do BPL

```
BPL ~ LogNormal(μ=ln(20), σ=0.5)

Parâmetros da lognormal em unidade natural:
  E[BPL] = exp(μ + σ²/2) = exp(ln(20) + 0.125) = 20 × exp(0.125) ≈ 22.5 dB
  Mediana = exp(μ) = 20 dB
  P10 ≈ 12 dB
  P90 ≈ 38 dB
```

| Percentil | BPL (dB) |
|-----------|---------|
| P10       | ~12 dB  |
| P25       | ~15 dB  |
| P50 (mediana) | 20 dB |
| P75       | ~27 dB  |
| P90       | ~38 dB  |

### Proporção de Dispositivos Indoor

```python
indoor_ratio = 0.3   # 30% indoor (padrão)
device.is_indoor = (random.random() < indoor_ratio)
```

30% indoor é um valor típico para cenários IoT urbanos (baseado em estudos de campo).

---

## 11. Link Budget — Integração Completa

### Equação Completa

```
RSSI [dBm] = P_tx + G_ed + G_gw − PL − BPL

onde:
  P_tx = potência TX (14 dBm padrão)
  G_ed = ganho antena ED (0 dBi — chip antenna)
  G_gw = ganho antena GW (3 dBi — omnidirecional)
  PL   = path_loss(modelo, distância, freq)
  BPL  = building_penetration (0 ou ~20 dB lognormal)
```

### Margens e Limites

```
Margem de cobertura = RSSI − gw_sensitivity[SF, BW]

Para recepção:   RSSI > gw_sensitivity[SF, BW]   (limiar de sensibilidade)
Para demodulação: SNR > snr_min[SF]              (limiar de SNR)
Para sem colisão: energy_ratio_dB ≥ threshold    (capture effect)
```

### Exemplo Completo (SF9, BW=125kHz, d=3.2km, indoor)

```
P_tx   = 14 dBm
G_ed   = 0 dBi
G_gw   = 3 dBi
PL     = log_normal_shadowing(3200m, 868.1MHz)
       = 7.7 + 10 × 3.76 × log10(3200) + N(0, 3.57)
       = 7.7 + 37.6 × 3.505 + ~0
       = 7.7 + 131.8 + 0 = 139.5 dB  (sem shadowing)
BPL    = 20 dB (indoor, mediana)

RSSI = 14 + 0 + 3 − 139.5 − 20 = −142.5 dBm

gw_sensitivity[SF9, 125kHz] = −135.0 dBm

Margem = −142.5 − (−135.0) = −7.5 dB  → ABAIXO DO LIMIAR (sem cobertura)

Com SF12:  gw_sensitivity = −142.5 dBm
Margem = −142.5 − (−142.5) = 0 dB  → No limite

→ Dispositivo indoor a 3.2km só tem cobertura com SF12 (e apenas marginalmente)
```

---

## 12. Interação Channel ↔ Gateway ↔ Network

### Diagrama de Sequência por Transmissão

```
EndDevice                Gateway              ChannelModel           Network
    │                        │                      │                    │
    │──── create Packet ─────►│                      │                    │
    │                        │                      │                    │
    │               process_uplink(pkt)             │                    │
    │                        │──pathloss(d,f,m)────────────────────────►│
    │                        │◄─────────────────── PL ──────────────────│
    │                        │──get_building_penetration(dev)──────────►│
    │                        │◄─────────────────── BPL ─────────────────│
    │                        │                      │                    │
    │                        │  [RSSI calculado]    │                    │
    │                        │                      │                    │
    │                        │──── iterate on_air ─►│                    │
    │                        │◄── interference data ─│                    │
    │                        │                      │                    │
    │                        │  [SINR calculado]    │                    │
    │                        │                      │                    │
    │◄─── pkt.rssi/.sinr ────│                      │                    │
    │                        │                      │                    │
    │──────────────────── add_transmission(pkt) ───►│                    │
    │                                               │                    │
    │     ... airtime ...                           │                    │
    │                                               │                    │
    │──────────────────── evaluate_reception(pkt) ─►│                    │
    │                        │                      │──noise_floor(bw)──►│
    │                        │                      │◄── NF ─────────────│
    │                        │                      │                    │
    │                        │                      │  [SNR check]       │
    │                        │                      │  [preamble lock]   │
    │                        │                      │  [capture effect]  │
    │                        │                      │                    │
    │◄─────────────────── collided T/F ─────────────│                    │
    │                                               │                    │
    │──────────────────── cleanup_expired(t) ───────►│                    │
```

### Dependências em Runtime

```
evaluate_reception(packet, gateway)
  └── network.calculate_noise_floor(packet.bw)     ← chamada ao Network
         └── parametors.{k, temperatura, noise_figure}

evaluate_reception busca:
  └── parametors.interference_model               ← lido em runtime
  └── parametors.interference_matrix              ← ou interference_matrix_goursaud
  └── parametors.snr_min_per_sf

process_uplink(packet) busca:
  └── network.pathloss(d, f, m, x, y)
  └── network.get_building_penetration(device)
  └── network.channel.on_air                       ← para interferência G14
  └── network.calculate_noise_floor(bw)
```

---

## 13. Análise de Sensibilidade dos Parâmetros

### Parâmetros com Maior Impacto no PDR

| Parâmetro              | Variação       | Impacto no PDR |
|------------------------|----------------|----------------|
| `interference_model`   | semtech→goursaud | −4 a −6%      |
| `model_pathloss`       | fspl→okumura   | ±15%           |
| `gamma` (log_normal)   | 3.0→4.0        | −10%           |
| `sigma` (shadowing)    | 0→5 dB         | −5%            |
| `indoor_ratio`         | 0%→50%         | −5 a −15%      |
| `correlated_shadowing` | False→True     | ±2% (mais realista) |
| `noise_figure`         | 4→8 dB         | −2 a −5%       |
| `gw_antenna_gain`      | 0→6 dBi        | +5%            |

### Interação gamma × Co-SF Threshold

```
gamma alto (>4.0) → distâncias maiores têm RSSI muito baixo
                 → mais pacotes abaixo da sensibilidade GW
                 → menos colisões (menos EDs em cobertura)
                 → PDR pode aumentar paradoxalmente

gamma baixo (<3.0) → todos os EDs têm boa cobertura
                  → mais colisões
                  → PDR diminui
```

### Efeito do Shadowing (sigma) no PDR

| sigma | Cobertura | Colisões | PDR estimado |
|-------|-----------|----------|-------------|
| 0 dB  | Determinística | Médias | ~85%       |
| 2 dB  | Ligeira variação | Leve variação | ~87% |
| 3.57 dB | Padrão FLoRa/ns-3 | Normal | ~90%   |
| 6 dB  | Alta variação | Alta variação | ~88%   |
| 10 dB | Muito variável | Alta | ~80%         |

---

## 14. Exemplos Numéricos Completos

### Exemplo 1: Recepção sem Interferência (SF7)

```
Dispositivo: SF7, BW=125kHz, tx_power=14dBm, outdoor, d=800m
Modelo: log_normal_shadowing (gamma=3.76, sigma=0, para exemplo limpo)

PL = 7.7 + 10 × 3.76 × log10(800) = 7.7 + 37.6 × 2.903 = 7.7 + 109.1 = 116.8 dB
BPL = 0 (outdoor)
RSSI = 14 + 0 + 3 − 116.8 = −99.8 dBm

noise_floor (BW=125kHz) = −116.9 dBm
SNR = −99.8 − (−116.9) = 17.1 dB
SNR_min[SF7] = −7.5 dB → 17.1 ≥ −7.5 ✓

Sem interferentes → SUCESSO
packet.collided = False
SINR ≈ SNR = 17.1 dB (sem interferência)
```

### Exemplo 2: Colisão por SNR Insuficiente (SF12)

```
Dispositivo: SF12, BW=125kHz, tx_power=14dBm, indoor, d=8000m
Modelo: log_normal_shadowing

PL_base = 7.7 + 10 × 3.76 × log10(8000) = 7.7 + 37.6 × 3.903 = 7.7 + 146.8 = 154.5 dB
BPL = 20 dB (indoor, mediana)
RSSI = 14 + 0 + 3 − 154.5 − 20 = −157.5 dBm

noise_floor (BW=125kHz) = −116.9 dBm
SNR = −157.5 − (−116.9) = −40.6 dB
SNR_min[SF12] = −20.0 dB → −40.6 < −20.0 ✗

→ Falha no Passo 1 → packet.collided = True (sem precisar avaliar interferentes)
```

### Exemplo 3: Capture Effect — Co-SF Sobrevive (Semtech)

```
Pacote alvo:      SF9, BW=125kHz, RSSI=−105 dBm, rectime=144ms
Interferente A:   SF9 (co-SF),  RSSI=−110 dBm, overlap=100ms, tx_start antes do lock

Passo 1: SNR = −105 − (−116.9) = 11.9 dB ≥ −12.5 ✓
Passo 2: Tsym = 512/125000 = 4.096ms
         preamble_lock_time = tx_start + 6 × 4.096ms = tx_start + 24.6ms
         Interferente A.tx_start < lock → avaliado ✓

Passo 4 (Semtech, co-SF):
  sf_target_idx   = 12 − 9 = 3
  sf_inter_idx    = 12 − 9 = 3
  threshold_dB    = matrix[3][3] = +1 dB

  energy_correction = 10 × log10(144 / 100) = 10 × 0.158 = 1.58 dB
  energy_ratio_dB   = (−105 − (−110)) + 1.58 = 5 + 1.58 = 6.58 dB

  6.58 dB ≥ 1 dB → SOBREVIVE ✓
  packet.collided = False
```

### Exemplo 4: Capture Effect — Co-SF Falha (Goursaud)

```
Mesmas condições do Exemplo 3, mas com matriz Goursaud:

  threshold_dB = goursaud_matrix[3][3] = +6 dB

  energy_ratio_dB = 6.58 dB

  6.58 dB ≥ 6 dB → SOBREVIVE ✓ (margem de apenas 0.58 dB)

Variação: Interferente B mais forte, RSSI=−106 dBm:
  energy_ratio_dB = (−105 − (−106)) + 1.58 = 1 + 1.58 = 2.58 dB
  2.58 dB < 6 dB → COLISÃO ✗ (Goursaud é mais conservador)
  Semtech: 2.58 dB ≥ 1 dB → SOBREVIVE ✓
```

### Exemplo 5: Cross-SF — SF9 vs SF12

```
Pacote alvo:      SF9,  RSSI=−108 dBm, rectime=144ms
Interferente:     SF12, RSSI=−100 dBm (mais forte!), overlap=50ms

Passo 4 (Semtech, cross-SF):
  sf_target_idx   = 12 − 9  = 3
  sf_inter_idx    = 12 − 12 = 0
  threshold_dB    = matrix[3][0] = −15 dB

  energy_correction = 10 × log10(144 / 50) = 10 × 0.459 = 4.59 dB
  energy_ratio_dB   = (−108 − (−100)) + 4.59 = −8 + 4.59 = −3.41 dB

  −3.41 dB ≥ −15 dB → SOBREVIVE ✓
  (Mesmo sendo 8 dBm mais fraco que o SF12, o pacote SF9 sobrevive
   porque SFs diferentes são quasi-ortogonais)
```

### Exemplo 6: Preamble Lock — Interferente Tardio Ignorado

```
Pacote alvo: SF9, tx_start=100.0s, rectime=144ms
  preamble_lock_time = 100.0 + 24.6ms = 100.0246s

Interferente A: tx_start = 100.015s (ANTES do lock de 100.025s) → AVALIADO
Interferente B: tx_start = 100.030s (APÓS  o lock de 100.025s) → IGNORADO
Interferente C: tx_start = 100.050s (muito depois)              → IGNORADO

→ Apenas Interferente A entra na avaliação de capture effect.
```

---

*Documentação gerada a partir do código-fonte — Sprint 7 concluído.*
