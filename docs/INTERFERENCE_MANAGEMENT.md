# PyLoRaWAN — Interference Management

> Documentação técnica completa do gerenciamento de interferência.
> Cobre todos os mecanismos de interferência, detecção, modelagem e mitigação
> implementados no simulador: co-SF, cross-SF, DL-UL, GW saturation,
> duty cycle, ADR, LR-FHSS/ACRDA e MRC.

---

## Sumário

1. [Taxonomia da Interferência no Simulador](#1-taxonomia-da-interferência-no-simulador)
2. [Interferência UL-UL — CSS](#2-interferência-ul-ul--css)
   - 2.1 [Modelo de Energia Acumulada (G14)](#21-modelo-de-energia-acumulada-g14)
   - 2.2 [Co-SF Interference](#22-co-sf-interference)
   - 2.3 [Cross-SF Interference](#23-cross-sf-interference)
   - 2.4 [Preamble Locking (G13)](#24-preamble-locking-g13)
3. [Matrizes de Interferência](#3-matrizes-de-interferência)
   - 3.1 [Semtech AN1200.18](#31-semtech-an120018)
   - 3.2 [Goursaud (ns-3)](#32-goursaud-ns-3)
   - 3.3 [Comportamento por Zona de SINR](#33-comportamento-por-zona-de-sinr)
4. [SINR — Cálculo Detalhado](#4-sinr--cálculo-detalhado)
5. [Interferência DL-UL (G8)](#5-interferência-dl-ul-g8)
6. [Gateway Saturation — Interferência por Capacidade](#6-gateway-saturation--interferência-por-capacidade)
7. [Interferência LR-FHSS](#7-interferência-lr-fhss)
   - 7.1 [Colisão por Fragmento](#71-colisão-por-fragmento)
   - 7.2 [ACRDA/SIC — Cancelamento Iterativo](#72-acrdastic--cancelamento-iterativo)
8. [Mecanismos de Mitigação](#8-mecanismos-de-mitigação)
   - 8.1 [Duty Cycle](#81-duty-cycle)
   - 8.2 [ADR — Adaptive Data Rate](#82-adr--adaptive-data-rate)
   - 8.3 [Frequency Hopping e Multi-canal](#83-frequency-hopping-e-multi-canal)
   - 8.4 [Building Penetration como Isolante](#84-building-penetration-como-isolante)
9. [MRC — Diversidade de Recepção](#9-mrc--diversidade-de-recepção)
10. [Interação entre Fontes de Interferência](#10-interação-entre-fontes-de-interferência)
11. [Fluxo Completo de Avaliação de Interferência](#11-fluxo-completo-de-avaliação-de-interferência)
12. [Estatísticas e Rastreamento](#12-estatísticas-e-rastreamento)
13. [Análise Quantitativa: Contribuição por Fonte](#13-análise-quantitativa-contribuição-por-fonte)
14. [Exemplos Numéricos](#14-exemplos-numéricos)

---

## 1. Taxonomia da Interferência no Simulador

O simulador modela **quatro tipos independentes** de interferência:

```
┌─────────────────────────────────────────────────────────────────┐
│                    FONTES DE INTERFERÊNCIA                      │
├──────────────────┬──────────────────┬───────────────────────────┤
│  UL-UL (CSS)     │  DL-UL           │  GW Saturation            │
│  channel.py      │  gateway.py      │  gateway.py               │
│                  │                  │                           │
│  Co-SF:          │  GW em TX DL     │  Todos os 8 paths         │
│  mesma SF/freq   │  bloqueia RX UL  │  SX1301 ocupados          │
│                  │  na mesma freq   │                           │
│  Cross-SF:       │                  │                           │
│  SF diferentes,  │                  │                           │
│  mesma freq      │                  │                           │
├──────────────────┴──────────────────┴───────────────────────────┤
│  LR-FHSS Fragment Collision                                     │
│  lrfhss.py — colisão por canal de hopping + janela temporal     │
└─────────────────────────────────────────────────────────────────┘

MITIGAÇÃO:
  Duty Cycle    → espaça transmissões no tempo
  ADR           → reduz SF/power de EDs com boa cobertura
  Multi-canal   → distribui carga entre 3 (EU868) ou 64 (US915) canais
  ACRDA/SIC     → recupera pacotes LR-FHSS via cancelamento iterativo
  MRC           → combina recepções de múltiplos GWs
```

### Responsabilidade por Arquivo

| Arquivo        | Tipo de Interferência                            |
|----------------|--------------------------------------------------|
| `channel.py`   | UL-UL CSS: capture effect, preamble lock, SINR  |
| `gateway.py`   | DL-UL (G8), GW saturation (8 paths)             |
| `lrfhss.py`    | LR-FHSS fragment collision, ACRDA/SIC           |
| `parametors.py`| Matrizes Semtech/Goursaud, SNR_min, noise figure|
| `network.py`   | SINR acumulado por SF, coordenação geral         |
| `network_server/components/adr.py` | ADR como mitigação ativa   |

---

## 2. Interferência UL-UL — CSS

### 2.1 Modelo de Energia Acumulada (G14)

Sprint 6 substituiu o modelo simplificado de comparação de RSSI pelo **modelo de energia acumulada** do `ns-3 LoraInterferenceHelper`.

#### Motivação Física

O modelo antigo comparava apenas RSSI instantâneo: se `RSSI_sinal > RSSI_interferente`, sobrevive. Mas isso ignora a **duração** de cada sobreposição.

Um interferente que sobrepõe apenas 10% do pacote alvo acumula apenas 10% da sua energia de interferência. A razão correta é:

```
Razão de energia = Energia_sinal / Energia_interferente
                 = (P_sinal × T_pacote) / (P_interferente × T_overlap)
                 = (P_sinal / P_interferente) × (T_pacote / T_overlap)
```

#### Fórmula Implementada

```python
# channel.py — evaluate_reception()

packet_duration = packet.rectime   # T_pacote

for interferer, overlap_duration, _ in interferers:
    # Índices: SF12→0, SF7→5
    sf_target_idx = 12 - packet.sf
    sf_inter_idx  = 12 - interferer.sf
    threshold_dB  = matrix[sf_target_idx][sf_inter_idx]

    energy_correction_dB = 10.0 * math.log10(
        packet_duration / max(overlap_duration, 1e-9)
    )

    energy_ratio_dB = (packet.rssi - interferer.rssi) + energy_correction_dB

    if energy_ratio_dB < threshold_dB:
        survived = False
        break
```

#### Correção de Energia por Razão de Sobreposição

| T_overlap / T_pacote | energy_correction_dB | Interpretação                          |
|----------------------|----------------------|----------------------------------------|
| 100% (total)         | 0.0 dB               | Interferência completa — só RSSI conta |
| 75%                  | +1.2 dB              | Leve vantagem ao sinal                 |
| 50%                  | +3.0 dB              | Sinal favorecido                       |
| 25%                  | +6.0 dB              | Sinal muito favorecido                 |
| 10%                  | +10.0 dB             | Interferente quase irrelevante         |
| 5%                   | +13.0 dB             | Interferente marginal                  |
| 1%                   | +20.0 dB             | Interferente desprezível               |

A correção protege pacotes longos (SF alto) contra interferentes curtos que chegam no final da transmissão.

---

### 2.2 Co-SF Interference

Dois pacotes com **mesmo SF** e **mesma frequência** competem pelo mesmo receptor. Esta é a forma mais frequente de colisão no LoRaWAN.

#### Condições de Co-SF Collision

```
1. Mesma frequência (packet.freq == interferer.freq)
2. Mesmo SF (sf_target == sf_interferer → diagonal da matriz)
3. Sobreposição temporal > 0
4. Interferente iniciou ANTES do preamble lock
5. energy_ratio_dB < threshold_co_SF
```

#### Thresholds Co-SF

| Matriz    | Threshold (todos os SF) | Interpretação                              |
|-----------|------------------------|---------------------------------------------|
| Semtech   | +1 dB                  | Dominar por 1 dB é suficiente para capturar |
| Goursaud  | +6 dB                  | Precisar dominar por 6 dB                   |

#### Probabilidade de Co-SF com 50 EDs

Com 3 canais EU868 e SFs aleatórios (SF7–SF12, distribuição uniforme):
- Prob. de mesmo canal: 1/3
- Prob. de mesmo SF: 1/6
- Prob. de mesmo canal + SF: 1/18 ≈ 5.6% por par de EDs
- Com 50 EDs: C(50,2) = 1225 pares → ~68 pares com colisão potencial por transmissão

Mas apenas pares com sobreposição temporal contam. Com λ=1/300s e ToA~144ms (SF9):
- Janela de vulnerabilidade: 144ms / 300s ≈ 0.05%
- Colisões efetivas: ≪ 1% dos pacotes por horário

---

### 2.3 Cross-SF Interference

Pacotes com **SFs diferentes** na mesma frequência. A ortogonalidade dos SFs LoRa fornece isolamento natural, mas não perfeito.

#### Por que SFs são quasi-ortogonais

Cada SF usa uma taxa de chirp diferente (`2^SF`). O receptor correlaciona com a taxa correta, obtendo ganho de processamento. Um sinal SF diferente aparece como ruído parcialmente correlacionado — não como sinal, mas com energia residual.

#### Thresholds Cross-SF (Semtech) — Exemplos Chave

| Alvo \ Interferente | SF7     | SF9     | SF12    |
|---------------------|---------|---------|---------|
| **SF7**             | +1 dB   | −9 dB   | −9 dB   |
| **SF9**             | −15 dB  | +1 dB   | −15 dB  |
| **SF12**            | −25 dB  | −25 dB  | +1 dB   |

Leitura: `SF9 alvo vs SF7 interferente = −15 dB`
→ O SF9 sobrevive mesmo se o SF7 for **15 dB mais forte**.
→ SFs diferentes oferecem excelente isolamento (−9 a −25 dB).

#### Cross-SF na Prática

Na grande maioria dos cenários, cross-SF não causa colisão. Um SF7 precisaria ser **mais de 9 dB mais forte** que o SF9 para perturbá-lo — situação incomum a não ser que o ED SF7 esteja muito próximo do GW e o ED SF9 muito longe.

---

### 2.4 Preamble Locking (G13)

```python
# channel.py — evaluate_reception()

Tsym = (2.0 ** packet.sf) / packet.bw
preamble_lock_time = tx_start + 6 * Tsym

for (other_pkt, other_start, other_end) in self.on_air:
    ...
    # Interferente chegou DEPOIS do lock → não conta
    if other_start >= preamble_lock_time:
        continue
```

#### Mecanismo Físico

O receptor LoRa executa detecção de preâmbulo nos primeiros **6 símbolos**. Uma vez sincronizado, o receptor está "trancado" no sinal alvo e pode tolerar interferência posterior (até o limiar de SINR mínimo para demodulação).

Interferentes que chegam após o lock:
- Aparecem como ruído ao detector já sincronizado
- Contribuem para o SINR calculado no gateway, mas **não entram na lógica de capture effect**
- Efeito prático: reduz colisões para transmissões com início tardio

#### Impacto por SF

| SF  | Lock Time | Janela vulnerável |
|-----|-----------|-------------------|
| SF7 | 6.1 ms    | Muito curta — poucos interferentes "chegam antes" |
| SF9 | 24.6 ms   | Moderada           |
| SF12| 196.6 ms  | Longa — exposta por ~200ms antes de travar |

SF12 é o mais vulnerável: qualquer interferente que inicie durante os ~197ms de preâmbulo é avaliado. Para λ=1/300s com 50 EDs, há probabilidade não negligível de sobreposição nessa janela.

---

## 3. Matrizes de Interferência

### 3.1 Semtech AN1200.18

**Padrão do simulador** (`interference_model = "semtech"`).

```
         SF12  SF11  SF10   SF9   SF8   SF7   ← SF interferente
SF12  [  +1,  -23,  -24,  -25,  -25,  -25 ]
SF11  [ -20,   +1,  -20,  -21,  -22,  -22 ]
SF10  [ -18,  -17,   +1,  -17,  -18,  -19 ]
SF9   [ -15,  -14,  -13,   +1,  -13,  -15 ]
SF8   [ -13,  -13,  -12,  -11,   +1,  -11 ]
SF7   [  -9,   -9,   -9,   -9,   -8,   +1 ]
↑ SF alvo
```

**Acesso em código:**
```python
threshold = interference_matrix[12 - sf_alvo][12 - sf_interferente]
```

**Características:**
- Co-SF: **+1 dB** — capture suave, mais colisões co-SF resolvidas pelo sinal mais forte
- Cross-SF simétrico nas extremidades: SF12 alvo é tolerante (−23 a −25), SF7 alvo é sensível (−8 a −9)
- Gradiente suave ao longo da diagonal

---

### 3.2 Goursaud (ns-3)

**Alternativo** (`interference_model = "goursaud"`).

```
         SF12  SF11  SF10   SF9   SF8   SF7   ← SF interferente
SF12  [  +6,  -36,  -36,  -36,  -36,  -36 ]
SF11  [ -29,   +6,  -33,  -33,  -33,  -33 ]
SF10  [ -28,  -26,   +6,  -30,  -30,  -30 ]
SF9   [ -27,  -25,  -23,   +6,  -27,  -27 ]
SF8   [ -24,  -22,  -20,  -19,   +6,  -24 ]
SF7   [ -20,  -16,  -18,  -19,  -16,   +6 ]
↑ SF alvo
```

**Características:**
- Co-SF: **+6 dB** — mais conservador; colisões co-SF mais frequentes
- Cross-SF: thresholds muito menores (−16 a −36) — SFs são mais ortogonais
- Matriz não-simétrica para cross-SF (off-diagonal): `matrix[i][j] ≠ matrix[j][i]`

---

### 3.3 Comportamento por Zona de SINR

#### Zona 1: Sinal Forte (SINR alto)

```
energy_ratio_dB >> threshold → sobrevive com folga
Região: RSSI >> −120 dBm, distância curta (< 1km)
Resultado: PDR ≈ 100%, colisões raras
```

#### Zona 2: Limiar de Captura (SINR próximo do threshold)

```
energy_ratio_dB ≈ threshold (±2 dB)
Região: RSSI ≈ −125 a −130 dBm, distância média
Resultado: Resultado estocástico — depende de shadowing e timing exato
           Maior diferença Semtech vs Goursaud (co-SF)
```

#### Zona 3: Sinal Fraco (SINR baixo)

```
energy_ratio_dB << threshold → colisão garantida
Região: RSSI << −130 dBm, distância longa (> 5km)
Resultado: Colisão por SNR insuficiente (Passo 1 de evaluate_reception)
           Matrizes não chegam a ser consultadas
```

---

## 4. SINR — Cálculo Detalhado

O SINR é calculado no `gateway.process_uplink()` e armazenado no Packet **antes** da avaliação de colisão em `channel.evaluate_reception()`.

### Acumulação de Interferência por SF

```python
# gateway.py — process_uplink()

interference_power_linear = 0
interference_per_sf = {}          # rastreamento por SF para análise

for (other_pkt, tx_start, tx_end) in network.channel.on_air:
    if other_pkt.packet_id == packet.packet_id: continue
    if other_pkt.freq != packet.freq: continue          # mesma frequência

    # Janela temporal de sobreposição
    overlap_start    = max(current_time, tx_start)
    overlap_end      = min(current_time + packet.rectime, tx_end)
    overlap_duration = overlap_end - overlap_start
    if overlap_duration <= 0: continue

    # Peso proporcional ao tempo de sobreposição
    overlap_ratio = overlap_duration / packet.rectime

    # RSSI do interferente no mesmo gateway
    other_rssi_dBm    = (other_pkt.tx_power
                         + ed_antenna_gain + gw_antenna_gain
                         - pathloss(other_device))
    other_rssi_linear = 10 ** (other_rssi_dBm / 10)

    # Acumula interferência linear (ponderada pelo overlap)
    interference_power_linear += other_rssi_linear * overlap_ratio

    # Agrupa por SF para diagnóstico
    interference_per_sf[other_pkt.sf] = (
        interference_per_sf.get(other_pkt.sf, 0)
        + other_rssi_linear * overlap_ratio
    )

packet.interference_per_sf = interference_per_sf
```

### Cálculo Final do SINR

```python
noise_linear  = 10 ** (packet.noise_floor / 10)   # ruído térmico
signal_linear = 10 ** (packet.rssi / 10)           # sinal

# SINR = S / (I + N)
sinr_linear = signal_linear / (interference_power_linear + noise_linear)
packet.sinr = 10 * np.log10(sinr_linear)

# SNR = S / N  (sem interferência)
packet.snr  = packet.rssi - packet.noise_floor

# SIR = S / I  (sem ruído)
if interference_power_linear > 0:
    packet.sir = packet.rssi - 10 * np.log10(interference_power_linear)
else:
    packet.sir = float('inf')
```

### Relação entre SNR, SIR e SINR

```
SINR_dB ≤ min(SNR_dB, SIR_dB)

Cenários:
  I >> N:  SINR ≈ SIR   (interferência domina sobre ruído)
  N >> I:  SINR ≈ SNR   (ruído domina — sem interferência)
  I ≈ N:   SINR ≈ SNR − 3 dB  (ambos contribuem igualmente)
```

### Por que SINR é calculado no Gateway e não no ChannelModel?

O gateway tem acesso ao `channel.on_air` e pode calcular o RSSI de cada interferente via `pathloss()`. O `ChannelModel` só tem os pacotes — sem posição dos dispositivos. A divisão de responsabilidades é:

- **Gateway**: calcula métricas físicas (RSSI, SINR, path_loss)
- **ChannelModel**: aplica a lógica de decisão (sobrevive/colide)

---

## 5. Interferência DL-UL (G8)

**Arquivo:** `gateway.py`

Sprint 4 implementou o bloqueio de half-duplex no gateway durante transmissão de downlink.

### Mecanismo

```python
# Gateway — rastreia quando cada frequência está ocupada em DL
self.dl_busy_until = {}   # {freq_MHz → tx_end_time}

# Ao transmitir downlink (em _on_rx1_open / _on_rx2_open):
gateway.dl_busy_until[dl_freq] = current_time + dl_airtime

# Em process_uplink() — verificação no início:
if self.dl_busy_until.get(packet.freq, 0) > current_time:
    packet.collided = True
    return   # UL bloqueado pela transmissão DL ativa
```

### Propriedades

| Propriedade          | Detalhe                                             |
|----------------------|-----------------------------------------------------|
| **Escopo**           | Por frequência — só bloqueia a frequência do DL ativo |
| **Duração**          | `dl_airtime` = função do SF do downlink             |
| **Direção**          | Apenas GW→ED bloqueia ED→GW (half-duplex)           |
| **Impacto no PDR**   | ~0.5–1% de perda adicional no cenário referência    |
| **Físico?**          | Sim — SX1301 não pode receber e transmitir ao mesmo tempo |

### Quantificação do Impacto

Para um DL típico (SF9 downlink, airtime=144ms, duty cycle RX1=1%):
- Bloqueio por DL: 144ms na frequência usada
- Taxa de DLs: ~10% dos pacotes são confirmados → ~5 DLs/hora (1 GW)
- Total de bloqueio por hora: 5 × 144ms ≈ 720ms em 3600s ≈ **0.02%** do tempo
- Probabilidade de colisão DL-UL: negligível no cenário de referência

---

## 6. Gateway Saturation — Interferência por Capacidade

**Arquivo:** `gateway.py` — `try_allocate_path()`

### Modelo SX1301

O chip SX1301 possui **8 demoduladores paralelos**. Quando todos estão ocupados, novos pacotes são descartados independentemente do RSSI ou da colisão.

```python
def try_allocate_path(self, packet, current_time):
    tx_end = current_time + packet.rectime
    for path in self.reception_paths:
        if path.is_free(current_time):
            path.assign(packet, tx_end)
            return True     # path disponível
    self.saturation_events += 1
    return False            # saturação → pacote perdido (colisão forçada)
```

### Quando a Saturação Ocorre

```
Ocupação simultânea de 8 paths:
  Com 50 EDs e ToA médio 144ms (SF9):
    E[paths_busy] = λ_aggregate × ToA_avg
                  = (50/300) × 0.144
                  ≈ 0.024 paths em média

  Prob. de saturação (Erlang-B, 8 paths, ρ=0.024):
    ≈ extremamente baixa (<< 0.01%) no cenário de referência
```

A saturação só se torna relevante em cenários com:
- Muitos EDs (> 200) com SF alto (SF11, SF12)
- Alta taxa λ ou payloads grandes
- Múltiplos EDs próximos sincronizados (improvável com Poisson)

### Distinção: Saturação vs. Colisão

| Tipo          | Causa                         | Detecção         | Efeito no Packet      |
|---------------|-------------------------------|------------------|-----------------------|
| Colisão SINR  | Interferência supera threshold | `channel.evaluate_reception()` | `packet.collided = True` |
| Saturação GW  | Todos os 8 paths ocupados     | `try_allocate_path()` | `packet.collided = True` |
| DL-UL block   | GW em TX DL na mesma freq     | `dl_busy_until` check | `packet.collided = True` |

Nos três casos, `packet.collided = True`, mas a causa é diferente. O `saturation_events` rastreia especificamente o terceiro caso.

---

## 7. Interferência LR-FHSS

**Arquivo:** `lrfhss.py`

O LR-FHSS usa um modelo de interferência fundamentalmente diferente do CSS: **a colisão é avaliada por fragmento individual**, não por pacote inteiro.

### 7.1 Colisão por Fragmento

```python
# LRFHSS_Channel.check_fragment_collisions()

def check_fragment_collisions(self, fragments):
    for frag in fragments:
        for (other, other_start, other_end) in self.active_fragments:
            if other.packet_id == frag.packet_id: continue  # mesmo pacote
            if other.channel != frag.channel: continue      # canal diferente → ortogonal
            # Verifica sobreposição temporal
            if frag.tx_end <= other_start or frag.tx_start >= other_end:
                continue
            frag.collided = True   # colisão deste fragmento
            break
```

#### Condições de Colisão de Fragmento

```
1. Pertence a pacote diferente (other.packet_id ≠ frag.packet_id)
2. Mesmo canal de hopping (other.channel == frag.channel)
3. Sobreposição temporal: NOT (frag.tx_end ≤ other_start OR frag.tx_start ≥ other_end)
```

#### Por que Fragmentos são Mais Resistentes

Um pacote CSS de SF9 ocupa 1 canal por 144ms.
Um fragmento LR-FHSS ocupa 1 canal por ~100ms (payload) ou ~233ms (header).

Com 35 canais OBW, a probabilidade de dois fragmentos coincidirem no mesmo canal **e** ao mesmo tempo cai dramaticamente:

```
P(colisão fragmento) = P(mesmo canal) × P(sobreposição temporal)
                     ≈ (1/35) × (fragment_duration / interval)
                     ≈ (1/35) × (0.1 / 300) ≈ 0.001%
```

Comparado ao CSS, onde dois pacotes na mesma frequência têm P(sobreposição) proporcional ao ToA.

### Critério de Decodificação Parcial

```python
# LRFHSS_Channel.evaluate_packet()

def evaluate_packet(self, fragments, threshold):
    h_success = sum(1 for f in fragments
                    if f.frag_type == 'header' and not f.collided)
    p_success = sum(1 for f in fragments
                    if f.frag_type == 'payload' and not f.collided)

    return h_success >= 1 and p_success >= threshold
```

Para CR=1/3, PL=20B: threshold=4, n_payloads=12.
O pacote tolera até **8 fragmentos de payload colididindo** (de 12) e ainda é decodificado.

### 7.2 ACRDA/SIC — Cancelamento Iterativo

**Arquivo:** `lrfhss.py` — classe `ACRDA`

O ACRDA (Asynchronous Coded Random Access with Diversity Aloha) aplica Successive Interference Cancellation pós-simulação para recuperar pacotes adicionais.

#### Algoritmo SIC Completo

```python
# ACRDA.process_window()

def process_window(self, channel):
    new_recovery = True
    iterations   = 0

    while new_recovery:
        new_recovery = False
        iterations  += 1

        for packet_id, fragments in self.packet_fragments.items():
            if packet_id in self.decoded_packets: continue

            h_ok  = Σ(frag.frag_type=='header'  and not frag.collided)
            p_ok  = Σ(frag.frag_type=='payload' and not frag.collided)
            n_pay = Σ(frag.frag_type=='payload')
            threshold = max(1, n_pay // 3)

            if h_ok >= 1 and p_ok >= threshold:
                self.decoded_packets.add(packet_id)
                self._cancel_interference(packet_id, channel)
                new_recovery = True   # pode haver mais pacotes recuperáveis

    return len(self.decoded_packets), iterations
```

#### Cancelamento de Interferência

```python
# ACRDA._cancel_interference()

def _cancel_interference(self, decoded_packet_id, channel):
    decoded_frags = self.packet_fragments[decoded_packet_id]

    for dec_frag in decoded_frags:
        for other_id, other_frags in self.packet_fragments.items():
            if other_id == decoded_packet_id: continue
            if other_id in self.decoded_packets: continue

            for other_frag in other_frags:
                if not other_frag.collided: continue
                if other_frag.channel != dec_frag.channel: continue
                if no_temporal_overlap(dec_frag, other_frag): continue

                # O fragmento de other_frag estava colidindo com dec_frag
                # Como dec_frag foi decodificado, sua interferência é removida
                other_frag.collided = False   # interferência cancelada!
```

#### Convergência do SIC

```
Iteração 0: Pacotes com nenhum fragmento colidido → decodificados imediatamente
Iteração 1: Pacotes cujas colisões eram APENAS com pacotes da iteração 0
            → interferência cancelada → alguns tornam-se decodificáveis
Iteração 2: Pacotes cujas colisões eram com pacotes das iterações 0 e 1
...
Convergência: quando nenhuma nova decodificação ocorre
```

**Efeito prático** (cenário referência, 50 EDs, 50% LR-FHSS):

| Fase              | Pacotes decodificados |
|-------------------|-----------------------|
| Sem ACRDA         | ~88–92%               |
| Após iteração 1   | ~96%                  |
| Convergência      | ~99–100%              |

---

## 8. Mecanismos de Mitigação

### 8.1 Duty Cycle

O duty cycle é a principal barreira contra interferência por sobrecarga de canal.

#### Implementação por Dispositivo

```python
# network.py — _on_device_send()

# Calcula limite de DC da sub-banda (EU868)
dc_limit = region.get_duty_cycle_limit(device.freq)

# Calcula release time
airtime = device.calculate_airtime()
device.dc_release_time = current_time + airtime / dc_limit

# Verifica antes de transmitir
if current_time < device.dc_release_time:
    # Agenda para depois do release
    delay = device.dc_release_time - current_time
    scheduler.schedule(delay, DEVICE_SEND, ...)
    return
```

#### Sub-bandas EU868 (G7)

```python
EU868.duty_cycle = {
    (863.0, 868.6): 0.01,    # G/G1 — canais 868.1/868.3/868.5 → 1%
    (868.7, 869.2): 0.001,   # G2 → 0.1%
    (869.4, 869.65): 0.10,   # G3 → 10% (canal RX2: 869.525 MHz)
    (869.7, 870.0): 0.01,    # G4 → 1%
}
```

#### Off-time Mínimo por SF (DC=1%, EU868)

```
off_time = airtime × (1/DC − 1) = airtime × 99

SF7  (36ms):   off_time ≈ 3.56s   → máx ~17 pkts/minuto
SF9  (144ms):  off_time ≈ 14.3s   → máx ~4 pkts/minuto
SF12 (1155ms): off_time ≈ 114.4s  → máx ~0.5 pkts/minuto
```

#### Duty Cycle no Gateway (Downlink)

```python
# gateway_manager.py — can_send_downlink()

dc_limit = rx1_dc if window == "rx1" else rx2_dc   # 1% ou 10%
off_time = airtime / dc_limit - airtime
return current_time >= last_tx_time[window] + off_time
```

| Janela | DC limite | off_time (SF9, 144ms) |
|--------|-----------|-----------------------|
| RX1    | 1%        | 14.3s entre DLs       |
| RX2    | 10%       | 1.3s entre DLs        |

#### Impacto do Duty Cycle na Interferência

O duty cycle garante que cada ED transmite no máximo 36 s/hora (SF7, 1%) ou 11.6 s/hora (SF12, 1%). Isso **limita a densidade de transmissões simultâneas** e é a razão pela qual cenários EU868 têm PDR relativamente alto (>85%) mesmo sem ADR.

---

### 8.2 ADR — Adaptive Data Rate

O ADR reduz a interferência co-SF ao mover EDs com boa cobertura para SFs menores, liberando SFs altos para EDs distantes.

#### Algoritmo ADR (adr.py)

```python
# 1. Acumula histórico de SNR (20 últimos pacotes)
history = deque(maxlen=ADR_HISTORY_SIZE)   # ADR_HISTORY_SIZE = 20

# 2. Computa métrica SNR
snr_metric = compute_snr_metric(history)   # média, máx, mín, percentil, ewma

# 3. Calcula margem
required_snr = snr_min_per_sf[SF]         # ex: SF9 → -12.5 dB
margin = snr_metric - required_snr - ADR_SNR_MARGIN_DB  # margin_db = 10 dB
n_steps = int(margin / ADR_STEP_DB)       # ADR_STEP_DB = 3 dB

# 4. Aplica ajustes
Passo 1: n_steps > 0 e SF > 7  → SF--        (reduz SF)
Passo 2: n_steps > 0 e power > 2 → power -= 2  (reduz TX power)
Passo 3: n_steps < 0 e power < 14 → power += 2 (aumenta TX power)
Passo 4: n_steps < 0 e SF < 12 → SF++        (aumenta SF)

# 5. Gera LinkAdrReq se mudou
if new_sf != old_sf or new_power != old_power:
    return [LinkAdrReq(dr=12-new_sf, tx_power=new_power)]
```

#### Como ADR Reduz Interferência

| Cenário               | Sem ADR                      | Com ADR                         |
|-----------------------|------------------------------|----------------------------------|
| ED próximo (100m)     | SF9 (aleatório) → ToA=144ms  | SF7 (ADR) → ToA=36ms           |
| ED médio (2km)        | SF9 mantido                  | SF9 mantido (margem ≈ 0)        |
| ED distante (8km)     | SF9 (aleatório) → cobertura ruim | SF12 → melhor cobertura     |
| Colisões co-SF9       | Alto (muitos EDs em SF9)     | Baixo (EDs distribuídos)        |

#### ADR Backoff

```python
ADR_ACK_LIMIT = 64   # frames sem DL antes de ativar pedido
ADR_ACK_DELAY = 32   # frames adicionais antes do backoff

# Se adr_ack_cnt >= ADR_ACK_LIMIT + ADR_ACK_DELAY:
# → aumenta TX power (até 14 dBm), depois SF (até SF12)
# Evita que EDs fiquem presos em configuração ADR ruim
```

#### ADR e Interferência: Efeito Colateral

ADR pode **aumentar** interferência em certos cenários:
- EDs muito próximos com SF7 têm ToA curto mas transmitem mais vezes por hora (menor off-time)
- Se muitos EDs convergirem para SF7, a densidade no canal SF7 aumenta
- O ADR_SNR_MARGIN_DB = 10 dB garante folga suficiente para compensar

---

### 8.3 Frequency Hopping e Multi-canal

```python
# enddevice.py — select_channel()

def select_channel(self):
    if self._available_channels:
        self.freq = random.choice(self._available_channels)
    return self.freq
```

O frequency hopping aleatoriza qual canal é usado em cada transmissão, distribuindo a carga de interferência.

#### EU868 — 3 canais default

```python
frequency_mhz = [868.1, 868.3, 868.5]   # todos em sub-banda G/G1
```

Com 3 canais e 50 EDs, cada canal recebe em média ~17 EDs. A probabilidade de colisão co-SF por canal cai por fator 3 comparado a canal único.

#### Expansão com Canais Adicionais (G7)

```python
EU868.additional_channels = [867.1, 867.3, 867.5, 867.7, 867.9]   # 5 canais extras
# Total: 8 canais → carga dividida por 8 (vs. 3)
# Colisões co-SF: redução de ~62%
```

#### US915 — 64 canais UL + 8 DL

```python
US915.default_channels = [902.3 + 0.2 * i for i in range(64)]   # 64 canais!
# Interferência co-canal quase impossível com <100 EDs
```

---

### 8.4 Building Penetration como Isolante

O building penetration (G3) afeta a interferência indiretamente:

```
Device outdoor → RSSI alto → maior probabilidade de interferir com outros
Device indoor  → RSSI baixo (−20 dB adicional) → menor poder de interferência
```

Um ED indoor a 500m pode ter RSSI = −86 − 20 = **−106 dBm** vs. outdoor a mesmo local = −86 dBm.

O diferencial de 20 dB faz com que um ED indoor seja um interferente muito menos agressivo. Para colisão co-SF (Semtech, threshold=1 dB):

```
ED outdoor: RSSI = -86 dBm, interfere com EDs com RSSI < −85 dBm
ED indoor:  RSSI = -106 dBm, interfere com EDs com RSSI < −105 dBm
→ ED indoor "alcança" muito menos EDs como interferente efetivo
```

---

## 9. MRC — Diversidade de Recepção

**Arquivo:** `channel.py` — uso de `packet.snr_mrc`

O Maximal Ratio Combining (MRC) combina recepções do mesmo pacote por múltiplos gateways para melhorar o SNR efetivo — contramedida direta à interferência.

### Princípio

```
SNR_MRC = Σ SNR_i_linear   [soma em linear]
         = Σ 10^(SNR_i / 10)

SNR_MRC_dB = 10 × log10(SNR_MRC_linear)
```

Para dois GWs com SNR1 = 10 dB e SNR2 = 7 dB:
```
SNR_MRC = 10^(10/10) + 10^(7/10) = 10 + 5.01 = 15.01
SNR_MRC_dB = 10 × log10(15.01) = 11.76 dB
(ganho de 1.76 dB sobre o melhor GW individual)
```

### Integração com evaluate_reception

```python
# channel.py — evaluate_reception()

# Sprint 9: usa SNR combinado se disponível
if getattr(packet, 'snr_mrc', None) is not None:
    snr = packet.snr_mrc          # SNR combinado (melhor)
else:
    snr = packet.rssi - noise_floor   # SNR de único GW

snr_required = snr_min_per_sf.get(packet.sf, -20.0)
if snr < snr_required:
    packet.collided = True
    return False
```

O `snr_mrc` é calculado externamente (função MRC no `network.py`) antes de `evaluate_reception()` ser chamado.

### Impacto na Interferência

MRC não elimina interferência, mas aumenta o SNR do sinal alvo, melhorando a razão energia no capture effect:

```
Sem MRC: energy_ratio = RSSI_GW1 - RSSI_interferer + correction
Com MRC: energia do sinal é efetivamente maior → capture effect mais fácil de satisfazer
```

---

## 10. Interação entre Fontes de Interferência

### Hierarquia de Verificações

```
process_uplink(packet) [gateway.py]
  │
  ├── [1] DL-UL check (G8)
  │       if dl_busy_until[freq] > t: colided=True; return
  │
  ├── [2] GW saturation check
  │       if try_allocate_path() == False: colided=True; return
  │
  ├── [3] Cálculo SINR (acumula I por SF)
  │       interference_per_sf, sinr, snr, sir calculados
  │
  └── packet.rssi/.snr/.sinr armazenados
            │
            ▼
evaluate_reception(packet) [channel.py]
  │
  ├── [4] SNR vs. SNR_min[SF]
  │       if snr < snr_min: colided=True; return False
  │
  ├── [5] Preamble locking (G13)
  │       filtra interferentes tardios
  │
  ├── [6] Capture effect por interferente (G14)
  │       for each interferer:
  │         if energy_ratio < threshold: survived=False; break
  │
  └── packet.collided = not survived
```

**Ordem de falha** (mais comum → menos comum):
1. SNR insuficiente (EDs fora de cobertura ou muito indoor)
2. Co-SF capture effect falha (EDs próximos com mesmo SF)
3. DL-UL block (raríssimo — apenas durante DL ativo)
4. GW saturation (apenas em alta densidade)

### Casos onde Múltiplas Interferências Interagem

#### Indoor + Longue Range + Co-SF

```
ED1: indoor (BPL=20dB), d=4km, SF12 → RSSI ≈ -140 dBm
ED2: outdoor, d=0.5km, SF12 → RSSI ≈ -90 dBm
Ambos transmitem ao mesmo tempo, mesma frequência

SNR[ED1] = -140 - (-116.9) = -23.1 dB
SNR_min[SF12] = -20.0 dB
-23.1 < -20.0 → ED1 falha no Passo 1 (nem chega à matriz)
→ ED1 não causa colisão para ED2 (abaixo da sensibilidade)
```

#### Dois EDs Distantes — Cross-SF não interfere

```
ED1: d=6km, SF12, RSSI=-133 dBm  (no limiar de sensibilidade)
ED2: d=1km, SF7,  RSSI=-100 dBm  (sinal forte)
Mesma frequência, sobreposição parcial (50%)

Avaliando ED1 (alvo SF12 vs interferente SF7):
  energy_correction = 10*log10(1/0.5) = 3 dB
  energy_ratio = (-133 - (-100)) + 3 = -33 + 3 = -30 dB
  threshold[SF12 alvo, SF7 interferer] = -25 dB (Semtech)
  -30 < -25 → ED1 COLIDE (SF7 forte demais para SF12 fraco)

  Com Goursaud: threshold = -36 dB
  -30 > -36 → ED1 SOBREVIVE
```

---

## 11. Fluxo Completo de Avaliação de Interferência

### Diagrama de Decisão Detalhado

```
                    PACKET CHEGA AO GATEWAY
                             │
                    ┌────────▼────────┐
                    │  DL-UL Check    │
                    │  (G8)           │
                    └────────┬────────┘
                 dl_busy?    │     não dl_busy
                    ├──YES───┘           │
              collided=True              │
                    │            ┌───────▼──────────┐
                    │            │  Path Allocation  │
                    │            │  (SX1301 8 paths) │
                    │            └───────┬───────────┘
                    │          saturado? │  path livre
                    │            ├──YES──┘     │
                    │      collided=True        │
                    │            │       ┌──────▼──────────────────────┐
                    │            │       │  Link Budget + SINR         │
                    │            │       │  RSSI, interference_per_sf  │
                    │            │       │  SINR = S/(I+N)             │
                    │            │       └──────┬──────────────────────┘
                    │            │              │
                    │            │       ┌──────▼──────────────────────┐
                    │            │       │  evaluate_reception()       │
                    │            │       │                             │
                    │            │       │  ① SNR ≥ SNR_min[SF]?      │
                    │            │       │    NO → collided=True       │
                    │            │       │                             │
                    │            │       │  ② Preamble lock filter     │
                    │            │       │    exclui tardios           │
                    │            │       │                             │
                    │            │       │  ③ Seleciona matriz         │
                    │            │       │    Semtech ou Goursaud      │
                    │            │       │                             │
                    │            │       │  ④ Para cada interferente:  │
                    │            │       │    energy_ratio ≥ threshold?│
                    │            │       │    NO → collided=True; break│
                    │            │       │                             │
                    │            │       └──────┬──────────────────────┘
                    │            │              │
                    └────────────┴──────────────┤
                                                │
                                      packet.collided = True/False
                                      packet.rssi, snr, sinr populados
```

---

## 12. Estatísticas e Rastreamento

### Por ChannelModel

```python
channel.total_collisions   # colisões CSS detectadas em evaluate_reception()
channel.total_receptions   # recepções CSS bem-sucedidas
channel.stats() → {
    "total_collisions": int,
    "total_receptions": int,
    "on_air_count": int,
}
```

### Por Packet

```python
packet.collided          # bool — colidiu?
packet.received          # bool — recebido com sucesso?
packet.sinr              # float — SINR dB
packet.snr               # float — SNR dB
packet.sir               # float — SIR dB (ou inf se sem I)
packet.interference_per_sf  # dict {SF → potência linear acumulada}
```

### Por Gateway

```python
gateway.saturation_events  # vezes que todos os 8 paths estavam ocupados
gateway.total_dl_sent      # total de downlinks enviados
gateway.active_paths_count(t)  # paths em uso no instante t
```

### Por LRFHSS_Channel

```python
lrfhss_channel.total_collisions   # fragmentos colididos
lrfhss_channel.total_receptions   # pacotes LR-FHSS decodificados
lrfhss_channel.active_fragments   # fragmentos on-air
```

### Por ACRDA

```python
acrda.decoded_packets   # set de UUIDs recuperados pelo SIC
# process_window() retorna (n_decoded, n_iterations)
```

### Métricas Agregadas (analytics.compute_metrics)

```python
{
    "performance": {
        "pdr_percent": float,
        "pdr_per_sf": {sf: pdr},        # PDR por SF — mostra onde co-SF é pior
        "collisions": int,
        "collision_rate_percent": float,
    },
    "radio": {
        "avg_sinr_db": float,
        "min_sinr_db": float,
        "p5_sinr_db": float,            # 5º percentil — piores casos
    }
}
```

---

## 13. Análise Quantitativa: Contribuição por Fonte

### Cenário de Referência (50 EDs, 1 GW, 10km, 1h, EU868)

| Fonte de Interferência    | Colisões estimadas | % do total | Acionado por              |
|---------------------------|-------------------|------------|---------------------------|
| Co-SF (mesma SF+freq)     | ~55–65%           | Principal  | SF aleatório + mesmo canal|
| SNR insuficiente (cobertura)| ~25–35%         | Secundário | EDs distantes/indoor      |
| Cross-SF                  | < 5%              | Menor      | SF muito diferente        |
| DL-UL (G8)                | < 1%              | Negligível | Poucos DLs com 1 GW       |
| GW saturation             | < 0.1%            | Irrelevante| Cenário ref. tem baixa carga |

### Efeito do Modelo de Matriz

| Matriz    | Colisões co-SF | Colisões cross-SF | PDR total |
|-----------|----------------|-------------------|-----------|
| Semtech   | ~8%            | < 1%              | ~92%      |
| Goursaud  | ~13%           | < 0.5%            | ~87%      |

### Efeito do Shadowing na Interferência

```
σ = 0 dB:    RSSI determinístico → capture effect previsível → PDR ~85%
σ = 3.57 dB: Alguns EDs têm RSSI alto (fading positivo) → capturam mais → PDR ~90%
σ = 7 dB:    Alta variância → alguns EDs muito bons, outros muito ruins → PDR ~85%
```

Paradoxo: **shadowing moderado melhora PDR** porque favorece naturalmente os pacotes "vencedores" do capture effect.

### Impacto do ADR na Interferência

Com ADR habilitado (cenário padrão):
- EDs próximos movem-se para SF7 → ToA curto → off-time curto → mais transmissões
- EDs distantes ficam em SF11/SF12 → poucos pacotes mas maior janela de vulnerabilidade
- Resultado líquido: **PDR aumenta ~3–5%** por redistribuição mais eficiente de SFs

---

## 14. Exemplos Numéricos

### Exemplo 1: Co-SF com Semtech — Colisão por Margem Pequena

```
Alvo:        SF9, freq=868.1, RSSI=−112 dBm, rectime=144ms, tx_start=100.0s
Interferente: SF9, freq=868.1, RSSI=−110 dBm, tx_start=100.010s
             overlap=[100.010, 100.144] = 134ms

Passo 1: SNR = -112 - (-116.9) = 4.9 dB ≥ -12.5 ✓
Passo 2: Tsym=4.096ms, lock=100+24.6ms=100.0246s
         Interferente.tx_start=100.010 < 100.025 → avaliado ✓

Passo 4 (Semtech):
  sf_target_idx = 12-9 = 3,  sf_inter_idx = 12-9 = 3
  threshold = matrix[3][3] = +1 dB

  energy_correction = 10*log10(144 / 134) = 10*log10(1.075) = 0.31 dB
  energy_ratio = (-112 - (-110)) + 0.31 = -2 + 0.31 = -1.69 dB

  -1.69 < +1 → COLISÃO ✗
  (alvo é 2 dBm mais fraco que interferente → captura fails)
```

### Exemplo 2: Co-SF com Semtech — Sobrevive por Timing

```
Mesmos parâmetros, mas interferente chega mais tarde:
  Interferente.tx_start = 100.120s (apenas 24ms de sobreposição no final)
  overlap = [100.120, 100.144] = 24ms

Passo 2: lock = 100.0246s
         Interferente.tx_start = 100.120 > 100.025 → IGNORADO (chegou após lock)

Sem interferentes válidos → SUCESSO ✓
```

### Exemplo 3: Cross-SF — SF7 forte não derruba SF12 fraco (Semtech)

```
Alvo:         SF12, RSSI=−135 dBm (no limiar), rectime=1155ms
Interferente: SF7,  RSSI=−108 dBm, overlap=500ms

Passo 4 (Semtech):
  sf_target_idx = 12-12 = 0,  sf_inter_idx = 12-7 = 5
  threshold = matrix[0][5] = -25 dB

  energy_correction = 10*log10(1155/500) = 10*log10(2.31) = 3.64 dB
  energy_ratio = (-135 - (-108)) + 3.64 = -27 + 3.64 = -23.36 dB

  -23.36 > -25 → SOBREVIVE ✓
  (SF12 tolera SF7 mesmo 27 dBm mais forte, graças à ortogonalidade)
```

### Exemplo 4: SINR acumulado de múltiplos interferentes

```
Alvo: SF9, RSSI=-105 dBm (signal=3.16e-11 mW)
Interferente A: SF9, overlap=100% → RSSI=-112 dBm, ratio=1.0 → I_A=6.31e-12 mW
Interferente B: SF7, overlap=50%  → RSSI=-108 dBm, ratio=0.5 → I_B=0.5*1.58e-11=7.94e-12 mW
noise_floor: -116.9 dBm → N=2.04e-12 mW

Total I+N = 6.31e-12 + 7.94e-12 + 2.04e-12 = 1.629e-11 mW
SINR = 3.16e-11 / 1.629e-11 = 1.94 linear
SINR_dB = 10*log10(1.94) = 2.88 dB

Agora evaluate_reception avalia cada interferente separadamente:
  Interferente A (co-SF):
    energy_correction = 10*log10(1.0/1.0) = 0 dB
    energy_ratio = (-105 - (-112)) + 0 = 7 dB
    threshold (Semtech co-SF) = +1 dB → 7 ≥ 1 ✓ sobrevive A

  Interferente B (cross-SF SF7):
    energy_correction = 10*log10(1.0/0.5) = 3 dB
    energy_ratio = (-105 - (-108)) + 3 = 3 + 3 = 6 dB
    threshold (Semtech SF9 vs SF7) = -15 dB → 6 ≥ -15 ✓ sobrevive B

RESULTADO: packet.collided = False  ✓
(SINR baixo em absoluto, mas cada interferente avaliado individualmente passa)
```

### Exemplo 5: DL-UL Block

```
t=500.0s: GW envia DL na freq 868.1 MHz, SF9 (airtime=144ms)
           gateway.dl_busy_until[868.1] = 500.0 + 0.144 = 500.144s

t=500.050s: ED envia UL na freq 868.1
             process_uplink() verifica: dl_busy_until[868.1]=500.144 > 500.050
             → packet.collided = True
             → ED não sabe que colidiu (sem ACK na RX1/RX2 → retransmite)
```

### Exemplo 6: LR-FHSS + ACRDA recupera pacote

```
t=0: Pacote P1 (15 fragmentos, threshold=4):
     Fragmentos 1-12: canal 7,3,21,8,15,2,29,11,4,18,6,9 (únicos)
     Fragmentos 13-15: canais 7,3,4 (colisão com P2 no canal 7 e 3)
     Após check_fragment_collisions: frag 13 e 14 colididos

     Antes ACRDA: h_ok=1, p_ok=10 (≥4=threshold) → DECODIFICADO ✓
     ACRDA não precisa intervir.

Cenário adverso: threshold=8 (CR=1/2), 5 fragmentos de payload colididos:
     p_ok = 7 < 8 → NÃO decodificado ainda

     Pacote P2 é decodificado primeiro (nenhuma colisão):
     _cancel_interference(P2):
       → remove interferência de P2 nos fragmentos colididos de P1
       → 3 dos 5 fragmentos de payload de P1 eram colididos com P2 → liberados
       → p_ok = 7 + 3 = 10 ≥ 8 → P1 agora DECODIFICADO ✓
```

---

*Documentação gerada a partir do código-fonte — Sprint 7 concluído.*
