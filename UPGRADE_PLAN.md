# PyLoRaWAN Simulator - Plano de Atualizacao e Melhorias

**Objetivo:** Tornar o PyLoRaWAN o simulador LoRaWAN mais completo entre os cinco frameworks analisados (FLoRa, LoRaWANSim, ns-3 LoRaWAN, PyLoRaWAN, LR-FHSS-sim), mantendo a facilidade de uso do Python.

**Data:** 05/03/2026
**Baseado na analise de:** `/home/igor/flora/simulations/FRAMEWORKS_COMPARACAO.md` + codigo-fonte completo

---

## Indice

1. [Diagnostico Atual](#1-diagnostico-atual)
2. [Bugs Criticos a Corrigir](#2-bugs-criticos-a-corrigir)
3. [Fase 1 - Fundamentos (PHY + Canal)](#3-fase-1---fundamentos-phy--canal)
4. [Fase 2 - MAC e Protocolo](#4-fase-2---mac-e-protocolo)
5. [Fase 3 - Network Server e ADR](#5-fase-3---network-server-e-adr)
6. [Fase 4 - Energia e Bateria](#6-fase-4---energia-e-bateria)
7. [Fase 5 - Parametros Regionais](#7-fase-5---parametros-regionais)
8. [Fase 6 - LR-FHSS (Diferencial Unico)](#8-fase-6---lr-fhss-diferencial-unico)
9. [Fase 7 - Deployment e Topologia](#9-fase-7---deployment-e-topologia)
10. [Fase 8 - Saida e Analise](#10-fase-8---saida-e-analise)
11. [Fase 9 - Performance e Escalabilidade](#11-fase-9---performance-e-escalabilidade)
12. [Fase 10 - Validacao, Testes e Infraestrutura](#12-fase-10---validacao-testes-e-infraestrutura)
13. [Matriz de Superacao por Framework](#13-matriz-de-superacao-por-framework)
14. [Prioridades e Roadmap](#14-prioridades-e-roadmap)

---

## 1. Diagnostico Atual

### 1.1 O que ja funciona bem

| Componente | Estado | Notas |
|------------|--------|-------|
| Event Scheduler | Solido | heapq com lazy cancellation, funcional |
| ToA Calculation | Correto | Formula Semtech AN1200.13 compativel |
| 5 modelos de path loss | Funcional | Okumura-Hata, Log-Distance, FSPL, COST-Hata, Fading |
| 4 modelos de mobilidade | Funcional | Random Walk, Gauss-Markov, Levy Walk, Matrix |
| ADR basico | Funcional | Baseado em ns-3 AdrComponent |
| Interference matrix | Correto | Semtech AN1200.18, 6x6 (SF7-SF12) |
| Channel model (SINR) | Funcional | Capture effect com matriz |
| Energy model | Basico | Maquina de estados TX/RX/SLEEP/STANDBY |
| Seed reproducibilidade | Implementado | `random_seed = 42` |
| Duty cycle (ED) | Basico | 1% uplink |

### 1.2 Gaps criticos vs outros simuladores

| Gap | FLoRa tem? | ns-3 tem? | LoRaWANSim tem? | Impacto |
|-----|-----------|----------|-----------------|---------|
| SINR completo (S/(I+N)) | Sim | Sim | Sim | **Alto** - PDR incorreto |
| Multi-canal (EU868: 8 ch) | Sim | Sim | Sim | **Alto** - colisoes infladas |
| Gateway 8 reception paths | Sim | Sim | Sim | **Alto** - capacity errada |
| Log-Normal Shadowing | Sim | Sim (correlated) | Sim | **Alto** - path loss irrealista |
| Regional parameters | Sim | Sim (EU/US/AU/AS) | Parcial | **Alto** - validacao |
| Network Server | Sim | Sim | N/A | **Medio** - arquitetura |
| Classe B/C corretas | N/A | N/A | Parcial | **Medio** - feature |
| ADR backoff | Sim | Sim | Sim | **Medio** - robustez |
| MAC commands completos | Sim | Sim | N/A | **Medio** - fidelidade |
| Duty cycle DL (GW) | N/A | Sim | Sim | **Medio** - DL realista |
| Battery depletion | Parcial | Sim | N/A | **Baixo** - lifetime |
| LR-FHSS | N/A | N/A | N/A | **Diferencial** |

### 1.3 Arquivos atuais e LOC

| Arquivo | LOC | Funcao |
|---------|-----|--------|
| `parametors.py` | 65 | Configuracao global |
| `packet.py` | 52 | Classe Packet |
| `enddevice.py` | 317 | End Device com mobilidade |
| `gateway.py` | 131 | Gateway basico |
| `channel.py` | 119 | Modelo de canal |
| `protocolos.py` | 153 | MAC + ADR |
| `energymodel.py` | 143 | Modelo de energia |
| `event_scheduler.py` | 122 | Scheduler de eventos |
| `packettracker.py` | 142 | Rastreamento de pacotes |
| `throughput.py` | 103 | Calculo de vazao |
| `network.py` | 701 | Orquestrador principal |
| **Total** | **~2048** | |

---

## 2. Bugs Criticos a Corrigir

### BUG-01: Gateway calcula interferencia de TODOS os devices (sem verificar quem esta transmitindo)

**Arquivo:** `gateway.py:43-51`
**Problema:** O `process_uplink()` soma a potencia de TODOS os outros devices na mesma frequencia como interferentes, mesmo que nao estejam transmitindo naquele momento. Isso infla a interferencia e reduz o SIR artificialmente.
**Correcao:** Verificar quais devices estao on-air no canal model antes de somar interferencia.

```python
# ANTES (incorreto):
for other in self.network.devices:
    if other.device_id == device.device_id or other.freq != device.freq:
        continue
    # Soma TODOS como interferentes

# DEPOIS (correto):
current_time = packet.arrival_time
for (other_pkt, tx_start, tx_end) in self.network.channel.on_air:
    if other_pkt.device_id == device.device_id:
        continue
    if other_pkt.freq != device.freq:
        continue
    if tx_end <= current_time or tx_start >= current_time + packet.rectime:
        continue  # Nao sobreposicao temporal
    # Calcula interferencia apenas de pacotes realmente on-air
```

### BUG-02: SINR nunca e calculado corretamente

**Arquivo:** `gateway.py:54-56`, `channel.py:47`
**Problema:** O gateway calcula SIR (Signal/Interference) e SNR (Signal/Noise) separadamente, mas nunca calcula SINR = S / (I + N). O channel model usa apenas SNR para decisao de recepcao.
**Correcao:** Calcular SINR combinando interferencia e ruido.

```python
# SINR correto:
noise_linear = 10 ** (noise_floor / 10)
sinr_linear = signal_linear / (interference_linear + noise_linear)
packet.sinr = 10 * np.log10(sinr_linear)
```

### BUG-03: ADR nunca aumenta SF quando link esta ruim

**Arquivo:** `protocolos.py:83-86`
**Problema:** Quando `n_steps < 0` (margem negativa), o ADR so aumenta TX power, nunca aumenta SF. Se o device ja esta com TX power maximo e a margem continua negativa, ele fica travado num SF baixo sem alcance.
**Correcao:** Apos esgotar aumento de TX power, aumentar SF.

```python
# Adicionar apos o loop de aumento de TX power:
while n_steps < 0 and new_sf < 12:
    new_sf += 1
    n_steps += 1
```

### BUG-04: Duty cycle off-by-one

**Arquivo:** `network.py:234`, `enddevice.py:143`
**Problema:** Formula `airtime * (100/DC - 1)` nao inclui o proprio airtime no calculo. Para DC=1%, o device deveria esperar `airtime/0.01 = 100*airtime` total (incluindo o airtime), mas espera apenas `99*airtime` apos o fim.
**Impacto:** Baixo (diferenca de 1 airtime), mas incorreto tecnicamente.
**Correcao:** `dc_release_time = time + airtime / (ed_dc_limit_percent / 100.0)`

### BUG-05: `find_best_gateway` com `avoid_congestion=True` nao recebe device

**Arquivo:** `gateway.py:83`
**Problema:** `balance_load()` chama `find_best_gateway(avoid_congestion=True)` sem passar o device. O metodo retorna `(gateway, None, None)` sem calcular distancia/RSSI.
**Correcao:** Passar device como parametro.

### BUG-06: Classe A downlink mal implementada

**Arquivo:** `enddevice.py:296-303`
**Problema:** `can_receive_downlink()` usa `current_time % 10 == 0` para Classe A, mas Classe A so recebe DL nas janelas RX1/RX2 apos um UL. Nao e periodico.
**Correcao:** Verificar se esta dentro de uma janela RX1/RX2 aberta apos transmissao.

### BUG-07: SNR history usa list.pop(0) — O(n)

**Arquivo:** `protocolos.py:47`
**Problema:** `list.pop(0)` e O(n). Com muitas iteracoes, isso degrada performance.
**Correcao:** Usar `collections.deque(maxlen=ADR_HISTORY_SIZE)`.

### BUG-08: Ganho de antena definido mas nunca usado no calculo de RSSI

**Arquivo:** `parametors.py:64-65`, `gateway.py:40`, `network.py:437`
**Problema:** Os parametros `gw_antenna_gain = 5` e `ed_antenna_gain = 5` estao definidos mas **nunca sao usados** no calculo de RSSI. Ambos `gateway.py:40` e `network.py:437` calculam:
```python
rssi = device.tx_power - path_loss  # Falta ganho de antena!
```
**Correcao:** Incluir ganho de antena no link budget:
```python
rssi = device.tx_power + ed_antenna_gain + gw_antenna_gain - path_loss
```
**Impacto:** Alto — sem os +10 dBi de ganho total, a cobertura e subestimada e devices sao marcados fora de cobertura quando deveriam estar dentro.

### BUG-09: Unidades de distancia inconsistentes nos modelos de path loss

**Arquivo:** `network.py:467-543`
**Problema:** A distancia calculada em `find_best_gateway()` esta em **unidades de area_size** (default 100 = km). Porem:
- Okumura-Hata e COST-Hata usam `log10(distance)` e esperam distancia em **km** (correto se area_size esta em km)
- FSPL usa `20*log10(distance)` — a formula padrao espera **km** (correto)
- Log-Normal Shadowing proposto usa `d/d0` com `d0=1m` — precisa de **metros** (inconsistente)
- A distancia minima `max(..., 0.1)` esta em km = 100m minimo, que pode ser excessivo

**Correcao:** Padronizar todas as distancias internas em **metros**. Converter para km apenas nos modelos que exigem (Hata, COST-Hata). Adicionar parametro `distance_unit` explicito.

```python
# Padrao interno: metros
distance_m = np.sqrt((device.x - gateway.x)**2 + (device.y - gateway.y)**2) * 1000  # se area em km
distance_m = max(distance_m, 1.0)  # Minimo 1 metro

# Nos modelos que usam km:
distance_km = distance_m / 1000.0
```

### BUG-10: Propagation delay definido mas nunca chamado

**Arquivo:** `network.py:139-141`
**Problema:** O metodo `get_propagation_delay(distance)` existe mas nunca e invocado na simulacao. Para distancias LoRa tipicas (5km), o delay e ~16.7us — desprezivel comparado ao ToA (ms-s), mas a existencia do metodo sem uso gera confusao.
**Decisao:** Documentar como decisao de design (delay << ToA, ignorado por simplificacao) ou integrar no calculo de arrival_time para rigor.

---

## 3. Fase 1 - Fundamentos (PHY + Canal)

### 3.1 Modelo de Propagacao Log-Normal Shadowing

**Por que:** FLoRa, ns-3 e LoRaWANSim todos suportam. E o modelo mais usado em literatura LoRaWAN.
**Arquivo:** `network.py` (metodo `pathloss()`)

**Implementacao:**
```
PL(d) = PL(d0) + 10 * gamma * log10(d/d0) + X_sigma
onde X_sigma ~ N(0, sigma^2)
```

**Parametros (compativeis com FLoRa):**
- `d0 = 1m` (distancia de referencia)
- `PL_d0 = 7.7 dB` (perda a d0)
- `gamma = 3.76` (expoente de path loss)
- `sigma = 0` (deterministico) ou `sigma = 3.57` (tipico urbano)

**Vantagem sobre FLoRa/ns-3:** Oferecer TODOS os modelos (6 total) numa unica interface:
1. Okumura-Hata (ja implementado)
2. Log-Distance (ja implementado)
3. FSPL (ja implementado)
4. COST-Hata (ja implementado)
5. Fading (ja implementado)
6. **Log-Normal Shadowing (NOVO)**
7. **Correlated Shadowing (NOVO)** — como ns-3
8. **Building Penetration Loss (NOVO)** — como ns-3

### 3.2 Shadowing Espacialmente Correlacionado

**Por que:** ns-3 tem `CorrelatedShadowingPropagationLossModel`. Nenhum outro simulador Python tem.
**Conceito:** O shadowing entre pares (TX, RX) proximos e correlacionado. Usa-se um grid com interpolacao.

**Implementacao:**
- Criar classe `CorrelatedShadowingMap`
- Grid de valores gaussianos pre-gerados
- Interpolacao bilinear para posicoes intermediarias
- Parametro: `correlation_distance` (ex: 110m)

### 3.3 Building Penetration Loss

**Por que:** ns-3 tem. Necessario para cenarios indoor/outdoor.
**Implementacao:**
- Parametro `indoor_ratio` (fracao de devices indoor)
- Atenuacao adicional: `BPL ~ LogNormal(mean=20dB, std=10dB)` por device
- Baseado em ITU-R P.2109

### 3.4 SINR Completo

**Arquivo:** `channel.py` + `gateway.py`
**Problema atual:** Usa SNR (S/N) e SIR (S/I) separados. Nao calcula SINR = S/(I+N).

**Implementacao:**
```python
def calculate_sinr(self, packet, gateway):
    signal_dbm = packet.rssi
    noise_dbm = self.network.calculate_noise_floor(packet.bw)

    # Soma interferencia de todos os on-air na mesma freq
    interference_linear = sum(
        10**(other.rssi/10) for other in self.get_active_interferers(packet)
    )

    signal_linear = 10**(signal_dbm/10)
    noise_linear = 10**(noise_dbm/10)

    sinr_linear = signal_linear / (interference_linear + noise_linear)
    return 10 * np.log10(sinr_linear)
```

### 3.5 Multi-Canal

**Por que:** TODOS os outros simuladores suportam. PyLoRaWAN usa apenas 1 frequencia (915 MHz).
**Impacto:** Com 1 canal, TODOS os devices colidem entre si. Com 8 canais (EU868) ou 64 (US915), a capacidade multiplica.

**Implementacao:**
```python
# EU868 default channels
eu868_channels = [868.1, 868.3, 868.5]  # MHz, mandatory
eu868_additional = [867.1, 867.3, 867.5, 867.7, 867.9]  # optional

# US915 channels
us915_uplink = [902.3 + 0.2*i for i in range(64)]  # 64 x 125kHz
us915_uplink_500 = [903.0 + 1.6*i for i in range(8)]  # 8 x 500kHz
```

- Cada device seleciona canal aleatoriamente por transmissao (round-robin ou random)
- Colisoes so ocorrem dentro do mesmo canal
- Reduz colisoes em ~8x para EU868

### 3.6 Gateway com 8 Reception Paths (SX1301)

**Por que:** FLoRa (LoRaGWRadio) e ns-3 (GatewayLoraPhy com ReceptionPath) modelam isso.
**Conceito:** O chip SX1301 tem 8 demoduladores, cada um pode decodificar 1 sinal simultaneo.

**Implementacao:**
```python
class GatewayPHY:
    def __init__(self, num_paths=8):
        self.reception_paths = [None] * num_paths  # 8 demoduladores

    def try_receive(self, packet):
        # Encontra path livre
        for i, path in enumerate(self.reception_paths):
            if path is None or path.tx_end <= packet.tx_start:
                self.reception_paths[i] = packet
                return True
        return False  # Todos os paths ocupados
```

- Se >8 pacotes chegam simultaneamente, os excedentes sao perdidos (gateway saturado)
- Metrica nova: `gateway_saturation_events`

### 3.7 Calculo de Tempo de Sobreposicao Parcial

**Por que:** LoRaWANSim calcula `overlap_time` e ns-3 usa `GetOverlapTime()`. A interferencia deve ser proporcional ao tempo de sobreposicao, nao binaria.
**Implementacao:**
- Se overlap < 50% do pacote alvo, reduzir impacto da interferencia
- Ou usar modelo de Goursaud (ns-3): so conta colisao se overlap cobre periodo critico (header ou preamble)

### 3.8 Padronizacao de Unidades

**Problema:** Distancias inconsistentes entre modelos (ver BUG-09).

**Convencao interna proposta:**
| Grandeza | Unidade interna | Conversao |
|----------|----------------|-----------|
| Distancia | metros (m) | area_size deve ser em m (ex: 5000 = 5km) |
| Frequencia | Hz | Converter MHz -> Hz onde necessario |
| Potencia | dBm | Padrao LoRa |
| Tempo | segundos (s) | Padrao SI |
| Energia | mJ (miliJoules) | Compativel com ns-3 |
| Corrente | mA | Compativel com datasheets |

**Impacto:** Requer ajustar `area_size` de 100 (km) para 100000 (m) ou adicionar multiplicador. Todos os modelos de path loss recebem metros e convertem internamente.

---

## 4. Fase 2 - MAC e Protocolo

### 4.1 Maquina de Estados Classe A Correta

**Por que:** FLoRa e ns-3 implementam FSM completa. PyLoRaWAN tem estados simplificados.

**FSM proposta:**
```
IDLE → TX → WAIT_RX1 → RX1 → WAIT_RX2 → RX2 → IDLE
                         ↓                  ↓
                    (ACK recebido)     (ACK recebido)
                         ↓                  ↓
                       IDLE               IDLE
```

**Estados com duracoes:**
| Estado | Duracao | Corrente |
|--------|---------|----------|
| TX | airtime | tx_current[power] |
| WAIT_RX1 | receive_delay1 (1s) | sleep (0.0016 mA) |
| RX1 | rx1_window (1s) | rx (11.2 mA) |
| WAIT_RX2 | receive_delay2 - receive_delay1 - rx1_window | sleep |
| RX2 | rx2_window (2s) | rx |
| IDLE | ate proximo TX | sleep |

### 4.2 Classe B com Beacons

**Funcionalidade:**
- Gateway transmite beacons periodicos (a cada 128s)
- Devices sincronizam janela de recepcao (ping slot) com beacon
- Parametros: `ping_period`, `ping_offset`, `beacon_period`
- Recepcao DL possivel fora das janelas RX1/RX2

**Implementacao:**
```python
class ClassBEndDevice(EndDevice):
    def __init__(self, ...):
        self.beacon_period = 128  # seconds
        self.ping_period = 32     # seconds (configurable)
        self.is_synchronized = False

    def process_beacon(self, beacon_time):
        self.is_synchronized = True
        self.next_ping_slot = beacon_time + self.ping_offset

    def can_receive_downlink(self, time):
        if not self.is_synchronized:
            return False
        return (time - self.last_beacon) % self.ping_period < self.rx_window_duration
```

### 4.3 Classe C (Continuous Listening)

**Funcionalidade:**
- Device mantem radio em RX continuamente (exceto durante TX)
- Consumo muito maior, mas latencia DL minima
- Ideal para atuadores e devices com alimentacao externa

**Implementacao:**
```python
class ClassCEndDevice(EndDevice):
    def can_receive_downlink(self, time):
        return self.current_state != RadioState.TX

    def get_idle_current(self):
        return rx_current_ma  # Em vez de sleep_current
```

### 4.4 MAC Commands Completos

**Por que:** ns-3 implementa 5+ MAC commands. FLoRa implementa LinkAdrReq. PyLoRaWAN so tem ADR basico.

**MAC Commands a implementar:**

| Command | CID | Direcao | Funcao |
|---------|-----|---------|--------|
| LinkCheckReq/Ans | 0x02 | ED→NS, NS→ED | Verifica link, retorna margem e GW count |
| LinkAdrReq/Ans | 0x03 | NS→ED | Ajusta DR, TX power, channel mask |
| DutyCycleReq/Ans | 0x04 | NS→ED | Ajusta max duty cycle |
| RXParamSetupReq/Ans | 0x05 | NS→ED | Ajusta RX1DROffset, RX2 DR, RX2 freq |
| DevStatusReq/Ans | 0x06 | NS→ED | Solicita nivel de bateria e SNR |
| NewChannelReq/Ans | 0x07 | NS→ED | Adiciona/modifica canal |
| RXTimingSetupReq/Ans | 0x08 | NS→ED | Ajusta delay de RX1 |
| DlChannelReq/Ans | 0x0A | NS→ED | Ajusta frequencia DL |

**Estrutura:**
```python
class MACCommand:
    def __init__(self, cid, payload):
        self.cid = cid
        self.payload = payload

class LinkCheckAns(MACCommand):
    def __init__(self, margin_db, gw_count):
        super().__init__(0x02, {"margin": margin_db, "gw_count": gw_count})

class LinkAdrReq(MACCommand):
    def __init__(self, data_rate, tx_power, ch_mask, nb_trans):
        super().__init__(0x03, {
            "data_rate": data_rate,
            "tx_power": tx_power,
            "ch_mask": ch_mask,
            "nb_trans": nb_trans
        })
```

### 4.5 OTAA Completo com JoinAccept

**Problema atual:** `join_request()` apenas cria uma sessao ficticia. Nao simula timing.

**Implementacao:**
- JoinRequest: Device envia em canal aleatorio com DR0
- JoinAccept: NS responde em RX1 (5s) ou RX2 (6s) com parametros de rede
- Timing: JOIN_ACCEPT_DELAY1 = 5s, JOIN_ACCEPT_DELAY2 = 6s
- Parametros no JoinAccept: DevAddr, AppNonce, NetID, DLSettings, RxDelay, CFList

### 4.6 ABP (Activation by Personalization)

**Implementacao simples:**
- DevAddr e session keys pre-configurados
- Sem overhead de Join
- Util para cenarios de benchmark (skip join phase)

### 4.7 Frame Counter Validation

**Por que:** LoRaWAN requer frame counter crescente. Pacotes com FC duplicado ou menor devem ser descartados.

**Implementacao:**
```python
def validate_frame_counter(self, device_id, frame_counter):
    last_fc = self.device_frame_counters.get(device_id, 0)
    if frame_counter <= last_fc:
        return False  # Replay attack ou duplicata
    self.device_frame_counters[device_id] = frame_counter
    return True
```

### 4.8 Payload Size Limits por DR

**Por que:** LoRaWAN spec define payload maximo por DR. Atualmente `pl=80` para todos os SFs, mas SF12/11/10 no EU868 permite no maximo 51 bytes. Nenhum dos simuladores Python valida isso.

**Limites (EU868):**
| DR | SF | BW | Max Payload (bytes) |
|----|----|----|---------------------|
| 0 | 12 | 125kHz | 51 |
| 1 | 11 | 125kHz | 51 |
| 2 | 10 | 125kHz | 51 |
| 3 | 9 | 125kHz | 115 |
| 4 | 8 | 125kHz | 222 |
| 5 | 7 | 125kHz | 222 |
| 6 | 7 | 250kHz | 222 |

**Implementacao:**
```python
def validate_payload(self, sf, bw, payload_size, region):
    max_pl = region.max_payload_bytes.get(self.sf_to_dr(sf, bw), 51)
    if payload_size > max_pl:
        # Opcao 1: Truncar e avisar
        # Opcao 2: Fragmentar em multiplos pacotes
        # Opcao 3: Rejeitar transmissao
        return min(payload_size, max_pl)
    return payload_size
```

### 4.9 Downlink Traffic e Scheduling Completo

**Por que:** LoRaWANSim modela DL em detalhe (Step 10). ns-3 tem `NetworkScheduler`. PyLoRaWAN so envia ACK, nao modela trafego DL real.

**Componentes a implementar:**

**a) Geracao de trafego DL:**
```python
class DownlinkTrafficGenerator:
    def __init__(self, mode="ack_only"):
        self.mode = mode  # "ack_only", "periodic", "on_demand"

    def generate(self, device_id, time):
        if self.mode == "ack_only":
            return None  # So ACK, sem dados
        elif self.mode == "periodic":
            return DownlinkPacket(device_id, payload=b'\x00' * 10)
        elif self.mode == "on_demand":
            return self.check_queue(device_id)
```

**b) Duty cycle DL no gateway (ja definido em parametors.py, nunca implementado):**
```python
class GatewayDutyCycle:
    def __init__(self):
        self.rx1_dc = gw_rx1_dc_limit_percent / 100.0  # 1%
        self.rx2_dc = gw_rx2_dc_limit_percent / 100.0  # 10%
        self.last_tx = {"rx1": 0, "rx2": 0}

    def can_send_rx1(self, current_time, airtime):
        off_time = airtime / self.rx1_dc - airtime
        return current_time >= self.last_tx["rx1"] + off_time

    def can_send_rx2(self, current_time, airtime):
        off_time = airtime / self.rx2_dc - airtime
        return current_time >= self.last_tx["rx2"] + off_time
```

**c) Selecao de GW para DL (multi-GW):**
- Quando multiplos GWs recebem o mesmo UL, o NS deve escolher o melhor para enviar DL
- Criterios: menor duty cycle usado, melhor SNR do UL, menor latencia
- ns-3 usa `GetBestGatewayForDevice()` no NetworkStatus

**d) Fila de prioridades DL no NS:**
```python
class DownlinkScheduler:
    PRIORITY_ACK = 0          # Mais alta
    PRIORITY_MAC_COMMAND = 1
    PRIORITY_APP_DATA = 2     # Mais baixa

    def schedule(self, device_id, packet, priority):
        heapq.heappush(self.queue, (priority, time, device_id, packet))
```

### 4.10 Dwell Time (US915/AU915)

**Por que:** Regioes US915 e AU915 nao usam duty cycle, usam **dwell time** (max 400ms por TX). Afeta SF11/SF12 @ 125kHz que excedem 400ms.

**Impacto no ToA:**
| SF | BW 125kHz | ToA (80B) | Excede 400ms? |
|----|-----------|-----------|---------------|
| 7 | 125kHz | ~144ms | Nao |
| 8 | 125kHz | ~257ms | Nao |
| 9 | 125kHz | ~462ms | **Sim** |
| 10 | 125kHz | ~862ms | **Sim** |
| 11 | 125kHz | ~1482ms | **Sim** |
| 12 | 125kHz | ~2793ms | **Sim** |

**Implementacao:**
```python
def check_dwell_time(self, region, sf, bw, payload_size):
    if region.max_dwell_time_ms is None:
        return True  # Sem limite (EU868)
    toa_ms = self.calculate_airtime() * 1000
    if toa_ms > region.max_dwell_time_ms:
        return False  # Deve usar DR maior ou BW 500kHz
    return True
```

---

## 5. Fase 3 - Network Server e ADR

### 5.1 Modulo Network Server

**Por que:** FLoRa (NetworkServerApp) e ns-3 (NetworkServer + NetworkStatus + NetworkController) tem. PyLoRaWAN processa tudo inline.

**Arquitetura proposta:**
```
network_server/
    __init__.py
    server.py           # NetworkServer principal
    device_registry.py  # Registro e status de devices
    gateway_manager.py  # Status e selecao de GWs
    scheduler.py        # Escalonador de DL
    controller.py       # Controlador com componentes plugaveis
    components/
        __init__.py
        adr.py          # ADR component
        duty_cycle.py   # Duty cycle management
        link_check.py   # Link check component
```

**Classes:**
```python
class NetworkServer:
    def __init__(self):
        self.device_registry = DeviceRegistry()
        self.gateway_manager = GatewayManager()
        self.scheduler = DownlinkScheduler()
        self.controller = NetworkController()

    def on_uplink_received(self, packet, gateway):
        # Registra recepcao
        self.device_registry.update(packet, gateway)
        # Executa componentes (ADR, link check, etc.)
        self.controller.on_new_packet(packet)
        # Agenda DL se necessario
        if self.device_registry.needs_reply(packet.device_id):
            self.scheduler.schedule_downlink(packet, gateway)

class DeviceRegistry:
    """Equivalente ao NetworkStatus do ns-3."""
    def __init__(self):
        self.devices = {}  # device_id -> DeviceStatus

    def update(self, packet, gateway):
        status = self.devices.setdefault(packet.device_id, DeviceStatus())
        status.last_packet = packet
        status.gateways[gateway.gw_id] = {
            "rssi": packet.rssi,
            "snr": packet.snr,
            "time": packet.arrival_time
        }
```

### 5.2 Multi-Gateway Diversity Reception

**Por que:** ns-3 tem `GetBestGatewayForDevice()` no NetworkStatus. FLoRa encaminha via PacketForwarder. Essencial para cenarios multi-GW reais.

**Problema atual:** Cada pacote e processado por 1 GW apenas (o melhor por RSSI). Em redes reais, multiplos GWs recebem o mesmo UL e o NS seleciona o melhor.

**Implementacao:**
```python
class GatewayManager:
    def process_uplink_from_all_gateways(self, packet, gateways):
        """Todos os GWs em alcance recebem o pacote. NS escolhe o melhor."""
        receptions = []
        for gw in gateways:
            distance = self.calculate_distance(packet.device, gw)
            path_loss = self.network.pathloss(distance, packet.freq, ...)
            rssi = packet.tx_power + ed_antenna_gain + gw_antenna_gain - path_loss
            sensitivity = self.get_sensitivity(packet.sf, packet.bw)
            if rssi >= sensitivity:
                receptions.append({"gateway": gw, "rssi": rssi, "snr": rssi - noise_floor})

        if not receptions:
            return None  # Nenhum GW recebeu

        # Seleciona melhor GW (maior SNR)
        best = max(receptions, key=lambda r: r["snr"])

        # Registra que multiplos GWs receberam (para metricas)
        packet.received_by_gateways = receptions
        packet.gw_count = len(receptions)

        return best["gateway"]
```

**Metricas novas:**
- `avg_gw_diversity` — media de GWs que recebem cada pacote
- `spatial_diversity_gain_db` — ganho SNR por diversity vs single GW

### 5.3 ADR Avancado — Multiplas Politicas

**Por que:** ns-3 tem AVERAGE, MAXIMUM, MINIMUM. FLoRa tem "avg". LoRaWANSim tem margem unica.

**Politicas a implementar:**

| Politica | Descricao | Uso |
|----------|-----------|-----|
| `average` | Media dos ultimos N SNRs | Default (como ns-3/FLoRa) |
| `maximum` | Maximo SNR entre GWs | Multi-GW (ns-3 MAXIMUM) |
| `minimum` | Minimo SNR (conservador) | Garantia de cobertura |
| `percentile` | Percentil 90 dos SNRs | Robusto a outliers |
| `ewma` | Media movel exponencial | Reacao rapida a mudancas |

```python
class ADRComponent:
    def __init__(self, method="average", margin_db=10, history_size=20):
        self.method = method
        self.margin_db = margin_db
        self.history_size = history_size

    def compute_snr_metric(self, snr_history):
        if self.method == "average":
            return np.mean(snr_history)
        elif self.method == "maximum":
            return np.max(snr_history)
        elif self.method == "minimum":
            return np.min(snr_history)
        elif self.method == "percentile":
            return np.percentile(snr_history, 90)
        elif self.method == "ewma":
            alpha = 0.3
            result = snr_history[0]
            for snr in snr_history[1:]:
                result = alpha * snr + (1 - alpha) * result
            return result
```

### 5.3 ADR Backoff Mechanism

**Por que:** LoRaWAN spec define ADR_ACK_LIMIT e ADR_ACK_DELAY. ns-3 implementa.

**Mecanismo:**
- Se device nao recebe DL por `ADR_ACK_LIMIT` (64) frames: seta ADR_ACK_REQ
- Se nao recebe resposta apos mais `ADR_ACK_DELAY` (32) frames: aumenta TX power
- Se TX power maximo: muda para SF+1
- Se SF12: ativa todos os canais default
- Garante que devices "perdidos" voltem a ser ouvidos

### 5.4 SF Assignment by Distance

**Por que:** ns-3 (`SetSpreadingFactorsUp`) e LoRaWANSim (otimizacao ADR) fazem isso.

**Implementacao:**
```python
def assign_sf_by_distance(self):
    """Atribui SF minimo que satisfaz link budget por device."""
    for device in self.devices:
        _, distance, _ = self.find_best_gateway(device)
        if distance is None:
            device.sf = 12
            continue
        path_loss = self.pathloss(distance, device.freq, self.model_pathloss)
        rssi = device.tx_power - path_loss

        # Encontra SF minimo onde RSSI > sensibilidade
        for sf in range(7, 13):
            sensitivity = device.set_sensibilidade(sf, device.bw)
            if rssi > sensitivity + margin_db:
                device.sf = sf
                break
        else:
            device.sf = 12
```

---

## 6. Fase 4 - Energia e Bateria

### 6.1 Battery Capacity e Depletion

**Por que:** ns-3 tem `BasicEnergySource` com deplecao. Nenhum simulador Python tem.

**Implementacao:**
```python
class BatteryModel:
    def __init__(self, capacity_mah=2400, voltage=3.3):
        self.capacity_mah = capacity_mah
        self.capacity_mj = capacity_mah * voltage * 3.6  # mAh -> mJ
        self.remaining_mj = self.capacity_mj
        self.depleted = False

    def consume(self, energy_mj):
        self.remaining_mj -= energy_mj
        if self.remaining_mj <= 0:
            self.remaining_mj = 0
            self.depleted = True

    def soc_percent(self):
        return (self.remaining_mj / self.capacity_mj) * 100

    def estimate_lifetime_days(self, avg_consumption_mj_per_hour):
        if avg_consumption_mj_per_hour <= 0:
            return float('inf')
        hours = self.remaining_mj / avg_consumption_mj_per_hour
        return hours / 24
```

### 6.2 Energy Harvesting

**Diferencial:** Nenhum dos 4 simuladores tem. Relevante para IoT com painel solar.

**Implementacao:**
```python
class EnergyHarvester:
    def __init__(self, model="solar", peak_power_mw=100):
        self.model = model
        self.peak_power_mw = peak_power_mw

    def get_power(self, time_of_day_hours):
        if self.model == "solar":
            # Modelo senoidal simples (6h-18h)
            if 6 <= time_of_day_hours <= 18:
                return self.peak_power_mw * np.sin(np.pi * (time_of_day_hours - 6) / 12)
            return 0
        elif self.model == "constant":
            return self.peak_power_mw
```

### 6.3 Correntes TX Detalhadas por SF

**Por que:** O LoRaWANSim usa correntes diferentes por DR/SF. Atualmente PyLoRaWAN so varia por TX power.

**Implementacao:** Tabela cruzada TX power x SF para corrente:
```python
# Corrente TX (mA) - SX1272 datasheet
tx_current_detailed = {
    # (tx_power_dbm, sf): current_ma
    (14, 7): 38, (14, 12): 38,  # Mesma corrente para mesmo power
    (10, 7): 32, (10, 12): 32,
    # Mas ToA maior em SF12 -> mais energia
}
```

### 6.4 Metricas de Energia Detalhadas

**Saida por device:**
```python
{
    "device_id": 0,
    "total_energy_mj": 156.3,
    "energy_breakdown": {
        "tx_mj": 89.2,
        "rx_mj": 45.1,
        "sleep_mj": 0.3,
        "standby_mj": 21.7
    },
    "battery_soc_percent": 98.7,
    "estimated_lifetime_days": 1825,
    "duty_cycle_actual": 0.87,
    "avg_current_ua": 12.3
}
```

---

## 7. Fase 5 - Parametros Regionais

### 7.1 Modulo de Parametros Regionais

**Por que:** ns-3 suporta EU, US, China, AU, AS, SK. FLoRa e configuravel via .ini. PyLoRaWAN tem parametros fixos.

**Estrutura:**
```python
# regions.py
class RegionalParameters:
    """Base class para parametros regionais LoRaWAN."""
    pass

class EU868(RegionalParameters):
    name = "EU868"
    frequency_range = (863, 870)  # MHz
    default_channels = [868.1, 868.3, 868.5]
    additional_channels = [867.1, 867.3, 867.5, 867.7, 867.9]
    rx2_frequency = 869.525
    rx2_dr = 0  # SF12
    max_tx_power_dbm = 16  # ERP
    duty_cycle = {
        (868.0, 868.6): 0.01,   # 1% sub-band g1
        (868.7, 869.2): 0.001,  # 0.1% sub-band g2
        (869.4, 869.65): 0.10,  # 10% sub-band g3
    }
    dr_table = {
        0: {"sf": 12, "bw": 125000},
        1: {"sf": 11, "bw": 125000},
        2: {"sf": 10, "bw": 125000},
        3: {"sf": 9,  "bw": 125000},
        4: {"sf": 8,  "bw": 125000},
        5: {"sf": 7,  "bw": 125000},
        6: {"sf": 7,  "bw": 250000},
    }
    max_payload_bytes = {0: 51, 1: 51, 2: 51, 3: 115, 4: 222, 5: 222, 6: 222}

class US915(RegionalParameters):
    name = "US915"
    uplink_channels_125k = [902.3 + 0.2*i for i in range(64)]
    uplink_channels_500k = [903.0 + 1.6*i for i in range(8)]
    downlink_channels = [923.3 + 0.6*i for i in range(8)]
    rx2_frequency = 923.3
    rx2_dr = 8  # SF12/500kHz
    max_tx_power_dbm = 30  # EIRP (with antenna gain)
    duty_cycle = None  # No duty cycle, uses dwell time
    max_dwell_time_ms = 400  # 400ms max TX time

class AU915(RegionalParameters):
    # Similar to US915 but different freqs
    ...

class AS923(RegionalParameters):
    ...
```

### 7.2 Duty Cycle por Sub-Banda

**Implementacao:**
```python
class DutyCycleManager:
    def __init__(self, region):
        self.region = region
        self.sub_band_usage = {}  # sub_band -> last_tx_end

    def can_transmit(self, frequency, current_time, airtime):
        sub_band = self.get_sub_band(frequency)
        dc_limit = self.region.duty_cycle.get(sub_band, 0.01)

        last_tx = self.sub_band_usage.get(sub_band, 0)
        required_off_time = airtime / dc_limit - airtime

        return current_time >= last_tx + required_off_time
```

---

## 8. Fase 6 - LR-FHSS (Diferencial Unico)

### Por que e o maior diferencial

**Nenhum dos outros 4 simuladores combina LoRa CSS + LR-FHSS.** O LR-FHSS-sim e separado e nao tem path loss, energia, ADR, mobilidade. Integrar LR-FHSS no PyLoRaWAN criaria o **unico simulador que compara as duas PHYs sob as mesmas condicoes**.

### 8.1 PHY LR-FHSS

**Estrutura:**
```python
class LRFHSS_PHY:
    def __init__(self, code_rate="1/3", obw=35, headers=3):
        self.code_rate = code_rate
        self.obw = obw  # Occupied Bandwidth (canais)
        self.headers = headers
        self.header_duration = 0.233472  # s
        self.payload_duration = 0.1024   # s
        self.transceiver_wait = 0.006472 # s

    def fragment_packet(self, payload_size):
        """Fragmenta pacote em headers + payloads."""
        if self.code_rate == "1/3":
            n_payloads = math.ceil((payload_size + 3) / 2)
            threshold = math.ceil(n_payloads / 3)
        elif self.code_rate == "1/2":
            n_payloads = math.ceil((payload_size + 3) / 3)
            threshold = math.ceil(n_payloads / 2)
        # ... outros code rates

        return self.headers, n_payloads, threshold

    def calculate_toa(self, payload_size):
        h, p, _ = self.fragment_packet(payload_size)
        return (h * self.header_duration +
                p * self.payload_duration +
                self.transceiver_wait)

    def generate_hopping_sequence(self, n_fragments):
        """Gera sequencia de canais aleatorios."""
        return [random.randint(0, self.obw - 1) for _ in range(n_fragments)]
```

### 8.2 Colisao por Fragmento

```python
class LRFHSS_Channel:
    def evaluate_packet(self, packet, fragments):
        """Avalia recepcao com decodificacao parcial."""
        h_success = sum(1 for f in fragments
                       if f.type == 'header' and not f.collided)
        p_success = sum(1 for f in fragments
                       if f.type == 'payload' and not f.collided)

        return h_success >= 1 and p_success >= packet.threshold
```

### 8.3 ACRDA (Asynchronous Coded Random Access)

**Cancelamento de interferencia iterativo:**
```python
class ACRDA:
    def __init__(self, window_size=3):
        self.window_size = window_size
        self.memory = {}

    def process_window(self):
        """SIC iterativo dentro da janela."""
        new_recovery = True
        while new_recovery:
            new_recovery = False
            for packet in self.get_failed_packets():
                if self.try_decode(packet):
                    self.cancel_interference(packet)
                    new_recovery = True
```

---

## 9. Fase 7 - Deployment e Topologia

### 9.1 Deploy Circular Uniforme (como FLoRa)

**Problema atual:** Devices em grid. FLoRa e LoRaWANSim usam disco circular.

```python
def deploy_circular(self, n_devices, radius, center_x, center_y):
    """Deploy uniforme num disco (sqrt(U) para uniformidade areal)."""
    devices = []
    for i in range(n_devices):
        r = radius * np.sqrt(np.random.uniform(0, 1))
        theta = np.random.uniform(0, 2 * np.pi)
        x = center_x + r * np.cos(theta)
        y = center_y + r * np.sin(theta)
        devices.append((x, y))
    return devices
```

### 9.2 Deploy Hexagonal (como ns-3)

```python
def deploy_hexagonal(self, n_devices, inter_distance):
    """Grid hexagonal para gateways."""
    positions = []
    row = 0
    while len(positions) < n_devices:
        x_offset = (inter_distance / 2) if row % 2 else 0
        col = 0
        while len(positions) < n_devices:
            x = col * inter_distance + x_offset
            y = row * inter_distance * np.sqrt(3) / 2
            positions.append((x, y))
            col += 1
        row += 1
    return positions[:n_devices]
```

### 9.3 Import de Posicoes Externas

```python
def load_positions_from_csv(self, filename):
    """Carrega posicoes de arquivo CSV (compativel com ns-3/FLoRa exports)."""
    positions = []
    with open(filename) as f:
        reader = csv.reader(f)
        next(reader)  # header
        for row in reader:
            positions.append((float(row[0]), float(row[1])))
    return positions
```

### 9.4 Multiplos Tipos de Deploy

| Tipo | Descricao |
|------|-----------|
| `grid` | Grade regular (atual) |
| `circular` | Disco uniforme (FLoRa) |
| `annular` | Anel [r_min, r_max] (LoRaWANSim) |
| `hexagonal` | Grade hexagonal (ns-3) |
| `random_uniform` | Uniforme na area |
| `clustered` | Clusters gaussianos |
| `from_file` | Importar CSV |

---

## 10. Fase 8 - Saida e Analise

### 10.1 Formato de Saida Compativel

**Problema atual:** Saida apenas em CSV basico e PNG. Sem compatibilidade com ferramentas de analise.

**Saidas propostas:**

| Formato | Descricao | Compativel com |
|---------|-----------|----------------|
| CSV detalhado | Por pacote, por device, por GW | Todos |
| JSON summary | Metricas agregadas | Scripts de comparacao |
| .vec (series) | Time series | OMNeT++ opp_scavetool |
| .sca (scalars) | Metricas escalares | OMNeT++ opp_scavetool |
| HDF5 | Dados binarios compactos | Python/MATLAB |

### 10.2 Metricas Completas

**Metricas globais:**
```python
{
    "simulation": {
        "duration_s": 3600,
        "num_devices": 50,
        "num_gateways": 1,
        "region": "EU868",
        "seed": 42
    },
    "performance": {
        "pdr_percent": 95.2,
        "pdr_per_sf": {7: 98.1, 8: 96.3, ...},
        "throughput_bps": 1234.5,
        "avg_delay_ms": 45.2,
        "collision_rate_percent": 4.8,
        "retransmission_rate_percent": 3.1
    },
    "energy": {
        "total_network_mj": 15600,
        "avg_per_device_mj": 312,
        "avg_battery_lifetime_days": 1825,
        "breakdown": {"tx": 60, "rx": 25, "sleep": 15}
    },
    "adr": {
        "adjustments_count": 23,
        "sf_distribution_final": {7: 20, 8: 15, ...},
        "avg_convergence_time_s": 120
    }
}
```

### 10.3 Graficos Automaticos Expandidos

| Grafico | Descricao |
|---------|-----------|
| PDR vs numero de devices | Escalabilidade |
| PDR vs distancia | Cobertura |
| PDR por SF | Fairness |
| Energia por SF | Eficiencia |
| SF distribution over time | ADR convergence |
| CDF de delay | Latencia |
| Heatmap de colisoes | Spatial analysis |
| Battery SoC over time | Lifetime |
| Throughput over time | Capacidade |
| SINR distribution | Link quality |

---

## 11. Fase 9 - Performance e Escalabilidade

### 11.1 Spatial Indexing (KD-Tree)

**Problema:** `find_best_gateway()` calcula distancia para TODOS os gateways a cada TX. O(N*M).
**Solucao:** KD-Tree para busca de vizinhos.

```python
from scipy.spatial import cKDTree

class SpatialIndex:
    def __init__(self, gateways):
        coords = [(gw.x, gw.y) for gw in gateways]
        self.tree = cKDTree(coords)
        self.gateways = gateways

    def find_nearest(self, x, y, k=3):
        distances, indices = self.tree.query([x, y], k=k)
        return [(self.gateways[i], distances[j]) for j, i in enumerate(indices)]
```

### 11.2 Event Scheduler Optimization

**Problema:** Lazy cancellation acumula eventos mortos no heap.
**Solucao:** Periodic compaction.

```python
def compact(self):
    """Remove eventos cancelados do heap periodicamente."""
    self.event_queue = [e for e in self.event_queue if not e.cancelled]
    heapq.heapify(self.event_queue)
```

### 11.3 Parallel Simulation (Multi-Seed)

```python
from concurrent.futures import ProcessPoolExecutor

def run_multiple_seeds(config, seeds, n_workers=4):
    """Executa simulacao com multiplas seeds em paralelo."""
    with ProcessPoolExecutor(max_workers=n_workers) as executor:
        futures = [executor.submit(run_single, config, seed) for seed in seeds]
        results = [f.result() for f in futures]
    return aggregate_results(results)
```

### 11.4 Progress Bar

```python
# Usando tqdm para simulacoes longas
from tqdm import tqdm

def run_with_progress(self, until):
    pbar = tqdm(total=int(until), desc="Simulando", unit="s")
    last_time = 0
    while self.event_queue:
        event = self.event_queue[0]
        if event.time > until:
            break
        # Update progress
        if int(event.time) > last_time:
            pbar.update(int(event.time) - last_time)
            last_time = int(event.time)
        # ... process event
    pbar.close()
```

---

## 12. Fase 10 - Validacao, Testes e Infraestrutura

### 12.1 Modelo Analitico de Validacao

**Por que:** LoRaWANSim calcula `analytical_Ps1` e `analytical_Ps2` (probabilidade de sucesso Poisson). Nenhum outro simulador Python tem modelo analitico integrado para validacao rapida.

**Modelo ALOHA puro (baseline):**
```python
def analytical_pdr_aloha(n_devices, toa_s, period_s, n_channels=1):
    """Probabilidade de sucesso via modelo ALOHA puro.
    G = carga de trafego oferecida por canal.
    P_success = exp(-2*G) para ALOHA puro."""
    G = n_devices * toa_s / (period_s * n_channels)
    return np.exp(-2 * G)
```

**Modelo com SFs ortogonais (mais realista):**
```python
def analytical_pdr_per_sf(n_devices, sf_distribution, toa_per_sf, period_s, n_channels):
    """PDR analitico considerando ortogonalidade parcial entre SFs."""
    pdr = {}
    for sf in range(7, 13):
        n_sf = n_devices * sf_distribution.get(sf, 0)
        G_sf = n_sf * toa_per_sf[sf] / (period_s * n_channels)
        pdr[sf] = np.exp(-2 * G_sf)  # Intra-SF collision only
    return pdr
```

**Uso:** Comparar resultado da simulacao com modelo analitico. Divergencia > 10% indica bug ou configuracao incorreta.

### 12.2 Testes Automatizados

**Por que:** Nenhum dos simuladores Python tem suite de testes. Essencial para confiabilidade e validacao academica.

**Estrutura:**
```
tests/
    test_toa.py              # Time-on-Air vs valores Semtech calculadora
    test_pathloss.py         # Path loss vs valores conhecidos
    test_sensitivity.py      # Tabela de sensibilidade vs datasheet
    test_energy.py           # Energia vs calculo manual
    test_adr.py              # ADR convergence
    test_channel.py          # Colisoes e capture effect
    test_duty_cycle.py       # Limites de duty cycle
    test_regions.py          # Parametros regionais vs spec
    test_regression.py       # Seed=42, resultado identico
    test_integration.py      # Cenario completo end-to-end
    conftest.py              # Fixtures (cenario 50 EDs, 1 GW)
```

**Testes de regressao (seed=42):**
```python
def test_regression_50ed_1gw():
    """Resultado deve ser identico entre versoes com mesmo seed."""
    network = Network(..., random_seed=42)
    network.simulate_transmissions()
    stats = network.packet_tracker.get_stats()
    assert stats["Taxa de Entrega (PDR)"] == 95.2  # Valor esperado
    assert stats["Colisões"] == 12                   # Valor esperado
```

**Testes de validacao cruzada:**
```python
def test_toa_sf7_125khz_20bytes():
    """ToA deve ser ~51.5ms (Semtech LoRa Calculator)."""
    device = EndDevice(sf=7, bw=125000, pl=20, cr=1, ...)
    toa = device.calculate_airtime()
    assert abs(toa - 0.0515) < 0.001  # Tolerancia 1ms

def test_toa_sf12_125khz_51bytes():
    """ToA deve ser ~1810ms (Semtech LoRa Calculator)."""
    device = EndDevice(sf=12, bw=125000, pl=51, cr=1, ...)
    toa = device.calculate_airtime()
    assert abs(toa - 1.810) < 0.01  # Tolerancia 10ms
```

### 12.3 Configuracao via YAML/JSON

**Por que:** Atualmente tudo esta hardcoded em `parametors.py`. Para batch runs e reproducibilidade cientifica, precisa de config files.

**Formato proposto (YAML):**
```yaml
# config/scenario_50ed_1gw.yaml
simulation:
  duration_s: 3600
  seed: 42
  region: EU868

network:
  num_devices: 50
  num_gateways: 1
  deployment: circular
  deployment_radius_m: 5000

devices:
  tx_power_dbm: 14
  payload_bytes: 20
  traffic_interval_s: 300
  adr_enabled: true
  mobility:
    enabled: false
    model: random_walk
    speed_mps: 1.0

physical:
  pathloss_model: log_normal_shadowing
  pathloss_params:
    gamma: 3.76
    sigma: 3.57
    d0_m: 1.0
    pl_d0_db: 7.7

output:
  csv: true
  json: true
  plots: true
  directory: results/scenario_50ed/
```

**Batch mode:**
```yaml
# config/batch_scalability.yaml
base: config/scenario_50ed_1gw.yaml
sweep:
  parameter: network.num_devices
  values: [10, 50, 100, 200, 500, 1000]
  seeds: [42, 123, 456, 789, 1024]
```

**Carregamento:**
```python
import yaml

def load_config(filename):
    with open(filename) as f:
        config = yaml.safe_load(f)
    return config

def run_from_config(config_file):
    config = load_config(config_file)
    network = Network(**config_to_params(config))
    network.simulate_transmissions()
```

### 12.4 Logging Estruturado

**Por que:** Atualmente usa `print()` com emojis misturados. Impossivel filtrar, redirecionar ou analisar logs.

**Implementacao:**
```python
import logging

# Loggers por componente
phy_logger = logging.getLogger("pylorawan.phy")
mac_logger = logging.getLogger("pylorawan.mac")
adr_logger = logging.getLogger("pylorawan.adr")
energy_logger = logging.getLogger("pylorawan.energy")
network_logger = logging.getLogger("pylorawan.network")

def setup_logging(level="INFO", log_file=None):
    """Configura logging com nivel e saida."""
    fmt = "%(asctime)s [%(name)s] %(levelname)s: %(message)s"
    handlers = [logging.StreamHandler()]
    if log_file:
        handlers.append(logging.FileHandler(log_file))
    logging.basicConfig(level=getattr(logging, level), format=fmt, handlers=handlers)
```

**Niveis:**
| Nivel | Uso |
|-------|-----|
| DEBUG | Cada pacote TX/RX, cada calculo SINR |
| INFO | Eventos importantes (join, ADR change, collision summary) |
| WARNING | Devices fora de cobertura, gateway saturado |
| ERROR | Configuracao invalida, bugs detectados |

### 12.5 Validacao Cruzada com FLoRa/ns-3

**Cenario de referencia (identico ao `ns3_comparison.ini` do FLoRa):**

| Parametro | Valor |
|-----------|-------|
| Devices | 50 |
| Gateways | 1 (centro) |
| Deploy | Disco circular, raio 5km |
| Duracao | 3600s (1 hora) |
| Payload | 20 bytes |
| Intervalo TX | 300s (exponencial) |
| Path loss | Log-Normal, gamma=3.76, sigma=0 |
| ADR | Desabilitado |
| Seed | 42 |
| Regiao | EU868 (3 canais default) |

**Metricas a comparar:**
| Metrica | FLoRa | ns-3 | PyLoRaWAN | Tolerancia |
|---------|-------|------|-----------|------------|
| PDR (%) | ? | ? | ? | +/- 5% |
| Energia total (mJ) | ? | ? | ? | +/- 10% |
| SF distribution | ? | ? | ? | +/- 2 devices/SF |
| Colisoes | ? | ? | ? | +/- 5% |

**Script de comparacao:**
```python
def compare_with_flora(pylorawan_results, flora_csv):
    """Compara resultados com saida do FLoRa (.sca)."""
    flora = parse_flora_scalars(flora_csv)
    comparison = {
        "pdr_diff": abs(pylorawan_results["pdr"] - flora["pdr"]),
        "energy_diff_percent": abs(pylorawan_results["energy"] - flora["energy"]) / flora["energy"] * 100,
    }
    return comparison
```

---

## 13. Matriz de Superacao por Framework

### Checklist: O que PyLoRaWAN tera que os outros NAO tem

| Feature | FLoRa | LoRaWANSim | ns-3 | LR-FHSS-sim | PyLoRaWAN (apos upgrade) |
|---------|-------|------------|------|-------------|--------------------------|
| LoRa CSS + LR-FHSS no mesmo sim | N | N | N | N | **SIM** |
| 8 modelos de path loss | 3 | 1 | 2 | 0 | **8** |
| 4 modelos de mobilidade | INET | 0 | ns-3 | 0 | **4** (nativo) |
| 5+ politicas ADR | 1 | 1 | 3 | 0 | **5** |
| Battery depletion + harvesting | Parcial | N | Sim | N | **SIM** |
| Energy harvesting | N | N | N | N | **SIM** |
| Parametros regionais (EU/US/AU/AS) | Config | Parcial | Sim | N | **SIM** |
| Multi-canal (8+ canais) | Sim | Sim | Sim | N/A | **SIM** |
| Gateway SX1301 (8 paths) | Sim | N/A | Sim | N/A | **SIM** |
| Classe A + B + C | A | A+C | A | N/A | **A+B+C** |
| MAC commands (5+) | 1 | 0 | 5 | 0 | **8** |
| ACRDA (SIC) | N | N | N | Sim | **SIM** |
| Correlated shadowing | N | N | Sim | N | **SIM** |
| Building penetration | N | N | Sim | N | **SIM** |
| Multi-seed parallel | N | Sim | N | N | **SIM** |
| Multi-GW diversity | Sim | Sim | Sim | N | **SIM** |
| Modelo analitico integrado | N | Sim | N | N | **SIM** |
| Suite de testes (pytest) | N | N | N | N | **SIM** |
| Config YAML/JSON + batch | N | N | N | N | **SIM** |
| Logging estruturado | N/A | N | N/A | N | **SIM** |
| Payload size validation | N | N | N | N/A | **SIM** |
| Downlink traffic + GW DC | Parcial | Sim | Sim | N | **SIM** |
| Dwell time (US915) | N/A | N | Sim | N | **SIM** |
| Validacao cruzada FLoRa/ns-3 | N/A | N/A | N/A | N/A | **SIM** |
| Python puro (pip install) | N (C++) | N (Octave) | N (C++) | Sim | **SIM** |
| GUI / plots nativos | Qtenv | Octave | Limitado | N | **Matplotlib** |
| Facilidade de instalacao | Baixa | Media | Baixa | Alta | **Alta** |

### Superacao por framework:

**vs FLoRa:** Mais modelos de path loss, LR-FHSS, energy harvesting, multiplas politicas ADR, modelo analitico, testes automatizados, config YAML, Classes B/C, Python puro
**vs LoRaWANSim:** Eventos discretos (nao script), mobilidade, Classe B/C, MAC commands, GUI, testes, logging, validacao cruzada
**vs ns-3:** Python puro (sem build C++), LR-FHSS, energy harvesting, 8 path loss models, modelo analitico, config YAML, facilidade de instalacao
**vs LR-FHSS-sim:** Full stack (MAC, ADR, energia, mobilidade, path loss), LoRa CSS + LR-FHSS integrados, testes, logging, multi-GW

---

## 14. Prioridades e Roadmap

### Sprint 1 — Correcoes Criticas (1-2 dias)
- [ ] BUG-01: Interferencia no gateway (verificar on-air)
- [ ] BUG-02: SINR completo (S/(I+N))
- [ ] BUG-03: ADR aumenta SF quando margem negativa
- [ ] BUG-04: Duty cycle corrigido
- [ ] BUG-05: balance_load passa device
- [ ] BUG-06: Classe A downlink corrigido
- [ ] BUG-07: deque para SNR history
- [ ] BUG-08: Ganho de antena no calculo RSSI
- [ ] BUG-09: Padronizacao de unidades de distancia
- [ ] BUG-10: Documentar decisao sobre propagation delay

### Sprint 2 — PHY Robusto (3-5 dias)
- [ ] Log-Normal Shadowing
- [ ] Correlated Shadowing
- [ ] Building Penetration Loss
- [ ] Multi-canal (EU868 3 canais default)
- [ ] Gateway 8 reception paths (SX1301)
- [ ] Padronizacao de unidades (metros internamente)

### Sprint 3 — MAC Completo (3-5 dias)
- [ ] FSM Classe A correta
- [ ] Classe B com beacons
- [ ] Classe C continuous
- [ ] MAC Commands (LinkCheck, LinkAdr, DutyCycle, DevStatus, NewChannel, RXParamSetup, RXTimingSetup, DlChannel)
- [ ] OTAA com JoinAccept timing
- [ ] ABP activation mode
- [ ] Frame counter validation
- [ ] Payload size validation por DR/regiao
- [ ] Dwell time enforcement (US915/AU915)

### Sprint 4 — Network Server + ADR (2-3 dias)
- [ ] Modulo NetworkServer (server, registry, controller)
- [ ] Device Registry com status por GW
- [ ] Multi-GW diversity reception
- [ ] Downlink Scheduler com fila de prioridades
- [ ] Downlink traffic generation (ack_only, periodic, on_demand)
- [ ] Duty cycle DL no gateway (RX1: 1%, RX2: 10%)
- [ ] ADR multiplas politicas (avg/max/min/percentile/ewma)
- [ ] ADR Backoff (ADR_ACK_LIMIT/DELAY)
- [ ] SF assignment by distance

### Sprint 5 — Energia e Regioes (2-3 dias)
- [ ] Battery capacity + depletion
- [ ] Energy harvesting (solar, constant)
- [ ] Parametros regionais (EU868, US915, AU915, AS923)
- [ ] Duty cycle por sub-banda
- [ ] Metricas de energia detalhadas por device

### Sprint 6 — LR-FHSS (3-5 dias)
- [ ] PHY LR-FHSS (fragmentacao, hopping)
- [ ] Colisao por fragmento
- [ ] Decodificacao parcial (threshold)
- [ ] ACRDA (SIC iterativo)
- [ ] Comparacao CSS vs LR-FHSS no mesmo cenario

### Sprint 7 — Deploy e Saida (2-3 dias)
- [ ] Deploy circular, annular, hexagonal, clustered, from_file
- [ ] Import posicoes de CSV (compativel FLoRa/ns-3)
- [ ] Output JSON/CSV/HDF5
- [ ] 10+ graficos automaticos
- [ ] Modelo analitico ALOHA para validacao

### Sprint 8 — Infraestrutura e Qualidade (2-3 dias)
- [ ] Logging estruturado (logging module, niveis, por componente)
- [ ] Substituir todos os print() por logger
- [ ] Configuracao via YAML/JSON
- [ ] Batch mode (sweep de parametros)
- [ ] Suite de testes (pytest): unit, regression, validation
- [ ] Validacao cruzada com FLoRa/ns-3

### Sprint 9 — Performance (1-2 dias)
- [ ] Spatial indexing (KD-Tree)
- [ ] Event scheduler compaction
- [ ] Multi-seed parallel execution
- [ ] Progress bar (tqdm)

---

### Estimativa Total: ~24-33 dias de desenvolvimento

### Resultado Esperado:
- **~8000-10000 LOC** (vs ~2048 atuais)
- **10 bugs corrigidos** (7 originais + 3 novos identificados)
- **Unico simulador** com LoRa CSS + LR-FHSS integrados
- **Mais modelos** que qualquer outro (8 path loss, 5 ADR, 4 mobility, 7 deploy)
- **24 features exclusivas** que nenhum outro simulador individual possui (ver tabela Secao 13)
- **Python puro** — pip install, sem build C++ ou MATLAB
- **Compliant** com LoRaWAN 1.1 (MAC commands, Classes A/B/C, regional params, payload limits, dwell time)
- **Validavel** — modelo analitico integrado + validacao cruzada com FLoRa/ns-3
- **Testado** — suite pytest com unit, regression e validation tests
- **Configuravel** — YAML/JSON config + batch mode para sweep de parametros
- **Profissional** — logging estruturado, sem prints com emoji, niveis por componente

### Contagem de Features (apos upgrade completo):

| Categoria | Quantidade |
|-----------|-----------|
| Bugs corrigidos | 10 |
| Modelos de path loss | 8 |
| Modelos de mobilidade | 4 |
| Politicas ADR | 5 |
| Classes LoRaWAN | 3 (A+B+C) |
| MAC Commands | 8 |
| Parametros regionais | 4 (EU868, US915, AU915, AS923) |
| Tipos de deploy | 7 |
| Formatos de saida | 5 (CSV, JSON, .vec, .sca, HDF5) |
| Graficos automaticos | 10+ |
| PHY layers | 2 (LoRa CSS + LR-FHSS) |
| Sprints totais | 9 |

---

*Documento gerado em 05/03/2026, atualizado em 05/03/2026. Baseado na analise completa dos 5 frameworks e do codigo-fonte do PyLoRaWAN.*
