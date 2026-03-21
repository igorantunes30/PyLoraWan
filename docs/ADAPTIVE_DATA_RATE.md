# Adaptive Data Rate Operation — PyLoRaWAN

## Visão Geral

O ADR (Adaptive Data Rate) é o mecanismo do LoRaWAN que ajusta dinamicamente o Spreading Factor (SF) e a potência de transmissão (TX Power) de cada End Device para maximizar a capacidade da rede, minimizar o consumo de energia e manter a confiabilidade do enlace.

No PyLoRaWAN, o ADR é implementado **exclusivamente no lado do Network Server** (modo `server_side_adr=True`, padrão), seguindo a arquitetura do ns-3. O NS coleta histórico de SNR de pacotes recebidos e emite `LinkAdrReq` via downlink para comandar os devices a ajustarem seus parâmetros de rádio.

```
EndDevice                     Network Server
   │                               │
   │──── UplinkPacket (SNR) ──────►│  on_uplink_received()
   │                               │    validate_frame_counter()
   │                               │    device_registry.update()
   │                               │    controller.on_new_packet()
   │                               │      adr_component.on_packet()
   │                               │        compute_snr_metric()
   │                               │        calcula margin, n_steps
   │                               │        gera LinkAdrReq
   │                               │    dl_scheduler.schedule(mac_cmd)
   │◄──── Downlink RX1/RX2 ────────│
   │                               │
_apply_mac_commands()              │
  MACCommandProcessor              │
    _apply_link_adr()              │
      device.sf = new_sf           │
      device.tx_power = new_tx     │
      device.airtime recalculado   │
      retorna LinkAdrAns           │
device.pending_mac_commands        │
  (LinkAdrAns enfileirado p/ UL)   │
```

---

## 1. Arquitetura e Arquivos

| Componente | Arquivo | Responsabilidade |
|---|---|---|
| `ADRComponent` | `network_server/components/adr.py` | Lógica ADR server-side |
| `NetworkController` | `network_server/controller.py` | Orquestra componentes plugáveis |
| `NetworkServer` | `network_server/server.py` | Instancia e registra ADRComponent |
| `Network` | `network.py` | Integra ADR no fluxo de simulação |
| `MACCommandProcessor` | `mac_commands.py` | Aplica `LinkAdrReq` no device via `process_downlink_commands()` |
| `DeviceStatus` | `network_server/device_registry.py` | Mantém `adr_ack_cnt` e `last_packet` por device |
| `parametors.py` | `parametors.py` | Constantes `ADR_HISTORY_SIZE`, `ADR_SNR_MARGIN_DB`, `ADR_STEP_DB` |

---

## 2. Parâmetros Globais

Definidos em `parametors.py`:

```python
adr_enabled = True          # Ativa ADR globalmente na simulação
ADR_HISTORY_SIZE = 20       # Número de pacotes no histórico de SNR
ADR_SNR_MARGIN_DB = 10      # Margem de segurança sobre SNR mínimo (dB)
ADR_STEP_DB = 3             # Tamanho de cada passo de ajuste (dB)
```

SNR mínimo requerido por SF (Semtech LoRa Design Guide / ns-3 / FLoRa):

```python
snr_min_per_sf = {
    12: -20.0,   # SF12 → -20 dB
    11: -17.5,   # SF11 → -17.5 dB
    10: -15.0,   # SF10 → -15 dB
     9: -12.5,   # SF9  → -12.5 dB
     8: -10.0,   # SF8  → -10 dB
     7:  -7.5,   # SF7  → -7.5 dB
}
```

---

## 3. ADRComponent

### 3.1 Instanciação

```python
# Em NetworkServer.__init__ (server.py)
self.adr_component = ADRComponent(method=adr_method)
self.controller.add_component(self.adr_component)
```

**Parâmetros do construtor:**

| Parâmetro | Default | Descrição |
|---|---|---|
| `method` | `"average"` | Política de agregação do histórico SNR |
| `margin_db` | `ADR_SNR_MARGIN_DB` (10) | Margem de segurança em dB |
| `history_size` | `ADR_HISTORY_SIZE` (20) | Janela de histórico de SNR |

### 3.2 Estado Interno

```python
self.device_histories = {}  # device_id -> deque(maxlen=history_size)
```

Um `deque` por device acumula os valores de SNR dos últimos `history_size` pacotes recebidos com sucesso.

---

## 4. Fluxo de Operação

### 4.1 Coleta de SNR — `on_packet()`

Chamado pelo `NetworkController.on_new_packet()` a cada pacote uplink recebido:

```python
def on_packet(self, packet, device_status):
    snr = packet.snr

    if snr is None:          # Pacote sem medição de SNR (ex.: colidido) — ignora
        return []

    if device_id not in self.device_histories:
        self.device_histories[device_id] = deque(maxlen=self.history_size)
    self.device_histories[device_id].append(snr)

    if len(history) < self.history_size:
        return []            # Aguarda dados suficientes (padrão: 20 amostras)
```

O componente só age após acumular `history_size` amostras. Antes disso, retorna lista vazia.

### 4.2 Cálculo da Margem

```python
snr_metric   = self.compute_snr_metric(self.device_histories[device_id])
required_snr = snr_min_per_sf.get(packet.sf, -20.0)   # limiar do SF atual
margin       = snr_metric - required_snr - self.margin_db
n_steps      = int(margin / ADR_STEP_DB)
```

**Exemplo numérico (SF9):**

```
SNR médio = -5.0 dB
required_snr(SF9) = -12.5 dB
margin_db  = 10 dB (segurança)

margin  = -5.0 - (-12.5) - 10.0 = -2.5 dB
n_steps = int(-2.5 / 3) = 0  → sem ajuste
```

```
SNR médio = +3.0 dB
margin  = 3.0 - (-12.5) - 10.0 = +5.5 dB
n_steps = int(5.5 / 3) = 1   → 1 passo de melhoria
```

### 4.3 Algoritmo de Ajuste de SF e TX Power

O ajuste segue a ordem de prioridade definida pela especificação LoRaWAN:

```
margem positiva (n_steps > 0) → link com folga:
  Passo 1: Reduz SF (SF12→SF11→...→SF7), 1 passo por vez
  Passo 2: Reduz TX Power (14→12→...→2 dBm), 2 dBm por passo

margem negativa (n_steps < 0) → link degradado:
  Passo 3: Aumenta TX Power (2→...→14 dBm), 2 dBm por passo
  Passo 4: Aumenta SF (SF7→...→SF12), 1 passo por vez
```

Implementação em `adr.py`:

```python
new_sf       = packet.sf
new_tx_power = packet.tx_power

# Passo 1: Reduz SF (prioridade: SF menor = maior throughput, menor airtime)
while n_steps > 0 and new_sf > 7:
    new_sf -= 1
    n_steps -= 1

# Passo 2: Reduz TX Power (economia de energia)
while n_steps > 0 and new_tx_power > 2:
    new_tx_power -= 2
    n_steps -= 1

# Passo 3: Aumenta TX Power (recupera link degradado, sem custo em airtime)
while n_steps < 0 and new_tx_power < 14:
    new_tx_power += 2
    n_steps += 1

# Passo 4: Aumenta SF (último recurso — penaliza capacity)
while n_steps < 0 and new_sf < 12:
    new_sf += 1
    n_steps += 1
```

**Limites:**
- SF: `[7, 12]`
- TX Power: `[2, 14]` dBm, com passo de 2 dBm

### 4.4 Geração de LinkAdrReq

Se houve mudança em SF ou TX Power:

```python
if new_sf != packet.sf or new_tx_power != packet.tx_power:
    from mac_commands import LinkAdrReq
    dr = 12 - new_sf   # Converte SF para DR: SF7=DR5, SF12=DR0
    return [LinkAdrReq(data_rate=dr, tx_power=new_tx_power, ch_mask=0xFFFF)]

return []
```

A conversão SF → DR segue a definição EU868:

| SF | DR |
|---|---|
| SF7 | DR5 |
| SF8 | DR4 |
| SF9 | DR3 |
| SF10 | DR2 |
| SF11 | DR1 |
| SF12 | DR0 |

---

## 5. Políticas de Agregação do Histórico SNR

Selecionadas via parâmetro `method` ao instanciar `ADRComponent` ou `NetworkServer`:

```python
ns = NetworkServer(adr_method="ewma")
```

| Política | `method` | Descrição | Uso recomendado |
|---|---|---|---|
| Média aritmética | `"average"` | `np.mean(history)` | Default; balanceado |
| Máximo | `"maximum"` | `np.max(history)` | Otimista; favorece SF menor |
| Mínimo | `"minimum"` | `np.min(history)` | Conservador; favorece robustez |
| Percentil 90 | `"percentile"` | `np.percentile(history, 90)` | Remove outliers negativos |
| EWMA | `"ewma"` | α=0.3, média exponencial ponderada | Responsivo a variações recentes |

Implementação em `compute_snr_metric()`:

```python
def compute_snr_metric(self, snr_history):
    history = list(snr_history)
    if not history:
        return -20

    if self.method == "average":
        return float(np.mean(history))
    elif self.method == "maximum":
        return float(np.max(history))
    elif self.method == "minimum":
        return float(np.min(history))
    elif self.method == "percentile":
        return float(np.percentile(history, 90))
    elif self.method == "ewma":
        alpha = 0.3
        result = history[0]
        for snr in history[1:]:
            result = alpha * snr + (1 - alpha) * result
        return float(result)
    else:
        return float(np.mean(history))   # fallback para "average"
```

---

## 6. ADR Backoff

### 6.1 Motivação

Quando um device não recebe nenhum downlink por `ADR_ACK_LIMIT` frames consecutivos, pode ter aumentado o SF demais ou estar fora de alcance. O mecanismo de backoff força o device a aumentar conservadoramente sua potência e SF para restaurar a conectividade.

### 6.2 Constantes

```python
# Em ADRComponent (adr.py)
ADR_ACK_LIMIT = 64   # Uplinks sem DL antes de pedir ADR ACK
ADR_ACK_DELAY = 32   # Frames adicionais antes de aplicar backoff
```

### 6.3 Contador `adr_ack_cnt`

O contador é mantido em dois lugares:

- **`EndDevice`** (`enddevice.py`): `self.adr_ack_cnt = 0` — inicializado no construtor
- **`DeviceStatus`** (`device_registry.py`): `self.adr_ack_cnt = 0` — cópia no NS para `check_backoff()`

**Incremento** (em `network.py`, `_on_device_send()`, após processar MAC commands):
```python
device.adr_ack_cnt += 1
```

**Reset** — sempre que um downlink é entregue com sucesso (RX1 ou RX2), em `_on_rx1_open()` e `_on_rx2_open()`:
```python
device.adr_ack_cnt = 0
```

### 6.4 `check_backoff()`

```python
def check_backoff(self, device_status):
    if device_status.adr_ack_cnt < self.ADR_ACK_LIMIT:
        return None   # Normal, sem ação

    excess = device_status.adr_ack_cnt - self.ADR_ACK_LIMIT
    if excess < self.ADR_ACK_DELAY:
        return None   # Dentro do delay, ainda aguardando

    last_packet = device_status.last_packet
    if last_packet is None:
        return None   # Sem referência de TX power/SF — não há como escalar

    # Backoff ativo: aumenta TX power, depois SF
    new_tx_power = min(last_packet.tx_power + 2, 14)
    new_sf       = last_packet.sf
    if new_tx_power >= 14 and new_sf < 12:
        new_sf += 1

    return new_tx_power, new_sf
```

**Linha do tempo do backoff:**

```
Frame 0          Frame 64         Frame 96         Frame 97+
|                |                |                |
|── normal ──────|── ADR_ACK ─────|── BACKOFF ─────|── +2dBm/SF a cada ciclo ──►
                 (sinaliza pedido) (delay expira)
```

---

## 7. Aplicação no Device — MACCommandProcessor

Após o NS gerar `LinkAdrReq`, o comando é agendado no `DownlinkScheduler`. Na janela RX1 ou RX2, `network.py` chama `_apply_mac_commands(device, mac_commands)`, que instancia um `MACCommandProcessor` e chama `process_downlink_commands()`:

```python
# network.py — _apply_mac_commands()
processor = MACCommandProcessor(network_server=self.ns)
responses = processor.process_downlink_commands(device, mac_commands)
if responses:
    device.pending_mac_commands.extend(responses)   # LinkAdrAns enfileirado para próximo UL
```

Internamente, `process_downlink_commands()` despacha cada `LinkAdrReq` para `_apply_link_adr()`:

```python
def _apply_link_adr(self, device, cmd):
    power_ack    = True
    dr_ack       = True
    ch_mask_ack  = True

    # Aplica TX Power
    new_tx = cmd.payload.get("tx_power", device.tx_power)
    if 2 <= new_tx <= 14:
        device.tx_power = new_tx
    else:
        power_ack = False

    # Aplica Data Rate (SF) e recalcula airtime
    new_dr = cmd.payload.get("data_rate")
    if new_dr is not None:
        new_sf = 12 - new_dr   # DR → SF
        if 7 <= new_sf <= 12:
            device.sf     = new_sf
            device.airtime = device.calculate_airtime()   # recalcula ToA
        else:
            dr_ack = False

    return LinkAdrAns(power_ack=power_ack, data_rate_ack=dr_ack,
                      channel_mask_ack=ch_mask_ack)
```

A resposta `LinkAdrAns` (CID 0x03) é enfileirada em `device.pending_mac_commands` e enviada no próximo uplink confirmando a aceitação dos novos parâmetros.

---

## 8. Integração no Fluxo de Simulação

### 8.1 Inicialização

```python
# network.py — Network.__init__()
self.ns = NetworkServer(region=self.region, adr_method=adr_method)

# Cada device recebe a flag
device.adr_enabled = self.adr_enabled
```

### 8.2 Fluxo completo por uplink

```
_on_device_send(device, packet)
  │
  ├── channel.evaluate_reception(packet, gw)       ← RSSI/SNR/SINR via canal
  ├── best_gateway.process_uplink(packet)
  │
  ├── [Sprint 9] MRC — combina SNR de múltiplos GWs se len(gateways) > 1
  │     └── packet.snr_mrc = 10*log10(Σ 10^(snr_i/10))
  │
  ├── ns.on_uplink_received(packet, best_gateway)
  │     ├── validate_frame_counter()              ← rejeita replay/duplicata
  │     ├── device_registry.update(packet, gw)    ← atualiza last_packet, gateways
  │     ├── controller.on_new_packet(packet, status)
  │     │     └── adr_component.on_packet(packet, status)
  │     │           ├── snr is None → return []
  │     │           ├── atualiza device_histories[device_id]
  │     │           ├── len < history_size → return []
  │     │           ├── compute_snr_metric()
  │     │           ├── calcula margin e n_steps
  │     │           └── retorna [LinkAdrReq(...)] ou []
  │     └── dl_scheduler.schedule(mac_cmd)        ← enfileira para RX1/RX2
  │
  ├── _apply_mac_commands(device, mac_commands)   ← aplica imediatamente via MACCommandProcessor
  │     └── _apply_link_adr(device, LinkAdrReq)
  │           ├── device.tx_power = new_tx
  │           ├── device.sf       = new_sf
  │           ├── device.airtime  = device.calculate_airtime()
  │           └── retorna LinkAdrAns → device.pending_mac_commands
  │
  ├── [legado] if adr_enabled and not server_side_adr:
  │     └── lorawan_mac.process_mac_commands(device)
  │
  └── device.adr_ack_cnt += 1
```

### 8.3 Reset do contador (RX1 / RX2)

```python
# network.py — _on_rx1_open() e _on_rx2_open()
if dl_packet.payload and hasattr(dl_packet.payload, 'cid'):
    self._apply_mac_commands(device, [dl_packet.payload])
device.retransmission_attempts = 0
device.adr_ack_cnt = 0          # ← qualquer downlink zera o contador
```

### 8.4 Modo legado (device-side ADR)

Quando `server_side_adr=False`, o ADR é processado localmente no device sem geração de MAC commands:

```python
# network.py — _on_device_send()
if self.adr_enabled and not self.server_side_adr:
    self.lorawan_mac.process_mac_commands(device)
```

Este modo existe para compatibilidade e testes comparativos. O modo padrão (`server_side_adr=True`) reflete implementações reais de NS como ChirpStack e ns-3.

### 8.5 Toggle em tempo de execução

```python
def toggle_adr(self):
    """Ativa/desativa ADR para toda a rede."""
    self.adr_enabled = adr_enabled   # usa valor do módulo parametors
    for device in self.devices:
        device.adr_enabled = self.adr_enabled
```

---

## 9. MAC Commands Relacionados

| CID | Comando | Direção | Descrição |
|---|---|---|---|
| `0x03` | `LinkAdrReq` | NS → ED | Solicita ajuste de DR, TX Power e channel mask |
| `0x03` | `LinkAdrAns` | ED → NS | Confirma (ou rejeita) o ajuste solicitado |

### LinkAdrReq (CID 0x03)

```python
class LinkAdrReq(MACCommand):
    def __init__(self, data_rate, tx_power, ch_mask, nb_trans=1):
        super().__init__(0x03, {
            "data_rate": data_rate,   # DR0..DR5 (mapeado de SF12..SF7)
            "tx_power":  tx_power,    # 2..14 dBm
            "ch_mask":   ch_mask,     # 0xFFFF = todos os canais ativos
            "nb_trans":  nb_trans,    # repetições por uplink
        })
```

### LinkAdrAns (CID 0x03)

```python
class LinkAdrAns(MACCommand):
    def __init__(self, power_ack=True, data_rate_ack=True, channel_mask_ack=True):
        # Bits ACK: True=aceito, False=rejeitado (parâmetro fora do range suportado)
```

---

## 10. DeviceStatus e Rastreamento no NS

`DeviceStatus` (em `device_registry.py`) mantém o estado ADR do device no lado do NS:

```python
class DeviceStatus:
    def __init__(self, device_id):
        self.adr_ack_cnt          = 0     # Contador para backoff
        self.last_packet          = None  # Último pacote recebido (SF, TX Power, SNR)
        self.gateways             = {}    # gw_id → {rssi, snr, sinr, time}
        self.pending_mac_commands = []
        self.frame_counter_up     = 0
        self.needs_ack            = False
```

O método `get_best_gateway(device_id)` retorna o GW com melhor SNR para o device, garantindo que o `LinkAdrReq` no downlink seja roteado pelo caminho mais confiável.

---

## 11. NetworkController — Plugin Architecture

O `ADRComponent` é um de cinco plugins registrados no `NetworkController` pelo `NetworkServer`:

```python
# server.py — NetworkServer.__init__()
self.adr_component  = ADRComponent(method=adr_method)
self.dc_component   = DutyCycleComponent(region=region)
self.link_check     = LinkCheckComponent()
self.dev_status     = DevStatusComponent()
self.new_channel    = NewChannelComponent()

self.controller.add_component(self.adr_component)
self.controller.add_component(self.dc_component)
self.controller.add_component(self.link_check)
self.controller.add_component(self.dev_status)
self.controller.add_component(self.new_channel)
```

Interface do controller:

```python
class NetworkController:
    def on_new_packet(self, packet, device_status):
        mac_commands = []
        for component in self.components:
            cmds = component.on_packet(packet, device_status)
            if cmds:
                mac_commands.extend(cmds)
        return mac_commands
```

Todos os componentes implementam `on_packet(packet, device_status) -> List[MACCommand]`. O `ADRComponent` é o único que gera `LinkAdrReq`; os demais geram `DutyCycleReq`, `LinkCheckAns`, `DevStatusReq`, e `NewChannelReq` respectivamente.

---

## 12. Referências de Implementação

| Referência | Correspondência no PyLoRaWAN |
|---|---|
| ns-3 `AdrComponent` | `ADRComponent` — mesma lógica de margem e passos |
| LoRaWAN 1.1 Spec §5.2 | `LinkAdrReq` / `LinkAdrAns` (CID 0x03) |
| Semtech AN1200.22 | `snr_min_per_sf`, `ADR_STEP_DB=3`, `ADR_SNR_MARGIN_DB=10` |
| ns-3 `NetworkStatus` | `DeviceRegistry` + `DeviceStatus` |
| ChirpStack ADR | `server_side_adr=True` (NS controla, não o device) |
