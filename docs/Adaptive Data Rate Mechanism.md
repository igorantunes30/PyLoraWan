# Adaptive Data Rate Mechanism — PyLoRaWAN

## Visão Geral

O mecanismo de Adaptive Data Rate (ADR) é implementado exclusivamente no lado do Network Server (`ADRComponent` em `network_server/components/adr.py`). O NS coleta métricas de qualidade de sinal dos uplinkss recebidos, mantém um histórico por device, estima a margem de enlace e ajusta dinamicamente o Spreading Factor e a potência de transmissão de cada End Device via MAC commands no downlink.

Essa abordagem server-side é consistente com a especificação LoRaWAN e com implementações de referência como ns-3 e ChirpStack, permitindo otimização centralizada da rede.

---

## 1. Coleta do Histórico de SNR

A cada pacote uplink recebido, `ADRComponent.on_packet()` armazena o SNR medido em um `deque` por device:

```python
def on_packet(self, packet, device_status):
    snr = packet.snr

    if snr is None:          # pacote sem medição (ex.: colisão) — ignorado
        return []

    if device_id not in self.device_histories:
        self.device_histories[device_id] = deque(maxlen=self.history_size)
    self.device_histories[device_id].append(snr)

    if len(self.device_histories[device_id]) < self.history_size:
        return []            # aguarda history_size amostras antes de agir
```

| Parâmetro | Origem | Default |
|---|---|---|
| `history_size` | `parametors.py` → `ADR_HISTORY_SIZE` | 20 pacotes |
| `margin_db` | `parametors.py` → `ADR_SNR_MARGIN_DB` | 10 dB |
| `ADR_STEP_DB` | `parametors.py` | 3 dB |

O componente só age após acumular exatamente `history_size` amostras válidas (SNR ≠ None).

---

## 2. Estimativa da Margem de Enlace

Com o histórico completo, o componente calcula uma métrica escalar de SNR e compara com o limiar mínimo do SF atual:

```python
snr_metric   = self.compute_snr_metric(self.device_histories[device_id])
required_snr = snr_min_per_sf.get(packet.sf, -20.0)

margin  = snr_metric - required_snr - self.margin_db
n_steps = int(margin / ADR_STEP_DB)
```

**SNR mínimo por SF** (`snr_min_per_sf` em `parametors.py`):

| SF | SNR mínimo |
|---|---|
| SF12 | −20.0 dB |
| SF11 | −17.5 dB |
| SF10 | −15.0 dB |
| SF9 | −12.5 dB |
| SF8 | −10.0 dB |
| SF7 | −7.5 dB |

A `margin_db = 10 dB` garante uma folga de segurança acima do limiar mínimo antes de qualquer redução de SF.

**Exemplo numérico** (device em SF9, SNR médio = −5.0 dB):

```
margin  = −5.0 − (−12.5) − 10.0 = −2.5 dB  →  n_steps = 0  (sem ajuste)
```

**Exemplo com margem suficiente** (SF9, SNR médio = +3.0 dB):

```
margin  = 3.0 − (−12.5) − 10.0 = +5.5 dB  →  n_steps = 1  (1 passo de melhoria)
```

---

## 3. Políticas de Agregação do Histórico

A métrica escalar de SNR é calculada por `compute_snr_metric()` segundo a política configurada no momento da instanciação:

```python
ns = NetworkServer(adr_method="ewma")   # padrão: "average"
```

| Política | `method` | Cálculo |
|---|---|---|
| Média aritmética | `"average"` | `np.mean(history)` |
| Máximo | `"maximum"` | `np.max(history)` |
| Mínimo | `"minimum"` | `np.min(history)` |
| Percentil 90 | `"percentile"` | `np.percentile(history, 90)` |
| EWMA | `"ewma"` | α = 0.3; `result = α×snr + (1−α)×result` para cada amostra |

Qualquer valor de `method` não reconhecido recai em `np.mean` como fallback.

---

## 4. Algoritmo de Ajuste de Parâmetros

O número de passos `n_steps` determina a direção e magnitude do ajuste. A ordem de prioridade segue a especificação LoRaWAN:

```python
new_sf       = packet.sf
new_tx_power = packet.tx_power

# Margem positiva — link com folga
while n_steps > 0 and new_sf > 7:        # Passo 1: reduz SF
    new_sf   -= 1
    n_steps  -= 1

while n_steps > 0 and new_tx_power > 2:  # Passo 2: reduz TX power
    new_tx_power -= 2
    n_steps      -= 1

# Margem negativa — link degradado
while n_steps < 0 and new_tx_power < 14: # Passo 3: aumenta TX power
    new_tx_power += 2
    n_steps      += 1

while n_steps < 0 and new_sf < 12:       # Passo 4: aumenta SF
    new_sf  += 1
    n_steps += 1
```

**Limites dos parâmetros:**

| Parâmetro | Mínimo | Máximo | Passo |
|---|---|---|---|
| SF | 7 | 12 | 1 |
| TX Power | 2 dBm | 14 dBm | 2 dBm |

**Prioridade de ajuste:**

- Margem positiva: reduz SF primeiro (aumenta throughput, reduz airtime e ocupação do canal), depois reduz TX power (economia de energia).
- Margem negativa: aumenta TX power primeiro (recupera enlace sem penalizar capacity), depois aumenta SF como último recurso.

---

## 5. Geração de `LinkAdrReq`

Se houver mudança em SF ou TX power, o componente retorna um `LinkAdrReq` para ser enfileirado no downlink scheduler:

```python
if new_sf != packet.sf or new_tx_power != packet.tx_power:
    dr = 12 - new_sf    # SF → DR: SF12=DR0, SF7=DR5
    return [LinkAdrReq(data_rate=dr, tx_power=new_tx_power, ch_mask=0xFFFF)]

return []
```

O campo `ch_mask=0xFFFF` mantém todos os canais disponíveis ativos. A conversão SF → DR segue a tabela EU868:

| SF | DR |
|---|---|
| SF12 | DR0 |
| SF11 | DR1 |
| SF10 | DR2 |
| SF9 | DR3 |
| SF8 | DR4 |
| SF7 | DR5 |

O `LinkAdrReq` gerado é entregue ao device na janela RX1 ou RX2 do próximo ciclo de transmissão.

---

## 6. Aplicação no End Device

Ao receber o `LinkAdrReq` no downlink, `network.py` chama `_apply_mac_commands()`, que instancia um `MACCommandProcessor` e aplica o comando via `_apply_link_adr()`:

```python
# _apply_link_adr() em mac_commands.py
new_tx = cmd.payload.get("tx_power", device.tx_power)
if 2 <= new_tx <= 14:
    device.tx_power = new_tx

new_dr = cmd.payload.get("data_rate")
if new_dr is not None:
    new_sf = 12 - new_dr
    if 7 <= new_sf <= 12:
        device.sf     = new_sf
        device.airtime = device.calculate_airtime()   # recalcula ToA imediatamente
```

O método usa `.get()` com fallback — acesso seguro sem `KeyError`. Após mudança de SF, `device.airtime` é recalculado imediatamente para manter o campo em sincronia com os novos parâmetros.

A resposta `LinkAdrAns(power_ack, data_rate_ack, channel_mask_ack)` é enfileirada em `device.pending_mac_commands` e enviada no próximo uplink.

---

## 7. ADR Backoff

Quando o device não recebe nenhum downlink por um período prolongado (SF pode ter sido reduzido além do alcance real), o mecanismo de backoff aumenta conservadoramente os parâmetros para restaurar a conectividade.

**Constantes em `ADRComponent`:**

```python
ADR_ACK_LIMIT = 64   # uplinks consecutivos sem downlink → sinaliza pedido de ACK
ADR_ACK_DELAY = 32   # frames adicionais após ADR_ACK_LIMIT → aplica backoff
```

**Contador `adr_ack_cnt`:**
- Incrementado em `network.py` após cada `_on_device_send()`.
- Zerado em `_on_rx1_open()` ou `_on_rx2_open()` sempre que um downlink é entregue com sucesso.

**`check_backoff()` em `adr.py`:**

```python
def check_backoff(self, device_status):
    if device_status.adr_ack_cnt < self.ADR_ACK_LIMIT:
        return None          # operação normal

    excess = device_status.adr_ack_cnt - self.ADR_ACK_LIMIT
    if excess < self.ADR_ACK_DELAY:
        return None          # dentro do período de delay

    last_packet = device_status.last_packet
    if last_packet is None:
        return None          # sem referência de parâmetros anteriores

    new_tx_power = min(last_packet.tx_power + 2, 14)
    new_sf = last_packet.sf
    if new_tx_power >= 14 and new_sf < 12:
        new_sf += 1

    return new_tx_power, new_sf
```

**Linha do tempo:**

```
Frame 0          Frame 64              Frame 96              Frame 97+
│                │                     │                     │
│── operação ────│── ADR_ACK sinaliz. ─│── backoff ativo ────│── +2 dBm ou +1 SF ──►
   normal          (sem DL por 64 ULs)   (delay expirou)
```

O backoff prioriza aumento de TX power antes de aumentar SF — mesma ordem do algoritmo principal.

---

## 8. Fluxo Completo

```
Uplink recebido pelo NS
  │
  ├── on_uplink_received(packet, gateway)
  │     ├── validate_frame_counter()
  │     ├── device_registry.update(packet, gw)     → atualiza last_packet, gateways
  │     └── controller.on_new_packet(packet, status)
  │           └── adr_component.on_packet(packet, status)
  │                 ├── snr is None? → return []
  │                 ├── device_histories[id].append(snr)
  │                 ├── len < history_size? → return []
  │                 ├── snr_metric = compute_snr_metric(history)
  │                 ├── required_snr = snr_min_per_sf[packet.sf]
  │                 ├── margin = snr_metric − required_snr − margin_db
  │                 ├── n_steps = int(margin / ADR_STEP_DB)
  │                 ├── ajusta new_sf e new_tx_power (4 passos)
  │                 └── retorna [LinkAdrReq(dr, tx_power, ch_mask)] ou []
  │
  └── dl_scheduler.schedule(LinkAdrReq)   → entrega em RX1 ou RX2
        │
        └── _apply_mac_commands(device, [LinkAdrReq])
              └── _apply_link_adr(device, cmd)
                    ├── device.tx_power = new_tx
                    ├── device.sf       = new_sf
                    ├── device.airtime  = device.calculate_airtime()
                    └── retorna LinkAdrAns → device.pending_mac_commands
```

---

## 9. Parâmetros Configuráveis

| Parâmetro | Arquivo | Default | Efeito |
|---|---|---|---|
| `ADR_HISTORY_SIZE` | `parametors.py` | 20 | Tamanho da janela de histórico SNR |
| `ADR_SNR_MARGIN_DB` | `parametors.py` | 10 dB | Margem de segurança sobre SNR mínimo |
| `ADR_STEP_DB` | `parametors.py` | 3 dB | Tamanho de cada passo de ajuste |
| `adr_method` | `NetworkServer(adr_method=)` | `"average"` | Política de agregação do histórico |
| `ADR_ACK_LIMIT` | `adr.py` | 64 frames | Uplinks sem DL antes de sinalizar backoff |
| `ADR_ACK_DELAY` | `adr.py` | 32 frames | Frames adicionais antes de aplicar backoff |
