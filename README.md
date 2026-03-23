# PyLoRaWAN Simulator — Como Executar

## Requisitos

- Python 3.10+
- numpy
- matplotlib

```bash
pip install numpy matplotlib
```

Sem outras dependencias. Nao requer compilacao, OMNeT++ ou ns-3.

---

## Execucao Rapida

```bash
cd /home/igor/Pylorawan
python3 network.py
```

Executa o cenario padrao: **50 devices, 1 gateway, area 10km x 10km, 1 hora**.

Saida esperada (~90s):

```
==================================================
Regiao:                        EU868
Dispositivos Cobertos:         41 / 50
Total de Pacotes:              3443
Retransmissoes:                194 (5.63%)
Colisoes:                      0
Pacotes Entregues:             3249
PDR (Packet Delivery Ratio):   90.27%
Vazao Total de Dados:          1764.97 kbps
GW Saturation Events:          0
==================================================
```

Arquivos gerados:

| Arquivo | Conteudo |
|---------|---------|
| `statistics.png` | SF distribution, energia, SNR (grafico basico) |
| `mobility.png` | Mapa de posicoes dos devices por SF |
| `results.json` | Metricas completas em JSON |
| `results_detailed.csv` | Log de cada pacote (RSSI, SINR, SF, colisao) |
| `results.npz` | Dados binarios compactos (numpy), carregavel com `numpy.load()` |
| `device_summary.csv` | Resumo por device (energia, SF, PDR) |
| `device_log.csv` | Log de eventos de TX/RX por device |
| `plot_pdr_per_sf.png` | PDR (%) por Spreading Factor |
| `plot_pdr_vs_distance.png` | PDR vs distancia ao gateway |
| `plot_energy_per_sf.png` | Energia media por SF |
| `plot_delay_cdf.png` | CDF dos delays de recepcao |
| `plot_sinr_distribution.png` | Histograma de SINR |
| `plot_collision_heatmap.png` | Heatmap espacial de colisoes |
| `plot_throughput_over_time.png` | Throughput ao longo do tempo |
| `plot_sf_distribution.png` | Distribuicao de SFs (expandido) |

---

## Configuracao

Todos os parametros ficam em **`parametors.py`**. Edite antes de rodar.

### Rede

```python
num_devices = 50          # Numero de end devices
num_gateways = 1          # Numero de gateways
area_size = 10000         # Area em metros (10000 = 10km x 10km)
simulation_time = 3600    # Duracao em segundos (3600 = 1 hora)
random_seed = 42          # Seed (None = aleatorio a cada execucao)
```

### PHY LoRa

```python
sf_range = [7, 8, 9, 10, 11, 12]   # SFs permitidos (ADR escolhe o minimo viavel)
tx_power = 14                        # Potencia TX inicial (dBm), max EU868
bw = [125000]                        # Largura de banda (Hz): 125000, 250000, 500000
cr = 1                               # Coding Rate: 1=4/5, 2=4/6, 3=4/7, 4=4/8
pl = 20                              # Payload (bytes)
frequency_mhz = [868.1, 868.3, 868.5]  # Canais EU868
```

### Modelos de Propagacao

```python
model_pathloss = "log_normal_shadowing"  # Padrao (compativel FLoRa/ns-3)
```

| Valor | Descricao |
|-------|-----------|
| `"log_normal_shadowing"` | Log-Normal com shadowing (padrao, compativel FLoRa) |
| `"okumura_hata"` | Okumura-Hata urbano |
| `"cost_hata"` | COST-Hata |
| `"fspl"` | Free-Space Path Loss |
| `"log_distance"` | Log-Distance |
| `"correlated_shadowing"` | Shadowing espacialmente correlacionado (como ns-3) |
| `"building_penetration"` | Log-Normal + perda indoor |
| `"fading"` | COST-Hata + fading (Rayleigh/Rician/Nakagami) |

### Mobilidade

```python
mobility_enabled = True
model = "random_walk"    # Modelo de mobilidade
speed = 1.5              # Velocidade (m/s)
lambda_rate = 0.1        # Taxa de transmissao (Hz) — media de 10s entre TX
```

| `model` | Descricao |
|---------|-----------|
| `"random_walk"` | Caminhada aleatoria (padrao) |
| `"gauss_markov"` | Gauss-Markov (direcao correlacionada) |
| `"levy_walk"` | Levy Walk (padroes com pauses) |
| `"matrix"` | Grade discreta |

### Deployment (Posicionamento dos Devices)

Configuravel no `__main__` de `network.py`:

```python
network = Network(..., deployment_type="circular")
```

| `deployment_type` | Descricao | Parametro extra |
|-------------------|-----------|-----------------|
| `"grid"` | Grade regular (padrao) | — |
| `"circular"` | Disco uniforme (como FLoRa) | `deployment_radius=` |
| `"annular"` | Anel entre dois raios | `deployment_radius=` |
| `"hexagonal"` | Grade hexagonal (como ns-3) | — |
| `"random_uniform"` | Uniforme na area | — |
| `"clustered"` | Clusters gaussianos | — |
| `"from_file"` | Importar posicoes de CSV | `deployment_file="posicoes.csv"` |

Para `from_file`, o CSV deve ter colunas `x,y` (header opcional):

```csv
x,y
1000,2000
3500,4200
...
```

### Parametros Regionais

```python
# Em network.py __main__, passar region_name:
network = Network(..., region_name="EU868")
```

| Regiao | Banda | Duty Cycle | Canais |
|--------|-------|------------|--------|
| `"EU868"` | 863–870 MHz | 1% | 3 obrigatorios + opcionais |
| `"US915"` | 902–928 MHz | Dwell time 400ms | 64+8 canais |
| `"AU915"` | 915–928 MHz | Dwell time 400ms | 64+8 canais |
| `"AS923"` | 923 MHz | 1% | 2 canais base |

### Interferencia

```python
interference_model = "semtech"   # ou "goursaud"
```

| Modelo | Co-SF threshold | Referencia |
|--------|----------------|------------|
| `"semtech"` | 1 dB (mais permissivo) | Semtech AN1200.18 / LoRaWANSim |
| `"goursaud"` | 6 dB (mais conservador) | ns-3 default (Goursaud et al.) |

### ADR

```python
adr_enabled = True       # Liga/desliga ADR
```

ADR e server-side (Network Server envia `LinkAdrReq` via downlink). Historico de 20 pacotes, margem de 10 dB, passo de 3 dB.

### LR-FHSS

```python
lrfhss_ratio = 0.0       # 0.0 = desativado | 0.5 = 50% dos devices usam LR-FHSS
lrfhss_code_rate = "1/3" # Code rate: "1/3", "2/3", "1/2"
lrfhss_obw = 35          # Occupied Bandwidth (numero de canais de hopping)
```

Com `lrfhss_ratio > 0`, o relatorio final exibe PDR separado por PHY:

```
[CSS] Pacotes:              53 | PDR: 90.6%
[LR-FHSS] Pacotes:         16 | PDR: 100.0%
[LR-FHSS] Colisoes frag:   0
```

### Energia e Bateria

Para simular bateria com esgotamento:

```python
# Em network.py __main__:
network = Network(..., battery_capacity_mah=2000)
```

Para energy harvesting (solar):

```python
network = Network(...,
    battery_capacity_mah=2000,
    energy_harvesting={"model": "solar", "peak_power_mw": 100}
)
```

---

## Cenarios de Exemplo

### Cenario 1 — Referencia (igual ao FLoRa/ns-3)

```python
# parametors.py
num_devices = 50
num_gateways = 1
area_size = 10000
simulation_time = 3600
model_pathloss = "log_normal_shadowing"
adr_enabled = True
lrfhss_ratio = 0.0
interference_model = "semtech"
random_seed = 42
```

```bash
python3 network.py
```

### Cenario 2 — Alta Densidade com Goursaud

```python
num_devices = 200
num_gateways = 4
interference_model = "goursaud"
model_pathloss = "correlated_shadowing"
```

### Cenario 3 — CSS + LR-FHSS Comparativo

```python
num_devices = 100
lrfhss_ratio = 0.5         # 50 devices CSS, 50 LR-FHSS
lrfhss_code_rate = "1/3"
```

### Cenario 4 — US915 com Mobilidade Intensa

```python
# Em network.py __main__:
network = Network(..., region_name="US915")

# Em parametors.py:
speed = 5.0
model = "gauss_markov"
mobility_enabled = True
```

### Cenario 5 — Bateria com Harvesting

```python
# Em network.py __main__:
network = Network(...,
    battery_capacity_mah=2000,
    energy_harvesting={"model": "solar", "peak_power_mw": 100}
)
```

### Cenario 6 — Deploy Circular com Posicoes de Arquivo

```python
# Em network.py __main__:
network = Network(...,
    deployment_type="from_file",
    deployment_file="minhas_posicoes.csv"
)
```

---

## Testes

```bash
# Todos os testes (200 testes)
python3 -m pytest tests/ -v

# Por modulo
python3 -m pytest tests/test_channel.py -v     # Canal e interferencia (G14)
python3 -m pytest tests/test_lrfhss.py -v      # LR-FHSS e ACRDA
python3 -m pytest tests/test_energy.py -v      # Modelo de energia
python3 -m pytest tests/test_regions.py -v     # Parametros regionais
python3 -m pytest tests/test_toa.py -v         # Time-on-Air
python3 -m pytest tests/test_pathloss.py -v    # Modelos de propagacao
python3 -m pytest tests/test_integration.py -v # Integracao end-to-end
python3 -m pytest tests/test_sprint7.py -v     # Deploy/NPZ/plots/metricas (Sprint 7)
```

---

## Estrutura de Arquivos

```
Pylorawan/
├── parametors.py          # Configuracao global (edite aqui)
├── network.py             # Orquestrador principal — execute este
├── enddevice.py           # End Device com FSM Classe A + mobilidade
├── gateway.py             # Gateway SX1301 (8 reception paths)
├── channel.py             # Modelo de canal (SINR, energy-based G14)
├── lrfhss.py              # LR-FHSS PHY + ACRDA/SIC
├── regions.py             # EU868, US915, AU915, AS923
├── energymodel.py         # Modelo de energia state-based
├── battery.py             # Bateria + energy harvesting
├── deployment.py          # Estrategias de posicionamento (6 tipos + from_file)
├── analytics.py           # Metricas, JSON/CSV/NPZ, modelo analitico Ps1/Ps2
├── plots.py               # Graficos expandidos (8 tipos + plot_all)
├── mac_commands.py        # MAC commands LoRaWAN 1.1
├── security.py            # OTAA, MIC, session keys
├── network_server/        # Network Server (ADR server-side, downlink)
│   ├── server.py
│   ├── components/adr.py
│   └── scheduler.py
└── tests/                 # 200 testes pytest
    ├── test_channel.py
    ├── test_lrfhss.py
    ├── test_energy.py
    ├── test_regions.py
    ├── test_toa.py
    ├── test_pathloss.py
    ├── test_integration.py
    ├── test_sprint7.py
    ├── test_sprint8.py
    ├── test_sprint9.py
    └── test_sprint10.py
```

---

## Comparacao com Outros Simuladores

| Feature | FLoRa | ns-3 | LoRaWANSim | LR-FHSS-sim | **PyLoRaWAN** |
|---------|:-----:|:----:|:----------:|:-----------:|:-------------:|
| Mobilidade (4 modelos) | — | — | — | — | **SIM** |
| LR-FHSS + CSS no mesmo sim | — | — | — | — | **SIM** |
| 8 modelos de propagacao | 3 | 2 | 1 | — | **8** |
| ADR server-side | SIM | SIM | SIM | — | SIM |
| FSM Classe A completa | SIM | SIM | Parcial | — | SIM |
| Gateway SX1301 (8 paths) | Parcial | SIM | — | — | SIM |
| Interferencia energy-based | — | SIM | — | — | SIM (G14) |
| Matriz Goursaud | — | SIM | — | — | SIM |
| Building penetration | — | SIM | — | — | SIM |
| Correlated shadowing | — | SIM | — | — | SIM |
| Parametros EU868/US915/AU915/AS923 | EU868 | Todos | Parcial | — | **Todos** |
| 7 estrategias de deployment | — | Parcial | — | — | **SIM** |
| Saida JSON + CSV + NPZ + 8 graficos | — | — | — | — | **SIM** |
| Python puro (sem build) | — | — | — | SIM | **SIM** |
| Testes automatizados | — | — | — | — | **200 testes** |
