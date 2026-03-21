# ============================================================
# PyLoRaWAN Simulator - Parametros de Configuracao
# ============================================================

# --- Rede ---
num_devices = 50          # Numero de end devices LoRa
num_gateways = 1          # Numero de gateways
area_size = 10000         # Dimensao da area em metros (10000m = 10km x 10km)
                          # BUG-09 FIX: antes era 100 (km), agora em metros para consistencia

# --- Mobilidade ---
lambda_rate = 0.1         # Taxa de movimentacao (Hz) — media de 10s entre movimentos
speed = 1.5               # Velocidade em m/s (BUG-09 FIX: antes 6 km/s, agora realista)
mobility_enabled = True
model = "random_walk"     # "gauss_markov" "levy_walk" "random_walk" "matrix"

# --- LoRa PHY ---
sf_range = [7, 8, 9, 10, 11, 12]
tx_power = 14             # Potencia de transmissao inicial (dBm) — max EU868
frequency_mhz = [868.1, 868.3, 868.5]  # EU868 canais default (MHz)
ht_m = 1.5                # Altura do device (metros) — nivel do chao
hr_m = 30                 # Altura do gateway (metros) — torre/telhado
bw = [125000]             # Largura de banda: 125000, 250000, 500000 Hz
cr = 1                    # Coding Rate (1=4/5, 2=4/6, 3=4/7, 4=4/8)
pl = 20                   # Payload em bytes (20B default, compativel com FLoRa/ns-3)

# --- Simulacao ---
simulation_time = 3600    # Duracao em segundos (1 hora, compativel com cenario de referencia)
adr_enabled = True
model_pathloss = "log_normal_shadowing"  # Modelo de propagacao default
# Opcoes validas: okumura_hata, log_distance, fspl, cost_hata,
#                 log_normal_shadowing, correlated_shadowing,
#                 building_penetration, fading, oulu

# --- Ruido ---
k = 1.38e-23              # Constante de Boltzmann (J/K)
temperatura = 294.15      # Temperatura de referencia (K) — ~21C
noise_figure = 6          # Fator de ruido (dB) — SX1272 tipico

# --- Seed ---
random_seed = 42          # Seed para reproducibilidade (None = aleatorio)

# --- Duty Cycle (%) ---
ed_dc_limit_percent = 1       # ED uplink: 1%
gw_rx1_dc_limit_percent = 1   # GW RX1 downlink: 1%
gw_rx2_dc_limit_percent = 10  # GW RX2 downlink: 10%

# --- Janelas de Recepcao ---
receive_delay1 = 1        # Delay antes da janela RX1 (s)
receive_delay2 = 2        # Delay antes da janela RX2 (s)
rd1 = 1                   # Periodo de dormencia antes da recepcao (s)
rd2 = 1                   # Periodo de dormencia depois da recepcao (s)
trx1 = 1                  # Duracao da janela RX1 (s)
trx2 = 2                  # Duracao da janela RX2 (s)

# --- Energia ---
voltage = 3.3             # Tensao de alimentacao (V)
tx_current_table = {      # Corrente TX por potencia (mA) — RN2483 datasheet
    14: 38, 12: 35.1, 10: 32.4, 8: 30, 6: 27.5, 4: 24.7, 2: 22.3
}
rx_current_ma = 11.2      # Corrente RX (mA) — SX1272 (compativel ns-3/FLoRa)
rx_delay_current_ma = 1.4 # Corrente durante RX delay / standby (mA)
sleep_current_ma = 0.0015 # Corrente sleep (mA) — SX1272 (compativel ns-3/FLoRa)

# --- Ganho de Antena (dBi) --- BUG-08 FIX: agora usados no calculo de RSSI
gw_antenna_gain = 3       # Ganho antena gateway (dBi) — tipico omnidirecional
ed_antenna_gain = 0       # Ganho antena end device (dBi) — chip antenna

# --- Interferencia ---
# Modelo de interferencia: "semtech" (Semtech AN1200.18) ou "goursaud" (ns-3 default)
interference_model = "semtech"

# Matriz 6x6 — Semtech AN1200.18 (LoRaWANSim, PyLoRaWAN default)
# Rows/Cols: SF12, SF11, SF10, SF9, SF8, SF7
# Valor = limiar de razao energia_sinal/energia_interferente (dB) para sobreviver
interference_matrix = [
    [ 1,  -23, -24, -25, -25, -25],  # SF12 (alvo)
    [-20,   1, -20, -21, -22, -22],  # SF11
    [-18, -17,   1, -17, -18, -19],  # SF10
    [-15, -14, -13,   1, -13, -15],  # SF9
    [-13, -13, -12, -11,   1, -11],  # SF8
    [ -9,  -9,  -9,  -9,  -8,   1],  # SF7
]

# Matriz 6x6 — Goursaud et al. (ns-3 default, mais conservador)
# Co-SF: threshold 6 dB (captura requer sinal dominante); cross-SF: atenuacao maior
interference_matrix_goursaud = [
    [  6, -36, -36, -36, -36, -36],  # SF12 (alvo)
    [-29,   6, -33, -33, -33, -33],  # SF11
    [-28, -26,   6, -30, -30, -30],  # SF10
    [-27, -25, -23,   6, -27, -27],  # SF9
    [-24, -22, -20, -19,   6, -24],  # SF8
    [-20, -16, -18, -19, -16,   6],  # SF7
]

# --- SNR minimo por SF (dB) — LoRa Design Guide / ns-3 / FLoRa ---
snr_min_per_sf = {12: -20, 11: -17.5, 10: -15, 9: -12.5, 8: -10, 7: -7.5}

# --- Sensibilidade por SF @ 125kHz (dBm) — SX1272 / ns-3 / FLoRa ---
sensitivity_table = {
    (7, 125000): -124.0,  (8, 125000): -127.0,  (9, 125000): -130.0,
    (10, 125000): -133.0, (11, 125000): -135.5, (12, 125000): -137.0,
    (7, 250000): -121.0,  (8, 250000): -124.0,  (9, 250000): -127.0,
    (10, 250000): -130.0, (11, 250000): -132.5, (12, 250000): -134.0,
    (7, 500000): -118.0,  (8, 500000): -121.0,  (9, 500000): -124.0,
    (10, 500000): -127.0, (11, 500000): -129.5, (12, 500000): -131.0,
}

# --- Sensibilidade GW por SF (dBm) — SX1301 (~6 dB melhor que SX1272) ---
# Referencia: ns-3 LoRaWAN gateway-lora-phy.cc (G10 Sprint 2)
gw_sensitivity_table = {
    (7, 125000): -130.0,  (8, 125000): -132.5,  (9, 125000): -135.0,
    (10, 125000): -137.5, (11, 125000): -140.0, (12, 125000): -142.5,
    (7, 250000): -127.0,  (8, 250000): -129.5,  (9, 250000): -132.0,
    (10, 250000): -134.5, (11, 250000): -137.0, (12, 250000): -139.0,
    (7, 500000): -124.0,  (8, 500000): -126.5,  (9, 500000): -129.0,
    (10, 500000): -131.5, (11, 500000): -134.0, (12, 500000): -136.0,
}

# --- ADR ---
ADR_HISTORY_SIZE = 20     # Tamanho do historico de SNR
ADR_SNR_MARGIN_DB = 10    # Margem de seguranca (dB)
ADR_STEP_DB = 3           # Tamanho do passo ADR (dB)

# --- LR-FHSS (Sprint 6) ---
lrfhss_ratio = 0.0        # Fracao de devices usando LR-FHSS (0.0=desativado, 0.5=metade)
lrfhss_code_rate = "1/3"  # Code rate LR-FHSS: "1/3", "2/3", "1/2"
lrfhss_obw = 35           # Occupied Bandwidth (numero de canais de hopping)
lrfhss_num_headers = 3    # Numero de headers por pacote LR-FHSS
lrfhss_acrda_window = 3   # Tamanho da janela ACRDA (multiplos do periodo de transmissao)
