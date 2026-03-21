"""Modulo de seguranca LoRaWAN — OTAA key derivation e MIC.

Implementa conforme LoRaWAN 1.0.x spec:
- Derivacao de NwkSKey e AppSKey a partir do AppKey
- Calculo e verificacao de MIC (Message Integrity Code)
- Geracao de JoinRequest MIC

Usa pycryptodome se disponivel (AES-128 real); caso contrario,
fallback automatico para HMAC-SHA256 truncado (stdlib only).
O fallback preserva todas as propriedades estruturais necessarias
para simulacao (autenticacao, unicidade, aleatoriedade).
"""

import struct
import hmac
import hashlib
import os


def _aes128_encrypt(key, data):
    """AES-128 ECB encrypt — 16 bytes in, 16 bytes out.

    Usa pycryptodome se disponivel; fallback para HMAC-SHA256 truncado.
    """
    try:
        from Crypto.Cipher import AES  # pycryptodome
        return AES.new(key[:16], AES.MODE_ECB).encrypt(data[:16].ljust(16, b'\x00'))
    except ImportError:
        return hmac.new(key[:16], data[:16].ljust(16, b'\x00'), hashlib.sha256).digest()[:16]


def _aes128_cmac(key, msg):
    """AES-128 CMAC — retorna 16 bytes.

    Usa pycryptodome se disponivel; fallback para HMAC-SHA256.
    """
    try:
        from Crypto.Hash import CMAC
        from Crypto.Cipher import AES
        c = CMAC.new(key[:16], ciphermod=AES)
        c.update(msg)
        return c.digest()
    except ImportError:
        return hmac.new(key[:16], msg, hashlib.sha256).digest()[:16]


def generate_app_key():
    """Gera AppKey aleatoria de 16 bytes."""
    return os.urandom(16)


def generate_dev_eui(device_id):
    """Gera DevEUI deterministico (8 bytes) a partir do device_id."""
    return struct.pack('>Q', 0x0102030405060000 | (device_id & 0xFFFF))


def generate_app_eui():
    """AppEUI padrao para simulacao (8 bytes)."""
    return bytes([0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x00, 0x00, 0x00])


def derive_session_keys(app_key, app_nonce, net_id, dev_nonce):
    """Deriva NwkSKey e AppSKey conforme LoRaWAN 1.0.x spec.

    NwkSKey = AES128(AppKey, 0x01 | AppNonce(3B) | NetID(3B) | DevNonce(2B) | 0x00..00)
    AppSKey = AES128(AppKey, 0x02 | AppNonce(3B) | NetID(3B) | DevNonce(2B) | 0x00..00)

    Args:
        app_key:   bytes (16)
        app_nonce: int (3 bytes)
        net_id:    int (3 bytes)
        dev_nonce: int (2 bytes)

    Returns:
        (nwk_skey, app_skey): tuple de bytes (16, 16)
    """
    def _build_input(buf_type):
        data = bytes([buf_type])
        data += struct.pack('<I', app_nonce)[:3]
        data += struct.pack('<I', net_id)[:3]
        data += struct.pack('<H', dev_nonce)
        return data.ljust(16, b'\x00')[:16]

    nwk_skey = _aes128_encrypt(app_key, _build_input(0x01))
    app_skey = _aes128_encrypt(app_key, _build_input(0x02))
    return nwk_skey, app_skey


def compute_join_mic(app_key, app_eui, dev_eui, dev_nonce):
    """Calcula MIC do JoinRequest (4 bytes).

    msg = MHDR(0x00) | AppEUI | DevEUI | DevNonce
    MIC = AES128_CMAC(AppKey, msg)[0:4]
    """
    msg = bytes([0x00]) + app_eui[:8] + dev_eui[:8] + struct.pack('<H', dev_nonce)
    return _aes128_cmac(app_key, msg)[:4]


def compute_frame_mic(nwk_skey, dev_addr, frame_counter, direction, payload_bytes):
    """Calcula MIC de data frame LoRaWAN (4 bytes).

    B0 = 0x49 | 0x00*4 | Dir(1B) | DevAddr(4B) | FCnt(4B) | 0x00 | len(payload)(1B)
    MIC = AES128_CMAC(NwkSKey, B0 | payload)[0:4]

    Args:
        nwk_skey:      bytes (16)
        dev_addr:      int (32-bit device address)
        frame_counter: int
        direction:     0=uplink, 1=downlink
        payload_bytes: bytes
    """
    b0 = bytes([0x49]) + b'\x00' * 4
    b0 += bytes([direction & 0x01])
    b0 += struct.pack('<I', dev_addr & 0xFFFFFFFF)
    b0 += struct.pack('<I', frame_counter & 0xFFFFFFFF)
    b0 += b'\x00'
    b0 += bytes([len(payload_bytes) & 0xFF])
    return _aes128_cmac(nwk_skey, b0 + payload_bytes)[:4]


def verify_frame_mic(nwk_skey, dev_addr, frame_counter, direction, payload_bytes, mic):
    """Verifica MIC de um data frame. Retorna True se valido."""
    computed = compute_frame_mic(nwk_skey, dev_addr, frame_counter, direction, payload_bytes)
    return computed == bytes(mic)[:4]
