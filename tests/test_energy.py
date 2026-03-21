"""Testes de modelo de energia e bateria."""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest
from energymodel import EnergyModel, RadioState
from battery import BatteryModel, EnergyHarvester


class TestEnergyModel:
    def test_initial_state(self):
        em = EnergyModel()
        assert em.energy_consumed == 0
        assert em.current_state == RadioState.SLEEP

    def test_transition_consumes_energy(self):
        em = EnergyModel(voltage=3.3)
        em.transition(RadioState.TX, 0, tx_power=14)
        energy = em.transition(RadioState.SLEEP, 1.0)  # 1 segundo de TX
        assert energy > 0
        assert em.energy_consumed > 0

    def test_sleep_consumes_very_little(self):
        em = EnergyModel(voltage=3.3)
        em.transition(RadioState.SLEEP, 0)
        em.transition(RadioState.SLEEP, 3600)  # 1 hora de sleep
        # 0.0015 mA * 3.3V * 3600s = 17.82 mJ
        assert em.energy_consumed < 20  # mJ

    def test_tx_consumes_more_than_rx(self):
        em_tx = EnergyModel(voltage=3.3)
        em_tx.transition(RadioState.TX, 0, tx_power=14)
        em_tx.transition(RadioState.SLEEP, 1.0)

        em_rx = EnergyModel(voltage=3.3)
        em_rx.transition(RadioState.RX, 0)
        em_rx.transition(RadioState.SLEEP, 1.0)

        assert em_tx.energy_consumed > em_rx.energy_consumed

    def test_energy_breakdown(self):
        em = EnergyModel()
        em.transition(RadioState.TX, 0, tx_power=14)
        em.transition(RadioState.RX, 0.1)
        em.transition(RadioState.SLEEP, 0.2)
        em.transition(RadioState.SLEEP, 1.0)
        breakdown = em.get_energy_breakdown()
        assert "TX" in breakdown
        assert "RX" in breakdown
        assert "SLEEP" in breakdown


class TestBatteryModel:
    def test_initial_full(self):
        b = BatteryModel(capacity_mah=2400)
        assert b.soc_percent() == 100
        assert not b.depleted

    def test_consume(self):
        b = BatteryModel(capacity_mah=2400)
        b.consume(100)
        assert b.soc_percent() < 100
        assert not b.depleted

    def test_depletion(self):
        b = BatteryModel(capacity_mah=1)  # Very small battery
        b.consume(b.capacity_mj + 1)
        assert b.depleted
        assert b.remaining_mj == 0

    def test_harvest_recharges(self):
        b = BatteryModel(capacity_mah=2400)
        b.consume(1000)
        soc_before = b.soc_percent()
        b.harvest(100, 10)  # 100mW * 10s = 1000mJ
        assert b.soc_percent() > soc_before

    def test_harvest_does_not_exceed_capacity(self):
        b = BatteryModel(capacity_mah=2400)
        b.harvest(1000000, 1000)  # Muito mais que capacidade
        assert b.remaining_mj <= b.capacity_mj

    def test_lifetime_estimate(self):
        b = BatteryModel(capacity_mah=2400)
        days = b.estimate_lifetime_days(10)  # 10 mJ/hora
        assert days > 0
        assert days < float('inf')


class TestEnergyHarvester:
    def test_solar_daytime(self):
        h = EnergyHarvester(model="solar", peak_power_mw=100)
        power = h.get_power(12)  # Meio-dia
        assert power > 90  # Proximo do pico

    def test_solar_nighttime(self):
        h = EnergyHarvester(model="solar", peak_power_mw=100)
        power = h.get_power(0)  # Meia-noite
        assert power == 0

    def test_constant(self):
        h = EnergyHarvester(model="constant", peak_power_mw=50)
        assert h.get_power(0) == 50
        assert h.get_power(12) == 50
        assert h.get_power(23) == 50
