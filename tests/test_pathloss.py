"""Testes de modelos de Path Loss."""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest
import numpy as np


class TestPathLoss:
    @pytest.fixture(autouse=True)
    def setup(self, single_device_network):
        self.network = single_device_network

    def test_fspl_1km(self):
        """FSPL a 1km/868MHz deve ser ~91.5 dB."""
        pl = self.network.pathloss(1000, 868.1, "fspl")
        assert 89 < pl < 94, f"FSPL 1km = {pl:.1f} dB"

    def test_fspl_increases_with_distance(self):
        """Path loss deve aumentar com distancia."""
        pl_1km = self.network.pathloss(1000, 868.1, "fspl")
        pl_5km = self.network.pathloss(5000, 868.1, "fspl")
        pl_10km = self.network.pathloss(10000, 868.1, "fspl")
        assert pl_1km < pl_5km < pl_10km

    def test_fspl_increases_with_frequency(self):
        """Path loss deve aumentar com frequencia."""
        pl_868 = self.network.pathloss(1000, 868.1, "fspl")
        pl_915 = self.network.pathloss(1000, 915.0, "fspl")
        assert pl_868 < pl_915

    def test_okumura_hata_reasonable(self):
        """Okumura-Hata a 1km deve estar em faixa razoavel."""
        pl = self.network.pathloss(1000, 868.1, "okumura_hata")
        assert 50 < pl < 200, f"Okumura-Hata 1km = {pl:.1f} dB"

    def test_cost_hata_reasonable(self):
        """COST-Hata a 1km deve estar em faixa razoavel."""
        pl = self.network.pathloss(1000, 868.1, "cost_hata")
        assert 50 < pl < 200, f"COST-Hata 1km = {pl:.1f} dB"

    def test_log_normal_shadowing(self):
        """Log-Normal Shadowing deve ter variabilidade mas media razoavel."""
        np.random.seed(42)
        losses = [self.network.pathloss(1000, 868.1, "log_normal_shadowing") for _ in range(100)]
        mean_pl = np.mean(losses)
        std_pl = np.std(losses)
        assert 80 < mean_pl < 160, f"Mean PL = {mean_pl:.1f}"
        assert std_pl > 1, f"Std PL = {std_pl:.1f}, deve ter variabilidade"

    def test_zero_distance_returns_inf(self):
        """Distancia zero deve retornar infinito."""
        pl = self.network.pathloss(0, 868.1, "fspl")
        assert pl == float('inf')

    def test_all_models_return_positive(self):
        """Todos os modelos devem retornar path loss positivo."""
        models = ["okumura_hata", "log_distance", "fspl", "cost_hata", "log_normal_shadowing"]
        for model in models:
            pl = self.network.pathloss(1000, 868.1, model)
            assert pl > 0, f"Model {model} retornou {pl}"

    def test_unknown_model_raises(self):
        """Modelo desconhecido deve gerar erro."""
        with pytest.raises(ValueError):
            self.network.pathloss(1000, 868.1, "modelo_inexistente")
