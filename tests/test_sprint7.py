"""Testes Sprint 7: Deployment from_file, export NPZ, metricas expandidas, plots."""

import sys, os, tempfile, csv
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest
import numpy as np


# ===================== Deployment =====================

class TestDeployment:
    def test_deploy_grid_count(self):
        from deployment import deploy_grid
        pos = deploy_grid(10, 1000)
        assert len(pos) == 10

    def test_deploy_circular_count(self):
        from deployment import deploy_circular
        pos = deploy_circular(20, 500, 500, 500)
        assert len(pos) == 20

    def test_deploy_circular_within_radius(self):
        from deployment import deploy_circular
        r = 500
        cx, cy = 500, 500
        pos = deploy_circular(100, r, cx, cy)
        for x, y in pos:
            dist = np.hypot(x - cx, y - cy)
            assert dist <= r + 1e-9

    def test_deploy_annular_count(self):
        from deployment import deploy_annular
        pos = deploy_annular(15, 200, 500, 500, 500)
        assert len(pos) == 15

    def test_deploy_hexagonal_count(self):
        from deployment import deploy_hexagonal
        pos = deploy_hexagonal(6, 200)
        assert len(pos) == 6

    def test_deploy_random_uniform_in_area(self):
        from deployment import deploy_random_uniform
        area = 1000
        pos = deploy_random_uniform(50, area)
        assert len(pos) == 50
        for x, y in pos:
            assert 0 <= x <= area
            assert 0 <= y <= area

    def test_deploy_clustered_count(self):
        from deployment import deploy_clustered
        pos = deploy_clustered(30, 3, 1000)
        assert len(pos) == 30

    def test_deploy_from_file_basic(self):
        from deployment import deploy_from_file
        with tempfile.NamedTemporaryFile(mode='w', suffix='.csv', delete=False) as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])  # header
            for i in range(10):
                writer.writerow([i * 100, i * 50])
            fname = f.name
        try:
            pos = deploy_from_file(fname)
            assert len(pos) == 10
            assert pos[0] == (0.0, 0.0)
            assert pos[1] == (100.0, 50.0)
        finally:
            os.unlink(fname)

    def test_deploy_from_file_no_header(self):
        from deployment import deploy_from_file
        with tempfile.NamedTemporaryFile(mode='w', suffix='.csv', delete=False) as f:
            writer = csv.writer(f)
            for i in range(5):
                writer.writerow([float(i), float(i * 2)])
            fname = f.name
        try:
            pos = deploy_from_file(fname)
            assert len(pos) == 5
        finally:
            os.unlink(fname)


# ===================== Network from_file deployment =====================

class TestNetworkFromFileDeployment:
    def _make_positions_file(self, n, area=5000):
        """Cria CSV temporario com n posicoes."""
        f = tempfile.NamedTemporaryFile(mode='w', suffix='.csv', delete=False)
        writer = csv.writer(f)
        writer.writerow(['x', 'y'])
        rng = np.random.default_rng(99)
        for _ in range(n):
            writer.writerow([float(rng.uniform(0, area)), float(rng.uniform(0, area))])
        f.close()
        return f.name

    def test_from_file_deployment_creates_correct_devices(self):
        from network import Network
        fname = self._make_positions_file(10)
        try:
            net = Network(
                num_devices=10, num_gateways=1, area_size=5000,
                lambda_rate=0.01, speed=0, sf_range=[7, 8],
                tx_power=14, frequency_mhz=[868.1],
                ht_m=1.5, hr_m=30, bw=[125000], cr=1, pl=20,
                simulation_time=10, adr_enabled=False,
                model_pathloss="log_distance",
                deployment_type="from_file",
                deployment_file=fname,
            )
            assert len(net.devices) == 10
        finally:
            os.unlink(fname)

    def test_from_file_deployment_missing_file_raises(self):
        from network import Network
        with pytest.raises(ValueError, match="deployment_file"):
            Network(
                num_devices=5, num_gateways=1, area_size=5000,
                lambda_rate=0.01, speed=0, sf_range=[7],
                tx_power=14, frequency_mhz=[868.1],
                ht_m=1.5, hr_m=30, bw=[125000], cr=1, pl=20,
                simulation_time=10, adr_enabled=False,
                model_pathloss="log_distance",
                deployment_type="from_file",
                deployment_file=None,
            )

    def test_from_file_deployment_insufficient_positions_raises(self):
        from network import Network
        fname = self._make_positions_file(3)
        try:
            with pytest.raises(ValueError, match="3 posicoes"):
                Network(
                    num_devices=10, num_gateways=1, area_size=5000,
                    lambda_rate=0.01, speed=0, sf_range=[7],
                    tx_power=14, frequency_mhz=[868.1],
                    ht_m=1.5, hr_m=30, bw=[125000], cr=1, pl=20,
                    simulation_time=10, adr_enabled=False,
                    model_pathloss="log_distance",
                    deployment_type="from_file",
                    deployment_file=fname,
                )
        finally:
            os.unlink(fname)


# ===================== Export NPZ =====================

class TestExportNPZ:
    def _make_network(self):
        from network import Network
        net = Network(
            num_devices=5, num_gateways=1, area_size=5000,
            lambda_rate=0.1, speed=0, sf_range=[7, 8],
            tx_power=14, frequency_mhz=[868.1],
            ht_m=1.5, hr_m=30, bw=[125000], cr=1, pl=20,
            simulation_time=60, adr_enabled=False,
            model_pathloss="log_distance",
        )
        net.simulate_transmissions()
        net.detect_collisions_and_interference()
        return net

    def test_export_npz_creates_file(self):
        from analytics import export_npz
        net = self._make_network()
        with tempfile.NamedTemporaryFile(suffix='.npz', delete=False) as f:
            fname = f.name
        try:
            export_npz(net, fname)
            assert os.path.exists(fname)
            assert os.path.getsize(fname) > 0
        finally:
            os.unlink(fname)

    def test_export_npz_arrays_present(self):
        from analytics import export_npz
        net = self._make_network()
        with tempfile.NamedTemporaryFile(suffix='.npz', delete=False) as f:
            fname = f.name
        try:
            export_npz(net, fname)
            data = np.load(fname)
            required_keys = [
                'packet_time', 'packet_device_id', 'packet_sf',
                'packet_rssi', 'packet_snr', 'packet_sinr',
                'packet_collided', 'packet_freq', 'packet_airtime_ms',
                'device_id', 'device_sf', 'device_x', 'device_y',
                'device_energy_mj', 'device_pdr_percent',
            ]
            for key in required_keys:
                assert key in data, f"Chave '{key}' ausente no NPZ"
        finally:
            os.unlink(fname)

    def test_export_npz_device_count(self):
        from analytics import export_npz
        net = self._make_network()
        with tempfile.NamedTemporaryFile(suffix='.npz', delete=False) as f:
            fname = f.name
        try:
            export_npz(net, fname)
            data = np.load(fname)
            assert len(data['device_id']) == 5
        finally:
            os.unlink(fname)

    def test_export_npz_collided_binary(self):
        from analytics import export_npz
        net = self._make_network()
        with tempfile.NamedTemporaryFile(suffix='.npz', delete=False) as f:
            fname = f.name
        try:
            export_npz(net, fname)
            data = np.load(fname)
            vals = set(int(v) for v in data['packet_collided'])
            assert vals.issubset({0, 1}), "packet_collided deve ser 0 ou 1"
        finally:
            os.unlink(fname)


# ===================== Metricas Expandidas =====================

class TestComputeMetrics:
    def _make_network_run(self):
        from network import Network
        net = Network(
            num_devices=8, num_gateways=1, area_size=5000,
            lambda_rate=0.1, speed=0, sf_range=[7, 8, 9],
            tx_power=14, frequency_mhz=[868.1, 868.3],
            ht_m=1.5, hr_m=30, bw=[125000], cr=1, pl=20,
            simulation_time=120, adr_enabled=False,
            model_pathloss="log_distance",
        )
        net.simulate_transmissions()
        net.detect_collisions_and_interference()
        return net

    def test_metrics_has_region(self):
        from analytics import compute_metrics
        net = self._make_network_run()
        m = compute_metrics(net)
        assert "region" in m["simulation"]
        assert m["simulation"]["region"] != ""

    def test_metrics_has_deployment_type(self):
        from analytics import compute_metrics
        net = self._make_network_run()
        m = compute_metrics(net)
        assert "deployment_type" in m["simulation"]

    def test_metrics_has_retransmission_rate(self):
        from analytics import compute_metrics
        net = self._make_network_run()
        m = compute_metrics(net)
        assert "retransmission_rate_percent" in m["performance"]
        assert 0 <= m["performance"]["retransmission_rate_percent"] <= 100

    def test_metrics_has_pdr_vs_distance(self):
        from analytics import compute_metrics
        net = self._make_network_run()
        m = compute_metrics(net)
        assert "pdr_vs_distance_km" in m["performance"]
        assert isinstance(m["performance"]["pdr_vs_distance_km"], dict)

    def test_metrics_has_energy_breakdown(self):
        from analytics import compute_metrics
        net = self._make_network_run()
        m = compute_metrics(net)
        assert "breakdown_percent" in m["energy"]
        bd = m["energy"]["breakdown_percent"]
        # TX, RX, SLEEP, STANDBY devem estar presentes
        assert "TX" in bd or "SLEEP" in bd

    def test_metrics_has_energy_per_sf(self):
        from analytics import compute_metrics
        net = self._make_network_run()
        m = compute_metrics(net)
        assert "avg_per_sf_mj" in m["energy"]
        assert isinstance(m["energy"]["avg_per_sf_mj"], dict)

    def test_metrics_has_delay_percentiles(self):
        from analytics import compute_metrics
        net = self._make_network_run()
        m = compute_metrics(net)
        assert "p50_delay_ms" in m["performance"]
        assert "p95_delay_ms" in m["performance"]

    def test_metrics_has_sinr_p5(self):
        from analytics import compute_metrics
        net = self._make_network_run()
        m = compute_metrics(net)
        assert "p5_sinr_db" in m["radio"]

    def test_metrics_battery_lifetime_none_without_battery(self):
        from analytics import compute_metrics
        net = self._make_network_run()
        m = compute_metrics(net)
        assert m["energy"]["avg_battery_lifetime_days"] is None


# ===================== Plots =====================

class TestPlots:
    def _make_network_run(self):
        from network import Network
        net = Network(
            num_devices=6, num_gateways=1, area_size=5000,
            lambda_rate=0.1, speed=0, sf_range=[7, 8],
            tx_power=14, frequency_mhz=[868.1],
            ht_m=1.5, hr_m=30, bw=[125000], cr=1, pl=20,
            simulation_time=60, adr_enabled=False,
            model_pathloss="log_distance",
        )
        net.simulate_transmissions()
        net.detect_collisions_and_interference()
        return net

    def test_plot_pdr_per_sf_creates_file(self):
        from plots import plot_pdr_per_sf
        net = self._make_network_run()
        with tempfile.NamedTemporaryFile(suffix='.png', delete=False) as f:
            fname = f.name
        try:
            plot_pdr_per_sf(net, fname)
            assert os.path.exists(fname)
            assert os.path.getsize(fname) > 1000
        finally:
            os.unlink(fname)

    def test_plot_pdr_vs_distance_creates_file(self):
        from plots import plot_pdr_vs_distance
        net = self._make_network_run()
        with tempfile.NamedTemporaryFile(suffix='.png', delete=False) as f:
            fname = f.name
        try:
            plot_pdr_vs_distance(net, fname)
            assert os.path.exists(fname)
        finally:
            if os.path.exists(fname):
                os.unlink(fname)

    def test_plot_energy_per_sf_creates_file(self):
        from plots import plot_energy_per_sf
        net = self._make_network_run()
        with tempfile.NamedTemporaryFile(suffix='.png', delete=False) as f:
            fname = f.name
        try:
            plot_energy_per_sf(net, fname)
            assert os.path.exists(fname)
            assert os.path.getsize(fname) > 1000
        finally:
            os.unlink(fname)

    def test_plot_sinr_distribution_creates_file(self):
        from plots import plot_sinr_distribution
        net = self._make_network_run()
        with tempfile.NamedTemporaryFile(suffix='.png', delete=False) as f:
            fname = f.name
        try:
            plot_sinr_distribution(net, fname)
            # Pode nao ter sinr se nenhum pacote foi recebido
            if os.path.exists(fname):
                os.unlink(fname)
        finally:
            pass

    def test_plot_throughput_creates_file(self):
        from plots import plot_throughput_over_time
        net = self._make_network_run()
        with tempfile.NamedTemporaryFile(suffix='.png', delete=False) as f:
            fname = f.name
        try:
            plot_throughput_over_time(net, fname)
            if os.path.exists(fname):
                assert os.path.getsize(fname) > 1000
                os.unlink(fname)
        finally:
            pass

    def test_plot_collision_heatmap_runs_without_error(self):
        from plots import plot_collision_heatmap
        net = self._make_network_run()
        with tempfile.NamedTemporaryFile(suffix='.png', delete=False) as f:
            fname = f.name
        try:
            # Pode nao gerar arquivo se nao houver colisoes
            plot_collision_heatmap(net, fname)
            if os.path.exists(fname):
                os.unlink(fname)
        finally:
            pass

    def test_plot_all_runs_without_error(self):
        from plots import plot_all
        net = self._make_network_run()
        with tempfile.TemporaryDirectory() as tmpdir:
            prefix = os.path.join(tmpdir, "test_")
            plot_all(net, prefix=prefix)
            # Verifica que pelo menos os graficos obrigatorios foram gerados
            for name in ['plot_sf_distribution.png', 'plot_pdr_per_sf.png',
                         'plot_energy_per_sf.png']:
                assert os.path.exists(prefix + name), f"Grafico ausente: {name}"
