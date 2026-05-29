"""
Unit tests for RTK/PPK module.
"""

import json
import os
import pytest
from dji_photogrammetry.rtk_ppk import GCPManager, PPKProcessor, GroundControlPoint


# ---------------------------------------------------------------------------
# GCPManager
# ---------------------------------------------------------------------------

class TestGCPManager:
    @pytest.fixture
    def sample_gcps(self):
        manager = GCPManager()
        for i in range(3):
            gcp = GroundControlPoint(
                name=f"GCP{i:02d}",
                latitude=47.37 + i * 0.001,
                longitude=8.54 + i * 0.001,
                altitude_m=430.0 + i,
                accuracy_h_m=0.01,
                accuracy_v_m=0.02,
                pixel_observations=[
                    {"image": f"DJI_{i:03d}.jpg", "x": 512.0, "y": 384.0}
                ]
            )
            manager.add_gcp(gcp)
        return manager

    def test_validate_enough_gcps(self, sample_gcps):
        result = sample_gcps.validate()
        assert result['valid'] is True
        assert result['count'] == 3
        assert result['total_pixel_observations'] == 3

    def test_validate_too_few_gcps(self):
        manager = GCPManager()
        manager.add_gcp(GroundControlPoint("A", 47.0, 8.5, 430.0))
        result = manager.validate()
        assert result['valid'] is False

    def test_export_load_json_roundtrip(self, sample_gcps, tmp_path):
        path = str(tmp_path / "gcps.json")
        sample_gcps.export_json(path)
        assert os.path.exists(path)

        new_manager = GCPManager()
        new_manager.load_from_json(path)
        assert len(new_manager.gcps) == 3
        assert new_manager.gcps[0].name == "GCP00"

    def test_export_odm_gcp_list(self, sample_gcps, tmp_path):
        path = str(tmp_path / "gcp_list.txt")
        sample_gcps.export_odm_gcp_list(path)
        assert os.path.exists(path)
        with open(path) as f:
            lines = f.readlines()
        # First line is projection, then one line per pixel observation
        assert len(lines) >= 4   # 1 header + 3 observations

    def test_odm_gcp_list_format(self, sample_gcps, tmp_path):
        path = str(tmp_path / "gcp_list.txt")
        sample_gcps.export_odm_gcp_list(path)
        with open(path) as f:
            lines = f.readlines()
        # Each data line should have 7 space-separated fields
        for line in lines[1:]:
            parts = line.strip().split()
            assert len(parts) == 7, f"Expected 7 fields, got: {parts}"

    def test_bounding_box_in_validation(self, sample_gcps):
        result = sample_gcps.validate()
        bb = result['bounding_box']
        assert bb['min_lat'] < bb['max_lat']
        assert bb['min_lon'] < bb['max_lon']


# ---------------------------------------------------------------------------
# PPKProcessor
# ---------------------------------------------------------------------------

class TestPPKProcessor:
    BASE_LAT = 47.3769
    BASE_LON = 8.5417
    BASE_ALT = 430.0

    @pytest.fixture
    def processor(self):
        p = PPKProcessor()
        p.set_base_station(self.BASE_LAT, self.BASE_LON, self.BASE_ALT)
        return p

    @pytest.fixture
    def sample_drone_log_json(self, tmp_path):
        data = {
            "trigger_events": [
                {
                    "image_filename": "DJI_001.JPG",
                    "drone_state": {
                        "latitude": 47.3770,
                        "longitude": 8.5420,
                        "altitude_m": 530.0,
                        "yaw_deg": 45.0,
                        "pitch_deg": 0.0,
                        "roll_deg": 0.0,
                        "heading_deg": 45.0,
                        "ground_speed_ms": 5.0,
                        "timestamp": "2024-06-01T10:00:00"
                    }
                },
                {
                    "image_filename": "DJI_002.JPG",
                    "drone_state": {
                        "latitude": 47.3775,
                        "longitude": 8.5425,
                        "altitude_m": 530.0,
                        "yaw_deg": 45.0,
                        "pitch_deg": 0.0,
                        "roll_deg": 0.0,
                        "heading_deg": 45.0,
                        "ground_speed_ms": 5.0,
                        "timestamp": "2024-06-01T10:00:05"
                    }
                }
            ]
        }
        path = str(tmp_path / "trigger_log.json")
        with open(path, 'w') as f:
            json.dump(data, f)
        return path

    @pytest.fixture
    def sample_drone_log_csv(self, tmp_path):
        import pandas as pd
        path = str(tmp_path / "trigger_log.csv")
        rows = [
            {"image_filename": "DJI_003.JPG", "latitude": 47.3771,
             "longitude": 8.5421, "altitude_m": 530.0,
             "timestamp": "2024-06-01T10:00:10"},
        ]
        pd.DataFrame(rows).to_csv(path, index=False)
        return path

    def test_set_base_station(self, processor):
        assert processor.base_latitude  == self.BASE_LAT
        assert processor.base_longitude == self.BASE_LON
        assert processor.base_altitude_m == self.BASE_ALT

    def test_load_drone_log_json(self, processor, sample_drone_log_json):
        rows = processor.load_drone_log(sample_drone_log_json)
        assert len(rows) == 2
        assert rows[0]['image_filename'] == "DJI_001.JPG"
        assert rows[0]['latitude'] == pytest.approx(47.3770, abs=1e-6)

    def test_load_drone_log_csv(self, processor, sample_drone_log_csv):
        rows = processor.load_drone_log(sample_drone_log_csv)
        assert len(rows) == 1
        assert rows[0]['image_filename'] == "DJI_003.JPG"

    def test_apply_corrections_returns_positions(self, processor, sample_drone_log_json):
        rows = processor.load_drone_log(sample_drone_log_json)
        corrected = processor.apply_corrections(rows)
        assert len(corrected) == 2
        for cp in corrected:
            assert cp.latitude  is not None
            assert cp.longitude is not None
            assert cp.fix_type in {"RTK_FIXED", "RTK_FLOAT", "PPK_FIXED", "PPK_FLOAT", "GNSS"}

    def test_corrected_positions_close_to_input(self, processor, sample_drone_log_json):
        rows = processor.load_drone_log(sample_drone_log_json)
        corrected = processor.apply_corrections(rows)
        for cp, row in zip(corrected, rows):
            assert abs(cp.latitude  - row['latitude'])  < 0.01
            assert abs(cp.longitude - row['longitude']) < 0.01

    def test_export_corrected_csv(self, processor, sample_drone_log_json, tmp_path):
        rows = processor.load_drone_log(sample_drone_log_json)
        processor.apply_corrections(rows)
        path = str(tmp_path / "corrected.csv")
        processor.export_corrected_csv(path)
        assert os.path.exists(path)
        import pandas as pd
        df = pd.read_csv(path)
        assert len(df) == 2
        assert 'latitude' in df.columns
        assert 'fix_type' in df.columns

    def test_survey_report(self, processor, sample_drone_log_json, tmp_path):
        rows = processor.load_drone_log(sample_drone_log_json)
        processor.apply_corrections(rows)
        path = str(tmp_path / "report.json")
        processor.generate_survey_report(path)
        assert os.path.exists(path)
        with open(path) as f:
            report = json.load(f)
        assert report['total_images'] == 2
        assert 'horizontal_accuracy' in report
        assert 'fix_type_counts' in report

    def test_no_positions_report_graceful(self, processor, tmp_path, capsys):
        path = str(tmp_path / "report.json")
        processor.generate_survey_report(path)
        out = capsys.readouterr().out
        assert "No corrected" in out
        assert not os.path.exists(path)
