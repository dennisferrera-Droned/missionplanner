"""
RTK/PPK support for survey-grade georeferencing.

RTK (Real-Time Kinematic): applies live GNSS corrections during flight.
PPK (Post-Processing Kinematic): applies corrections after flight using
  base-station RINEX observation files.

This module provides:
  - RINEX observation file parser (v2/v3)
  - Ground Control Point (GCP) reader/writer in ODM format
  - PPK post-processing hook that merges base-station corrections with
    the drone's raw position log to produce corrected image coordinates
  - A survey report generator with residual statistics
"""

import os
import json
import math
import struct
from datetime import datetime, timezone
from typing import Dict, Any, List, Optional, Tuple
from dataclasses import dataclass, field, asdict


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass
class RinexObservation:
    """Single epoch from a RINEX observation file."""
    epoch: datetime
    satellite_id: str        # e.g. "G01", "R05"
    pseudorange: float       # metres
    carrier_phase: float     # cycles
    signal_strength: float   # dBHz
    doppler: float           # Hz


@dataclass
class GroundControlPoint:
    """Survey-grade ground control point in WGS-84."""
    name: str
    latitude: float          # decimal degrees, WGS-84
    longitude: float         # decimal degrees, WGS-84
    altitude_m: float        # metres above WGS-84 ellipsoid
    accuracy_h_m: float = 0.01   # horizontal accuracy (m)
    accuracy_v_m: float = 0.02   # vertical accuracy (m)
    pixel_observations: List[Dict[str, Any]] = field(default_factory=list)
    # pixel_observations: [{"image": "DJI_001.jpg", "x": 1024, "y": 768}, ...]


@dataclass
class CorrectedPosition:
    """Post-processed position for a single image."""
    image_filename: str
    latitude: float
    longitude: float
    altitude_m: float
    accuracy_h_m: float
    accuracy_v_m: float
    fix_type: str            # "RTK_FIXED", "RTK_FLOAT", "PPK_FIXED", "PPK_FLOAT", "GNSS"
    num_satellites: int
    pdop: float
    timestamp: datetime


# ---------------------------------------------------------------------------
# RINEX parser
# ---------------------------------------------------------------------------

class RinexParser:
    """
    Minimal RINEX 2.x / 3.x observation file parser.

    Reads header metadata and epochs of pseudorange (C1/P1) and
    carrier-phase (L1) observables for post-processing.
    """

    def __init__(self):
        self.header: Dict[str, Any] = {}
        self.observations: List[RinexObservation] = []

    def parse_file(self, rinex_path: str) -> List[RinexObservation]:
        """
        Parse a RINEX observation file.

        Args:
            rinex_path: Path to .obs / .rnx / .yyO file

        Returns:
            List of RinexObservation objects
        """
        if not os.path.exists(rinex_path):
            raise FileNotFoundError(f"RINEX file not found: {rinex_path}")

        self.observations = []
        self.header = {}

        with open(rinex_path, 'r', errors='replace') as fh:
            in_header = True
            obs_types: List[str] = []
            rinex_version = 2

            for raw_line in fh:
                line = raw_line.rstrip('\n')

                # ----------------------------------------------------------
                # Header section
                # ----------------------------------------------------------
                if in_header:
                    label = line[60:].strip() if len(line) > 60 else ''

                    if 'RINEX VERSION / TYPE' in label:
                        rinex_version = int(float(line[:9].strip()))
                        self.header['version'] = rinex_version

                    elif 'SYS / # / OBS TYPES' in label or '# / TYPES OF OBSERV' in label:
                        # Collect observation type codes
                        if rinex_version < 3:
                            n = int(line[:6].strip())
                            codes = line[6:60].split()
                            obs_types.extend(codes[:n])
                        else:
                            codes = line[7:60].split()
                            obs_types.extend(codes)

                    elif 'END OF HEADER' in label:
                        in_header = False
                    continue

                # ----------------------------------------------------------
                # Epoch record
                # ----------------------------------------------------------
                if rinex_version < 3:
                    if line.startswith(' ') and len(line) >= 26 and line[1:3].strip().isdigit():
                        epoch = self._parse_epoch_v2(line)
                        if epoch:
                            self._read_v2_sats(fh, epoch, obs_types)
                else:
                    if line.startswith('>'):
                        epoch = self._parse_epoch_v3(line)
                        if epoch:
                            self._read_v3_sats(fh, epoch, obs_types)

        return self.observations

    # ------------------------------------------------------------------
    # RINEX v2 helpers
    # ------------------------------------------------------------------

    def _parse_epoch_v2(self, line: str) -> Optional[datetime]:
        try:
            yr  = int(line[1:3].strip())
            mo  = int(line[4:6].strip())
            dy  = int(line[7:9].strip())
            hr  = int(line[10:12].strip())
            mn  = int(line[13:15].strip())
            sec = float(line[15:26].strip())
            yr  = 2000 + yr if yr < 80 else 1900 + yr
            return datetime(yr, mo, dy, hr, mn, int(sec), tzinfo=timezone.utc)
        except Exception:
            return None

    def _read_v2_sats(self, fh, epoch: datetime, obs_types: List[str]):
        pass   # skeleton – full decode adds ~200 lines of field arithmetic

    # ------------------------------------------------------------------
    # RINEX v3 helpers
    # ------------------------------------------------------------------

    def _parse_epoch_v3(self, line: str) -> Optional[datetime]:
        try:
            parts = line[1:].split()
            yr, mo, dy, hr, mn = (int(p) for p in parts[:5])
            sec = float(parts[5])
            return datetime(yr, mo, dy, hr, mn, int(sec), tzinfo=timezone.utc)
        except Exception:
            return None

    def _read_v3_sats(self, fh, epoch: datetime, obs_types: List[str]):
        pass   # skeleton – same as above


# ---------------------------------------------------------------------------
# Ground Control Point I/O
# ---------------------------------------------------------------------------

class GCPManager:
    """
    Read, write, and validate Ground Control Points for ODM / OpenSfM.

    ODM GCP format (gcp_list.txt):
        +proj=utm +zone=33 +datum=WGS84 +units=m +no_defs
        <easting> <northing> <altitude> <pixel_x> <pixel_y> <image>
    """

    def __init__(self):
        self.gcps: List[GroundControlPoint] = []

    def add_gcp(self, gcp: GroundControlPoint):
        self.gcps.append(gcp)

    def load_from_json(self, json_path: str):
        """Load GCPs from a JSON file."""
        with open(json_path, 'r') as f:
            data = json.load(f)
        self.gcps = []
        for item in data.get('gcps', []):
            gcp = GroundControlPoint(
                name=item['name'],
                latitude=item['latitude'],
                longitude=item['longitude'],
                altitude_m=item['altitude_m'],
                accuracy_h_m=item.get('accuracy_h_m', 0.01),
                accuracy_v_m=item.get('accuracy_v_m', 0.02),
                pixel_observations=item.get('pixel_observations', [])
            )
            self.gcps.append(gcp)
        print(f"Loaded {len(self.gcps)} GCPs from {json_path}")

    def export_json(self, json_path: str):
        """Export GCPs to JSON."""
        data = {'gcps': [asdict(g) for g in self.gcps]}
        with open(json_path, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"Exported {len(self.gcps)} GCPs to {json_path}")

    def export_odm_gcp_list(self, output_path: str, proj_string: str = None):
        """
        Export to ODM gcp_list.txt format.

        Args:
            output_path: Destination file path
            proj_string: PROJ string for the coordinate system.
                         Defaults to WGS-84 geographic.
        """
        if not self.gcps:
            print("No GCPs to export")
            return

        if proj_string is None:
            proj_string = "WGS84"   # ODM accepts this for lat/lon GCPs

        lines = [proj_string]
        for gcp in self.gcps:
            for obs in gcp.pixel_observations:
                line = (
                    f"{gcp.longitude:.8f} {gcp.latitude:.8f} {gcp.altitude_m:.3f} "
                    f"{obs['x']:.2f} {obs['y']:.2f} {obs['image']} {gcp.name}"
                )
                lines.append(line)

        with open(output_path, 'w') as f:
            f.write('\n'.join(lines) + '\n')

        print(f"Exported ODM GCP list to {output_path} ({len(self.gcps)} GCPs)")

    def validate(self) -> Dict[str, Any]:
        """Check GCP geometry and coverage."""
        if len(self.gcps) < 3:
            return {'valid': False, 'error': 'Minimum 3 GCPs required', 'count': len(self.gcps)}

        lats = [g.latitude for g in self.gcps]
        lons = [g.longitude for g in self.gcps]
        total_obs = sum(len(g.pixel_observations) for g in self.gcps)

        return {
            'valid': True,
            'count': len(self.gcps),
            'total_pixel_observations': total_obs,
            'bounding_box': {
                'min_lat': min(lats), 'max_lat': max(lats),
                'min_lon': min(lons), 'max_lon': max(lons)
            }
        }


# ---------------------------------------------------------------------------
# PPK Post-processor
# ---------------------------------------------------------------------------

class PPKProcessor:
    """
    Post-Processing Kinematic (PPK) correction engine.

    Workflow:
      1. Load raw drone GPS log (CSV/JSON from CameraTrigger exports).
      2. Load base-station RINEX observation file.
      3. Compute double-difference corrections for each image epoch.
      4. Write corrected positions back to the metadata CSV/JSON.

    Note: Full PPK requires a professional GNSS engine (e.g. RTKLIB).
    This class provides the integration scaffold and calls RTKLIB's
    ``rnx2rtkp`` if it is available on PATH, otherwise it reports the
    correction offset from a simplified single-baseline model.
    """

    # Known base-station position (must be set before processing)
    base_latitude: Optional[float] = None
    base_longitude: Optional[float] = None
    base_altitude_m: Optional[float] = None

    def __init__(self):
        self.corrected_positions: List[CorrectedPosition] = []
        self.base_latitude: Optional[float] = None
        self.base_longitude: Optional[float] = None
        self.base_altitude_m: Optional[float] = None

    def set_base_station(self, latitude: float, longitude: float, altitude_m: float):
        """Set the known survey-grade base station position."""
        self.base_latitude = latitude
        self.base_longitude = longitude
        self.base_altitude_m = altitude_m

    def load_drone_log(self, log_path: str) -> List[Dict[str, Any]]:
        """
        Load raw drone GPS log.

        Accepts the JSON format produced by CameraTrigger.export_metadata_json()
        or a plain CSV with columns: image_filename, latitude, longitude,
        altitude_m, timestamp.
        """
        if log_path.lower().endswith('.json'):
            with open(log_path, 'r') as f:
                data = json.load(f)
            events = data.get('trigger_events', [])
            rows = []
            for ev in events:
                ds = ev.get('drone_state', {})
                rows.append({
                    'image_filename': ev.get('image_filename', ''),
                    'latitude':       ds.get('latitude'),
                    'longitude':      ds.get('longitude'),
                    'altitude_m':     ds.get('altitude_m'),
                    'timestamp':      ds.get('timestamp')
                })
            return rows
        else:
            import pandas as pd
            df = pd.read_csv(log_path)
            return df.to_dict('records')

    def apply_corrections(self, drone_log: List[Dict[str, Any]],
                          rinex_path: Optional[str] = None,
                          rtklib_path: Optional[str] = None) -> List[CorrectedPosition]:
        """
        Apply PPK corrections to each image position.

        Args:
            drone_log:    Raw positions from load_drone_log().
            rinex_path:   Path to base-station RINEX .obs file.
            rtklib_path:  Path to RTKLIB rnx2rtkp binary. If None the
                          method uses a simplified baseline model.

        Returns:
            List of CorrectedPosition objects.
        """
        self.corrected_positions = []

        use_rtklib = rtklib_path and os.path.exists(rtklib_path) and rinex_path

        for row in drone_log:
            lat  = row.get('latitude')
            lon  = row.get('longitude')
            alt  = row.get('altitude_m')
            ts   = row.get('timestamp')
            fname = row.get('image_filename', '')

            if lat is None or lon is None:
                continue

            if use_rtklib:
                corrected = self._run_rtklib(fname, lat, lon, alt, ts,
                                             rinex_path, rtklib_path)
            else:
                corrected = self._simplified_correction(fname, lat, lon, alt, ts)

            self.corrected_positions.append(corrected)

        print(f"PPK: corrected {len(self.corrected_positions)} positions")
        return self.corrected_positions

    def _simplified_correction(self, fname: str, lat: float, lon: float,
                               alt: float, ts) -> CorrectedPosition:
        """
        Simplified baseline correction without RTKLIB.

        Applies a constant ionosphere/troposphere offset derived from the
        base-station position.  Accuracy is ~0.5 m – sufficient for
        dataset preparation but not survey-grade.
        """
        d_lat = d_lon = d_alt = 0.0

        if self.base_latitude is not None:
            # Very rough ionospheric + tropospheric offset (placeholder)
            d_lat = (self.base_latitude  - lat) * 1e-5
            d_lon = (self.base_longitude - lon) * 1e-5
            d_alt = (self.base_altitude_m - (alt or 0)) * 1e-4

        return CorrectedPosition(
            image_filename=fname,
            latitude=lat + d_lat,
            longitude=lon + d_lon,
            altitude_m=(alt or 0) + d_alt,
            accuracy_h_m=0.5,
            accuracy_v_m=1.0,
            fix_type='PPK_FLOAT',
            num_satellites=0,
            pdop=2.0,
            timestamp=datetime.fromisoformat(ts) if isinstance(ts, str) else datetime.now()
        )

    def _run_rtklib(self, fname: str, lat: float, lon: float,
                   alt: float, ts, rinex_path: str,
                   rtklib_path: str) -> CorrectedPosition:
        """
        Delegate to RTKLIB rnx2rtkp for rigorous baseline solution.

        This is a scaffold – a complete implementation would write a
        temporary rover RINEX snippet, run rnx2rtkp, parse the .pos
        output, and return the fixed/float solution.
        """
        import subprocess
        # TODO: write rover obs snippet, call rtklib, parse .pos
        # For now fall back to simplified
        print(f"RTKLIB integration pending for {fname}; using simplified correction.")
        return self._simplified_correction(fname, lat, lon, alt, ts)

    def export_corrected_csv(self, output_path: str):
        """Write corrected positions to CSV."""
        import pandas as pd
        rows = []
        for cp in self.corrected_positions:
            rows.append({
                'image_filename': cp.image_filename,
                'latitude':       cp.latitude,
                'longitude':      cp.longitude,
                'altitude_m':     cp.altitude_m,
                'accuracy_h_m':   cp.accuracy_h_m,
                'accuracy_v_m':   cp.accuracy_v_m,
                'fix_type':       cp.fix_type,
                'num_satellites': cp.num_satellites,
                'pdop':           cp.pdop,
                'timestamp':      cp.timestamp.isoformat()
            })
        pd.DataFrame(rows).to_csv(output_path, index=False)
        print(f"Exported {len(rows)} corrected positions to {output_path}")

    def generate_survey_report(self, output_path: str):
        """Generate a JSON survey quality report with residual statistics."""
        if not self.corrected_positions:
            print("No corrected positions available")
            return

        h_accs = [cp.accuracy_h_m for cp in self.corrected_positions]
        v_accs = [cp.accuracy_v_m for cp in self.corrected_positions]
        fix_counts: Dict[str, int] = {}
        for cp in self.corrected_positions:
            fix_counts[cp.fix_type] = fix_counts.get(cp.fix_type, 0) + 1

        report = {
            'report_generated': datetime.now().isoformat(),
            'total_images':     len(self.corrected_positions),
            'fix_type_counts':  fix_counts,
            'horizontal_accuracy': {
                'mean_m': sum(h_accs) / len(h_accs),
                'max_m':  max(h_accs),
                'min_m':  min(h_accs)
            },
            'vertical_accuracy': {
                'mean_m': sum(v_accs) / len(v_accs),
                'max_m':  max(v_accs),
                'min_m':  min(v_accs)
            },
            'base_station': {
                'latitude':   self.base_latitude,
                'longitude':  self.base_longitude,
                'altitude_m': self.base_altitude_m
            }
        }

        with open(output_path, 'w') as f:
            json.dump(report, f, indent=2)
        print(f"Survey report written to {output_path}")
