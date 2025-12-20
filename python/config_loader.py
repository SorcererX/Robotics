"""
Utility helpers for loading Sepia-style YAML configuration without depending on
the C++ SepiaComm library.
"""

import os
from pathlib import Path
from typing import Dict

try:
    import yaml
except ModuleNotFoundError as exc:  # pragma: no cover
    raise SystemExit(
        "Missing dependency PyYAML. Install it with `pip install pyyaml` or "
        "your distro's python3-yaml package."
    ) from exc

DEFAULT_CONFIG = "sepia.yaml"


def _read_yaml( path: Path ) -> Dict:
    try:
        with path.open( "r", encoding="utf-8" ) as handle:
            return yaml.safe_load( handle ) or {}
    except FileNotFoundError:
        return {}


def load_configuration() -> Dict[ str, object ]:
    """
    Mimics sepia::comm2::Global::loadConfiguration by reading SEPIA_CONFIG or
    falling back to `sepia.yaml`. Missing fields default to the hardcoded values
    used by the C++ implementation.
    """
    config_path = os.environ.get( "SEPIA_CONFIG", DEFAULT_CONFIG )
    if not config_path:
        config_path = DEFAULT_CONFIG

    data = _read_yaml( Path( config_path ) )

    return {
        "publisher_url": data.get( "publisher_url", "tcp://127.0.0.1:31339" ),
        "subscriber_url": data.get( "subscriber_url", "tcp://127.0.0.1:31340" ),
        "context_threads": int( data.get( "context_threads", 4 ) ),
    }
