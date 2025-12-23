import os
from pathlib import Path
from typing import Dict
import yaml

DEFAULT_CONFIG = "sepia.yaml"


def _read_yaml( path: Path ) -> Dict:
    try:
        with path.open( "r", encoding="utf-8" ) as handle:
            return yaml.safe_load( handle ) or {}
    except FileNotFoundError:
        return {}


def load_configuration() -> Dict[ str, object ]:
    config_path = os.environ.get( "SEPIA_CONFIG", DEFAULT_CONFIG )
    if not config_path:
        config_path = DEFAULT_CONFIG

    data = _read_yaml( Path( config_path ) )

    return {
        "publisher_url": data.get( "publisher_url", "tcp://127.0.0.1:31339" ),
        "subscriber_url": data.get( "subscriber_url", "tcp://127.0.0.1:31340" ),
        "context_threads": int( data.get( "context_threads", 4 ) ),
    }
