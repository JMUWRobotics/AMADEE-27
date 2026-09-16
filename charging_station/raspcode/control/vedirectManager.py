# -*- coding: utf-8 -*-
"""VedirectController für SmartSolar MPPT 150/35."""
import logging
import yaml
import sys
from ve_utils.utype import UType as Ut
from vedirect_m8.ve_controller import VedirectController
from vedirect_m8 import configure_logging
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent.parent
CONFIG_PATH = BASE_DIR / "config" / "system.yaml"

logging.basicConfig()
logger = logging.getLogger("vedirect")

def print_data_callback(packet):
    """Callback zur Ausgabe der empfangenen VE.Direct Pakete."""
    logger.info("%s\n", packet)


if __name__ == '__main__':
    with open(CONFIG_PATH, "r") as f:
        config = yaml.safe_load(f)
    configure_logging(config.get("debug", False))

    conf = {
        "serial_port": config.get("serial_port"),
        "timeout": config.get("timeout")
    }

    ve = VedirectController(
        serial_conf=conf,
        serial_test=config.get("PID_test"),
    )

    breakpoint()

    ve.read_data_callback(print_data_callback)