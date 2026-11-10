#!/usr/bin/env python3
"""Externen Hardware-Watchdog füttern - Pins und Ablauf wie in GPIOManager.

- RPi.GPIO, BCM-Nummerierung
- WATCHDOG_ENABLE_PIN: einmal kurz HIGH -> Watchdog aktivieren
- WATCHDOG_PING_PIN:   regelmäßiger Impuls -> Watchdog füttern
Bleibt der Impuls aus, löst der Watchdog nach seinem Timeout den Reset aus.
"""
import logging
import signal
import threading
import time

import RPi.GPIO as GPIO

logger = logging.getLogger(__name__)

WATCHDOG_ENABLE_PIN = 18   # Default wie in GPIOManager ("watchdog_enable_pin")
WATCHDOG_PING_PIN = 27     # Default wie in GPIOManager ("watchdog_ping")
PING_INTERVAL = 1.0        # s zwischen den Impulsen, deutlich kleiner als der Watchdog-Timeout!
PULSE_WIDTH = 0.01         # s, Mindestpulsbreite im Datenblatt des Watchdogs prüfen
ENABLE_PULSE_WIDTH = 0.1   # s, wie in GPIOManager.enable_watchdog()


def setup_gpio() -> None:
    # Jede Library, die die Pins anfasst, muss dieselbe Nummerierung benutzen.
    if GPIO.getmode() is None:
        GPIO.setmode(GPIO.BCM)
    elif GPIO.getmode() != GPIO.BCM:
        raise RuntimeError("GPIO already configured with a numbering other than BCM")
    GPIO.setup(WATCHDOG_ENABLE_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(WATCHDOG_PING_PIN, GPIO.OUT, initial=GPIO.LOW)


def enable_watchdog() -> None:
    """Watchdog aktivieren: Enable-Pin kurz auf HIGH."""
    GPIO.output(WATCHDOG_ENABLE_PIN, GPIO.HIGH)
    time.sleep(ENABLE_PULSE_WIDTH)
    GPIO.output(WATCHDOG_ENABLE_PIN, GPIO.LOW)


def ping_watchdog() -> None:
    """Einen Impuls (steigende + fallende Flanke) an den Watchdog senden."""
    GPIO.output(WATCHDOG_PING_PIN, GPIO.HIGH)
    time.sleep(PULSE_WIDTH)
    GPIO.output(WATCHDOG_PING_PIN, GPIO.LOW)


def system_ok() -> bool:
    """Hier eigene Gesundheitsprüfung einbauen (Prozess läuft, Sensor antwortet, ...).

    Nur wenn True zurückkommt, wird der Watchdog gefüttert.
    """
    return True


def run(stop: threading.Event) -> None:
    setup_gpio()
    enable_watchdog()
    while not stop.is_set():
        if system_ok():
            ping_watchdog()
        else:
            logger.warning("System nicht ok - Watchdog wird nicht gefüttert")
        stop.wait(PING_INTERVAL)
    # Beim Beenden wird nicht mehr gefüttert -> der Watchdog löst nach dem Timeout aus.


def main() -> None:
    logging.basicConfig(level=logging.INFO)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    run(stop)


if __name__ == "__main__":
    main()
