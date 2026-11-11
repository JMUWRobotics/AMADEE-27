import asyncio
import logging
from logging.handlers import RotatingFileHandler
import sys
import signal
from pathlib import Path

from raspcode.core.state_machine import StateMachine

logger = logging.getLogger(__name__)

def setup_logging(log_dir: Path = Path("log")) -> None:
    """Configure centralized logging for console and a rotating file."""
    # Ensure the log directory exists (corresponds to your root 'log/' folder)
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file = log_dir / "raspcode.log"

    # Get the root logger 
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.DEBUG)  # Capture DEBUG and above globally

    # Prevent duplicate handlers if setup is accidentally called twice
    if root_logger.hasHandlers():
        root_logger.handlers.clear()

    # Define a clean, uniform log format
    formatter = logging.Formatter(
        "%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S"
    )

    # Console Handler (Outputs to terminal / stdout)
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.DEBUG)
    console_handler.setFormatter(formatter)
    root_logger.addHandler(console_handler)

    # Rotating File Handler (Saves to log/raspcode.log)
    # maxBytes=5MB, backupCount=5 keeps max ~30MB of logs total to protect Pi SD cards
    file_handler = RotatingFileHandler(
        log_file, 
        maxBytes=5 * 1024 * 1024, 
        backupCount=5, 
        encoding="utf-8"
    )
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(formatter)
    root_logger.addHandler(file_handler)

    # Optional: Silence or reduce verbosity of chatty third-party libraries
    logging.getLogger("yaml").setLevel(logging.WARNING)


async def main() -> None:
    """Application entry point."""
    project_root = Path(__file__).resolve().parents[2]

    setup_logging(log_dir=project_root / "log")

    logger.info("Initializing RaspCode application...")

    # Initialize state machine
    CONFIG_PATH = project_root / "config" / "system.yaml"
    sm = StateMachine(config_file=CONFIG_PATH)

    # Setup signal handling for graceful shutdown (Ctrl+C or systemd stop)
    loop = asyncio.get_running_loop()
    task = asyncio.current_task()

    for sig in (signal.SIGTERM, signal.SIGINT):
        try:
            loop.add_signal_handler(sig, lambda: task.cancel() if task else None)
        except (NotImplementedError, RuntimeError):
            # Signal handlers might not work on non-main threads or certain platforms
            pass

    try:
        await sm.run()
    except asyncio.CancelledError:
        logger.info("Application execution cancelled.")
    except Exception as e:
        logger.critical(f"Unhandled critical exception in main loop: {e}", exc_info=True)
        raise


if __name__ == "__main__":
    # Configure global logging formatting
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S"
    )
    
    # Optional: adjust third-party loggers if they are too chatty
    logging.getLogger("yaml").setLevel(logging.WARNING)

    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Program terminated manually via KeyboardInterrupt.")