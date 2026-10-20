import asyncio
import logging
from typing import Optional
from control.adapter_manager import AdapterManager, AdapterState

logger = logging.getLogger("DebugManager")


class DebugManager:
    def __init__(self, charging_manager: AdapterManager) -> None:
        self.charging_manager = charging_manager

    def _print_help(self) -> None:
        print("\n" + "=" * 50)
        print("            DEBUG MANAGER COMMANDS")
        print("=" * 50)
        print("  list                       - List all registered power supplies")
        print("  set <addr> <volt> <power>  - Set voltage (V) and power (W)")
        print("  off <addr>                 - Turn off output for a device")
        print("  on <addr>                  - Turn on output for a device")
        print("  setup <addr>               - Reset device to safe defaults")
        print("  exit                       - Exit DEBUG state -> Return to IDLE")
        print("  help                       - Display this help menu")
        print("=" * 50 + "\n")

    async def run_cli_session(self) -> None:
        """Interactive non-blocking CLI loop for DEBUG mode."""
        print("\n=== SYSTEM IN DEBUG MODE ===")
        self._print_help()

        while True:
            try:
                # Wait for keyboard input in a separate thread without blocking the main event loop
                user_input = await asyncio.to_thread(input, "DEBUG> ")
                cmd_parts = user_input.strip().split()

                if not cmd_parts:
                    continue

                cmd = cmd_parts[0].lower()

                # --- EXIT ---
                if cmd in ("exit", "quit", "q"):
                    print("Exiting DEBUG mode...")
                    break

                # --- HELP ---
                elif cmd == "help":
                    self._print_help()

                # --- LIST ---
                elif cmd == "list":
                    print("\nRegistered Buck Converters:")
                    for addr, dev_wrapper in self.charging_manager.devices.items():
                        alloc_p = self.charging_manager.allocated_power.get(addr, 0.0)
                        max_v = 65.0 if dev_wrapper.boosted else 50.0
                        print(
                            f"  - Address {addr:2d} | Name: {dev_wrapper.name} "
                            f"| Boosted: {dev_wrapper.boosted} (max {max_v}V) "
                            f"| Allocated: {alloc_p:.1f}W"
                        )
                    print()

                # --- SET VOLTAGE & POWER ---
                elif cmd == "set":
                    if len(cmd_parts) < 4:
                        print("Error: Usage is 'set <address> <voltage_V> <power_W>'")
                        continue

                    addr = int(cmd_parts[1])
                    voltage = float(cmd_parts[2])
                    power = float(cmd_parts[3])

                    success = await self.charging_manager.request_output(addr, voltage, power)
                    if success:
                        print(f"-> Device {addr} set successfully: {voltage:.2f}V | {power:.1f}W")
                    else:
                        print(f"-> Failed to set device {addr}. Check parameters or hardware limits.")

                # --- DISABLE OUTPUT ---
                elif cmd == "off":
                    if len(cmd_parts) < 2:
                        print("Error: Usage is 'off <address>'")
                        continue

                    addr = int(cmd_parts[1])
                    success = await self.charging_manager.disable_output(addr)
                    if success:
                        print(f"-> Device {addr} output disabled.")
                    else:
                        print(f"-> Failed to disable device {addr}.")

                # --- DISABLE OUTPUT ---
                elif cmd == "on":
                    if len(cmd_parts) < 2:
                        print("Error: Usage is 'on <address>'")
                        continue

                    addr = int(cmd_parts[1])
                    success = await self.charging_manager.enable_output(addr)
                    if success:
                        print(f"-> Device {addr} output enabled.")
                    else:
                        print(f"-> Failed to enable device {addr}.")

                # --- RESET / SETUP DEVICE ---
                elif cmd == "setup":
                    if len(cmd_parts) < 2:
                        print("Error: Usage is 'setup <address>'")
                        continue

                    addr = int(cmd_parts[1])
                    success = await self.charging_manager.setup_device(addr)
                    if success:
                        print(f"-> Device {addr} re-initialized (0V/0A, Output OFF).")
                    else:
                        print(f"-> Failed to initialize device {addr}.")

                else:
                    print(f"Unknown command: '{cmd}'. Type 'help' for command overview.")

            except ValueError as ve:
                print(f"Input error (number expected): {ve}")
            except Exception as e:
                print(f"Command execution error: {e}")