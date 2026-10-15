# team-2-software-firmware
Final Project

## Local GUI and Fake Firmware test

Start Mosquitto first. In PowerShell, start the installed service:

```powershell
Start-Service -Name mosquitto
```

If Mosquitto is not installed as a service, run the executable instead:

```powershell
& "C:\Program Files\Mosquitto\mosquitto.exe" -v
```

In a second PowerShell terminal, start Fake Firmware:

```powershell
uv run python -m rov_mqtt.fake_firmware
```

In a third PowerShell terminal, start the GUI:

```powershell
uv run python gui/testing.py
```

The simulator publishes plain-text `online`, retained initial `ESTOP` mode, telemetry on `robot/telemetry`, and robot heartbeats. It answers ping requests, confirms ESTOP/reset mode transitions, and logs simulated drive/arm commands without controlling hardware. Start the GUI before or after the simulator; retained status and mode plus periodic telemetry support either order. Press Reset ESTOP and wait for `MANUAL` confirmation before controller drive/arm commands are accepted. Press the PS4 OPTIONS button to request ESTOP.

The simulator and GUI both default to `localhost:1883`. To point the GUI at a broker address explicitly, set `ROV_MQTT_BROKER_HOST` in the GUI's PowerShell terminal before launching it. For a broker hosted on this laptop's `172.20.10.3` Wi-Fi address:

```powershell
$env:ROV_MQTT_BROKER_HOST = "172.20.10.3"
uv run python gui/testing.py
```

The environment setting selects the broker endpoint; it does not prove that a remote Firmware process is publishing the Software protocol's expected status, mode, telemetry, or pong messages.
For GUI-first testing, start Mosquitto, run `uv run python gui/testing.py`, then run `uv run python -m rov_mqtt.fake_firmware` in another terminal. For simulator-first testing, start them in the order shown above.

Run the automated unit and local-broker integration suite with either command:

```powershell
uv run pytest -v
uv run python -m unittest discover -s tests -v
```

The integration tests require Mosquitto running on `localhost:1883`; they skip when the broker is unavailable. Pytest only collects `tests/`; the top-level `test_controller.py`, `test_buttons.py`, and `test_qr_webcam.py` are interactive hardware scripts and are intentionally not run as automated tests.
