# Telemetry Logs

Controller CSV logs are written here when a controller is started with `--log-csv`.

Example:

```powershell
python scripts\launchers\run_position_ui_controller.py --log-csv
python scripts\analysis\analyze_telemetry.py logs\<csv-file-name>.csv
```
