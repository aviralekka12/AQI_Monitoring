---
description: Compile and upload Arduino sketch to Mega 2560
---

# Arduino Upload Workflow

This workflow compiles and uploads the current Arduino sketch to an Arduino Mega 2560 on COM3.

## Steps

// turbo-all

1. **Compile the sketch**
   ```powershell
   & "C:\Program Files\Arduino CLI\arduino-cli.exe" compile --fqbn arduino:avr:mega:cpu=atmega2560 .
   ```

2. **Upload to Arduino Mega 2560 on COM3**
   ```powershell
   & "C:\Program Files\Arduino CLI\arduino-cli.exe" upload -p COM3 --fqbn arduino:avr:mega:cpu=atmega2560 .
   ```

## Notes
- Make sure **Serial Monitor is closed** before uploading
- If COM port changes, update the `-p COM3` parameter
- Working directory should be the Arduino project folder
