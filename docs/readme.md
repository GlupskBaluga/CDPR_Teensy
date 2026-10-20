# PlatformIO Commands

Run all commands from the project root directory.

## BUILD

**Linux:**
```bash
~/.venvs/platformio/bin/pio run
```

**macOS:**
```bash
~/.platformio/penv/bin/pio run
```

## UPLOAD

**Linux:**
```bash
~/.venvs/platformio/bin/pio run -t upload
```

**macOS:**
```bash
~/.platformio/penv/bin/pio run -t upload
```

## SERIAL MONITOR

**Linux:**
```bash
~/.venvs/platformio/bin/pio device monitor --project-dir . --baud 115200
```

**macOS:**
```bash
~/.platformio/penv/bin/pio device monitor --project-dir . --baud 115200
```
