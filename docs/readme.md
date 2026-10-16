# i just need to remember these ocmmands

run from root: 

BUILD: 
~/.venvs/platformio/bin/pio run --project-conf cdpr_teensy.ini
~/.venvs/platformio/bin/pio run

UPLOAD: 
~/.venvs/platformio/bin/pio run --project-conf cdpr_teensy.ini -t upload
~/.venvs/platformio/bin/pio run -t upload


to see serial output:
~/.venvs/platformio/bin/pio device monitor --project-dir . --baud 115200