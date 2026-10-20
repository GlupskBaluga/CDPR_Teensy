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

## Interactive planar kinematics test

The placeholder robot has four cables and three planar degrees of freedom:
X, Y, and phi (counterclockwise rotation). It starts at `(0, 0, 0)`.
Frame anchors are at the corners of a 2000 mm square, with platform attachment
points at the corners of a 300 mm square. This is a simulation; no motors move.

Upload from the project root:

```bash
~/.venvs/platformio/bin/pio run -t upload
```

Close the PlatformIO serial monitor with Ctrl+C before opening the plotter.
Launch the installed Arduino IDE:

```bash
arduino-ide
```

Select the Teensy's current serial port under Tools > Port, then open
Tools > Serial Plotter and select 115200 baud. You can also use the plotter
button at the top right of the IDE. See the
[Arduino Serial Plotter documentation](https://docs.arduino.cc/software/ide-v2/tutorials/ide-v2-serial-plotter).

Send commands from the plotter's input field:

```text
goto(10,10,3)
goto(-50,25,-10)
goto(0,0,0)
```

X and Y are in millimetres, phi is in degrees. The closing `)` completes a command;
newline or CRLF endings also work. Commands immediately change the simulated pose;
there is no trajectory interpolation. Malformed or oversized commands are ignored,
leaving the previous pose in place. The plotter receives four labelled cable lengths
in millimetres at 50 Hz. At the initial pose all four curves overlap at about
1202.082 mm.

In `src/main.cpp`, `CONTROL_PERIOD_US` controls the calculation rate (1000 = 1 kHz,
0 = unrestricted), and `PLOT_PERIOD_MS` controls plotting (20 = 50 Hz). Set
`PLOT_CABLE_LENGTHS = false` to restore benchmark timing text instead of plot samples.
Ethernet stays disabled by default so no network link is needed.
