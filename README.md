# Smart Railway Level-Crossing Safety Controller

An Arduino Uno controller for a **scale-model, single-track railway crossing**, simulated in Tinkercad Circuits. It counts train axles, estimates train speed to time warnings, and controls road signals, railway flashers, a buzzer, and a servo barrier. The controller also has emergency override and latched fault handling.

> **Safety:** This is an educational prototype for a model track. It is not a certified railway signalling or safety system and must not be used to control a real crossing.

## Project files

- [Arduino sketch](<level_crossing (3).ino>) — controller state machine, sensor handling, alerts, and serial event log.
- [EAGLE board layout](Case_c.brd) — board design in EAGLE XML format, using Tinkercad component libraries.
- [Project report](Level_Crossing_Project_Report.docx) — design rationale, hardware, timing, simulation procedure, results, and limitations.
- [Project presentation](Railway_Crossing_Controller_v3.pptx) — project overview and design summary.

The report describes a Tinkercad Circuits simulation, but a Tinkercad share link or circuit export is not included here.

## How it works

Two force-sensitive resistors (FSRs) detect axles at the entry and exit of the protected section. The controller counts each axle and keeps the crossing closed until the number of axles detected at the exit balances the entry count. The time between the first two entry axles provides an estimated speed and arrival time. If a second axle is not detected within 1.5 seconds, the controller proceeds with a fail-safe warning sequence.

A non-blocking state machine sequences the road lights, alternating railway warning LEDs, buzzer, and servo barrier. An interrupt-driven pushbutton requests emergency closure; a second press releases emergency mode. Sensor wire breaks, a stuck sensor, an exit axle detected before entry, and an occupied crossing timeout trigger fault or stall handling.

## Hardware

The project materials specify:

- Arduino Uno R3
- Two FSRs for entry and exit axle detection
- SG90-type micro servo for the barrier
- Six road LEDs (two each for green, yellow, and red) and two railway warning LEDs
- Piezo buzzer and momentary emergency pushbutton
- 10 kΩ × 2, 470 kΩ × 2, 470 Ω × 6, 330 Ω × 2, and 220 Ω × 1 resistors
- 470 µF capacitor across the servo supply, as specified in the report

Each FSR uses a 10 kΩ pull-down and a 470 kΩ bypass path. The report uses the bypass to distinguish a cut sensor wire from an idle sensor. **The supplied `Case_c.brd` element list does not include the 470 µF capacitor; check the hardware design before building the circuit.**

### Arduino pin map

| Arduino pin | Connection |
| --- | --- |
| A0 | Entry FSR voltage-divider node |
| A1 | Exit FSR voltage-divider node |
| D2 | Emergency button to GND (`INPUT_PULLUP`, interrupt) |
| D3 | Road green LEDs |
| D4 | Road yellow LEDs |
| D5 | Road red LEDs |
| D6, D7 | Alternating railway warning LEDs |
| D8 | Piezo buzzer through 220 Ω resistor |
| D9 | Servo signal |
| 5 V, GND | Sensor and circuit power rails |

The two road approaches share the green, yellow, and red outputs, with two LEDs of each color on the corresponding pin.

## Build and run

1. Open the `.ino` sketch in the Arduino IDE and select **Arduino Uno** as the board. The sketch uses the Arduino `Servo` library.
2. Wire the sensors and outputs according to the pin map and the project report. Connect the emergency button between D2 and GND.
3. Adjust the geometry constants near the top of the sketch for the model: axle spacing (60 mm by default), entry-sensor-to-crossing distance (600 mm), and sensor spacing (800 mm).
4. Upload the sketch and open the Serial Monitor at **9600 baud** to see state changes, sensor readings, axle counts, and train logs.
5. In Tinkercad Circuits, simulate an axle by raising an FSR slider and returning it to zero. The sketch defaults to press/release thresholds of 300/150 ADC counts and a 40 ms debounce.

Normal operation uses a 2-second yellow phase and closes the barrier in 1.35 seconds. Fast-train mode shortens these to 1 second and 0.45 seconds. The warning target is 5 seconds before the estimated arrival; if the estimated arrival is too close, the sketch reports that the barrier may close late.

## Fault behavior and limitations

- A sensor reading below 3 ADC counts for 300 ms is treated as an open circuit when the bypass wiring is used.
- A sensor held pressed for 30 seconds or an occupied crossing held for 30 seconds is treated as stuck or stalled.
- A fault remains latched until the operator acknowledges it while both sensors report healthy.
- Speed prediction assumes constant speed and uses the first two entry axles. The report notes that an individual FSR element failure can appear as no train because the bypass resistor remains connected.
- The prototype uses a single controller and has no certified safety redundancy. Its timing and sensor behavior are intended for a scale model only.

## Credits

Team 1: V Chinmaya Datta, N Jathin, Ch Sai Krishna, and S Konda Reddy.