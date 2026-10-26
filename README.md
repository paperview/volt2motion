# volt2motion

Simulate a robot joint from battery voltage to arm motion.

A simulator for one robot joint that covers the whole chain: battery and DC
bus, driver PCB, FOC current loop, motor electrics, heat, gearbox friction, and
the arm's motion in MuJoCo. You use it in two phases for the same robot:
first to design the joint, then to calibrate the model against the built one.

## The approach: physics first, then real data

Neither physics nor learning is enough on its own.

- **Physics models** can predict a machine that does not exist yet, so they
  are what you design with. But their parameters are never exactly right, and
  some effects (breakaway friction, gear imperfections) are hard to model.
- **Learned models** (like the ETH Zurich ANYmal "actuator network") match real
  hardware closely, but they need the hardware to exist, and they only know
  conditions that appeared in their training data.

So this project uses both, in order:

| Phase | When | What it does | Command |
|---|---|---|---|
| 1. Design | Before building | Simulate the design from datasheet, CAD and FEA values. Check torque, voltage headroom, bus voltage, heat over a shift. Change parameters until every check passes. | `volt2motion design` |
| 2. Calibrate and hone | After building | Log the real joint. Fit the physics parameters to the logs, then train a small neural network on what the physics still gets wrong. Reflash the firmware with the calibrated values. | `volt2motion calibrate` |

The learned part is a **correction on top of physics**, not a replacement. The
physics fit stays interpretable (you can compare fitted resistance and friction
to the datasheet and fix the next hardware revision), and the network only has
to learn the leftovers.

## Quick start

Needs Python 3.12 or newer. Install straight from the repository:

```bash
python3 -m venv ~/.volt2motion
~/.volt2motion/bin/pip install "git+https://github.com/paperview/volt2motion.git"
source ~/.volt2motion/bin/activate      # puts the volt2motion command on your PATH
```

Add `@v0.1.0` (a tag) or `@main` (a branch) to the end of the URL to pick a
version. pip fetches the other packages it needs (MuJoCo, NumPy, SciPy,
Matplotlib, scikit-learn) from PyPI. If you use `pipx` or `uv`,
`pipx install git+https://...` and `uv tool install git+https://...` set up
the environment for you.

To work on the code itself, clone it and install it editable:

```bash
git clone https://github.com/paperview/volt2motion.git && cd volt2motion
python3 -m venv .venv
.venv/bin/pip install -e ".[dev]"     # code changes take effect without reinstalling
source .venv/bin/activate
pytest
```

Then, in any folder you want to work in:

```bash
volt2motion init demo && cd demo   # baseline.json, parts.json and a sample KiCad board

# Phase 1: check a design
volt2motion design baseline.json
volt2motion design baseline.json --set arm.payload_mass=10 --set gearbox.ratio=80

# Fill in the driver values from a KiCad board, then check the result
volt2motion import-kicad sample_driver --run

# Phase 2, before hardware exists: run the loop against a synthetic stand-in machine
volt2motion calibrate --virtual

# Phase 2, with real hardware
volt2motion calibrate baseline.json --log cal.csv --validation-log pick.csv
```

Every command writes to `out/` unless you pass `--out`. Run
`volt2motion <command> --help` for all options. `python -m volt2motion` works
too. Run the tests from the source folder with `pytest`.

| Command | What it does |
|---|---|
| `init [folder]` | Writes `baseline.json` (every design parameter, ready to edit), `parts.json` (datasheet values for the KiCad importer) and a sample driver board. Keeps files that already exist. |
| `design [design.json]` | Runs pick-and-place cycles at full detail, then a long shift at fast detail, and prints pass or fail for each check. `--set KEY=VALUE` tries a change without editing the file. |
| `import-kicad <board>` | Reads a KiCad project folder or `.kicad_pcb` and writes `out/<name>.json`. `--run` runs the design check on it. |
| `calibrate [design.json]` | Fits the model to logs from the machine (`--log`, `--validation-log`), or to a synthetic one (`--virtual`). |
| `trajectory <kind>` | Exports the calibration routine or a pick-and-place job as setpoints for the robot's firmware to play while logging. |
| `params [filter]` | Lists every design parameter with its unit, default and meaning. `--design file.json` shows a file's values. |

## Guides

| Guide | Read it when |
|---|---|
| [The design file](docs/design-file.md) | Filling in your motor, gearbox, battery and arm, and reading the design check |
| [Design parameters](docs/parameters.md) | Looking up any key, its unit and default |
| [Preparing a KiCad board](docs/kicad-board.md) | Getting your driver board ready for `import-kicad` |
| [The parts table](docs/parts-table.md) | Adding your MOSFETs' datasheet values |
| [Logging the built joint](docs/logging.md) | Recording logs on the real machine and running `calibrate` |

## What is modeled

| Layer | Model | File |
|---|---|---|
| Battery and bus | Li-ion pack with internal resistance, bus capacitor, brake chopper, over and undervoltage faults, regen into a supply that cannot sink current | `power.py` |
| Driver PCB | FOC PI current loop with decoupling, one-period computation delay, current sensor noise and quantization, SVPWM voltage limit, MOSFET conduction, switching and dead-time losses, thermal current derating | `actuator.py` |
| Motor | PMSM in the rotor frame, temperature-dependent resistance and magnet flux, iron loss, cogging, optional torque-constant curve from FEA | `actuator.py` |
| Heat | Winding, housing and MOSFET nodes, plus steady-state prediction for any duty cycle | `thermal.py` |
| Gearbox | Ratio, reflected rotor inertia, Coulomb and viscous friction, load-dependent efficiency loss, breakaway and position-dependent friction | `joint.py` |
| Arm | MuJoCo hinge with link and gripper payload, real stick-slip through MuJoCo joint friction | `joint.py` |
| Control | Quantized motor encoder, filtered velocity, PD plus model feedforward | `controller.py` |

All parameters live in one JSON design file. See [the design file guide](docs/design-file.md).

### Two detail levels

- `fidelity="full"` steps the current loop and motor electrics every PWM
  period (50 us). About 13x faster than real time.
- `fidelity="fast"` assumes the current loop tracks within its voltage limit
  and steps every 1 ms. About 100x faster than real time, for long runs such as
  a full shift. The two agree within a few percent on power, current and
  temperature (checked in the tests).

The driver is an averaged model: it keeps the effects of PWM switching
(voltage limit, losses) but not individual switching edges. Use LTspice for
switching transients and gate drive.

### From a KiCad board

`volt2motion import-kicad` reads a motor driver's `.kicad_pcb` (and
`.kicad_sch` if present) and fills in the driver, bus and MOSFET cooling
parameters:

```bash
volt2motion import-kicad sample_driver --run                  # the board from init
volt2motion import-kicad ~/kicad/my_driver --base baseline.json
volt2motion import-kicad my_driver.kicad_pcb --convection 40   # with a fan
```

| From the board | Becomes |
|---|---|
| MOSFET part numbers, looked up in `parts.json` | conduction resistance and its temperature rise, gate charge |
| Pad pin functions (G, D, S) and nets | which MOSFETs are high and low side, which nets are phases, bus and ground |
| Shunt value and position, `SIM_GAIN`, `SIM_ADC_BITS` | current sensing resolution and range |
| Gate resistors, `SIM_VDRIVE`, `SIM_ISOURCE`, `SIM_ISINK` | switching time, hence switching loss |
| Copper pours and tracks on the phase nets, stackup thickness | copper resistance between the bridge and the motor connector |
| Capacitors across the bus, `SIM_DERATE`, voltage ratings | bus capacitance, plus a voltage rating check |
| Copper area near the MOSFETs | MOSFET-to-air thermal resistance |

Datasheet values KiCad files do not hold go in `parts.json`, keyed by the
part's `MPN` field. Firmware values (PWM frequency, current limit) stay from
the base design. Every derived value is printed with its source and marked
"from board" or "estimate". [Preparing a KiCad board](docs/kicad-board.md)
covers pin names, fields and the optional `SIM_*` properties.

Try it on the sample board that `volt2motion init` writes. The import flags
two problems with it, which shows the kind of thing it catches:

- **Bus capacitors rated too low.** C1 and C2 are rated 63 V. The driver lets
  the bus rise to 60 V before it shuts down, and the check wants 20% headroom
  above that, so 72 V.
- **MOSFETs run hot.** With only the board's copper to cool them, they settle
  at about 80 C over a shift, against the 43 C the default design assumes. The
  design still passes, but with little margin before the driver starts cutting
  current at 100 C.

The sample board is only for trying the importer. It opens in KiCad but is not
a working layout, so do not send it for manufacture.

### Where FEA plugs in

Run your motor in FEMM, Elmer or Ansys Maxwell, then enter the results as
`motor.kt_curve_current` and `motor.kt_curve_scale` (torque constant falling
off as the iron saturates), plus resistance, inductance and flux. You run FEA
once per motor design, not inside the simulation loop.

## Phase 2 in detail

### 1. Log the built joint

Export the calibration routine with `volt2motion trajectory calibration`, play
it on the joint while logging at 1 kHz, then log a normal job as the held-out
validation run. [Logging the built joint](docs/logging.md) has the columns,
sign conventions and safety notes.

### 2. Fit and learn

`volt2motion calibrate` then:
1. Fits winding resistance, magnet flux and inductance from the voltage
   equations, and gearbox friction and efficiency from the motion.
2. Cross-checks the torque constant two ways (back-EMF versus motion), and
   stops if a sign mistake in the log made the fit unphysical.
3. Trains the residual network on the calibration run.
4. Scores each model on the held-out run, both open-loop torque prediction and
   a full closed-loop simulation of the same job compared to the log.
5. Saves `<name>_calibrated.json` and `residual_model.npz`. Use them in
   `simulate(..., residual=ResidualModel.load(...))`.

### 3. Hone the machine

Flash the calibrated feedforward values into the firmware, rerun the job, log
it and run `calibrate` again. With the virtual machine this step cuts tracking
error by about half.

### Results on the virtual machine

The virtual machine (`virtual_hardware.py`) is synthetic: the same simulator
with hidden parameters that differ from the design (12% higher resistance, 6%
weaker magnets, 60% more friction) plus effects the design model leaves out.
Calibration never reads the hidden values. Typical output:

| Model | Torque error on held-out run | Closed-loop sim vs log |
|---|---|---|
| Design (phase 1) | 5.5 Nm rms | 0.088 deg, 1.5 A rms |
| Calibrated physics | 1.2 Nm rms | 0.019 deg, 0.32 A rms |
| Calibrated physics + residual | 0.7 Nm rms | 0.017 deg, 0.29 A rms |

Real hardware will be messier. Treat these numbers as proof the loop works, not
as a prediction.

## Known limits and next steps

- **One joint.** Multi-joint arms, with joints sharing one power bus and
  moving each other's links, are not supported yet.
- **Heat is not calibrated.** `calibrate` fits the motor and gearbox but not
  the thermal model. Heat builds over minutes, so checking it needs a separate
  30-60 minute log. Compare that log's winding temperature with
  `volt2motion design`'s shift run by hand.
- **No field weakening.** The d-axis current is held at zero, so top speed is
  set by back-EMF.
- **Residual near standstill.** The learned correction is off below 0.02 rad/s
  and phases in up to 0.05 rad/s. MuJoCo's static friction handles standstill.
  There is a small tracking blip as the arm slows into a hold.
- **No backlash or gearbox compliance.** Add these for harmonic or cycloidal
  drives if the logs show them.
- **Speed.** The simulation is pure Python: fast enough for design and
  calibration, too slow for reinforcement learning at scale, which would need
  the actuator model compiled as a MuJoCo plugin.

## License

MIT. See [LICENSE](LICENSE).
