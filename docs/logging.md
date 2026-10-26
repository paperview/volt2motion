# Logging the built joint

Phase 2 fits the model to the real joint. You give `volt2motion calibrate` two
logs from the machine, and it fits the motor and gearbox values, trains a small
correction for what the physics still misses, and checks both on the run it
did not train on.

```bash
volt2motion calibrate baseline.json --log cal.csv --validation-log pick.csv
```

To see the whole loop first without hardware, run
`volt2motion calibrate --virtual`.

## 1. Export the trajectories

The robot's firmware plays a list of setpoints and logs while it does.

```bash
volt2motion trajectory calibration --q-min -0.5 --q-max 1.0 -o cal_setpoints.csv
volt2motion trajectory pick-and-place --q-pick -0.4 --q-place 0.9 -o pick_setpoints.csv
```

Each file has these columns at 1 kHz:

| Column | Meaning |
|---|---|
| `t` | time from the start, s |
| `q_ref` | target joint angle, rad |
| `qd_ref` | target velocity, rad/s |
| `qdd_ref` | target acceleration, rad/s^2 |
| `payload` | 1 when the gripper should hold the payload, else 0 |

- **The calibration routine** mixes slow constant-speed sweeps (to separate
  constant friction from speed-dependent friction), fast moves (to load the
  motor and gearbox) and holds, with and without the payload. 90 seconds is
  the default (`--seconds`).
- **The validation run** can be any normal job. The pick-and-place export is
  a convenient one, but `calibrate` replays whatever setpoints are in the
  validation log, so a real production job works too.

**Before playing a file on the machine:**

- Set `--q-min` and `--q-max` inside your joint's safe range, away from
  hard stops and anything the arm could hit. The command prints the range and
  top speed it produced.
- Move the arm slowly to the first setpoint before starting. The file starts
  there, not where the arm happens to be.
- Keep the firmware current limit and an emergency stop active.

## 2. Log at 1 kHz

Log both runs with these columns, as CSV with a header row:

| Column | Meaning | Unit |
|---|---|---|
| `t` | time | s |
| `q_ref` | the setpoint being played | rad |
| `q` | measured joint angle | rad |
| `qd` | the controller's filtered joint velocity | rad/s |
| `payload` | 1 while the gripper holds the payload, else 0 | |
| `iq_ref` | q-axis current command | A |
| `iq`, `id` | measured dq currents, averaged over each 1 ms | A |
| `vd`, `vq` | dq voltages the driver applied, averaged over each 1 ms | V |
| `vbus` | DC bus voltage | V |
| `t_winding` | winding temperature | C |

Recommended extra columns: `qd_ref` and `qdd_ref`. Without them, `calibrate`
works out velocity and acceleration from `q_ref`, which rounds off the sudden
speed changes at the start of each constant-speed sweep.

Extra columns are ignored, so you can log more than this.

### Conventions

Getting one of these backwards is the most common reason a fit comes out
wrong.

- **Angles are at the joint output**, after the gearbox. 0 is horizontal and
  positive lifts the arm against gravity. If your encoder's zero is elsewhere,
  subtract the offset before logging.
- **Positive `iq` makes positive torque**, lifting the arm. If your driver's
  sign is opposite, flip `iq`, `iq_ref`, `vq` and `vd` in the log.
- **Amplitude-invariant dq.** Currents and voltages are phase peak values, the
  convention where the dq transform keeps a 2/3 factor. If your firmware uses
  the power-invariant transform, multiply currents and voltages by 0.816
  before logging.
- **Averages, not snapshots.** `iq`, `id`, `vd` and `vq` should be averaged
  over each 1 ms log period. The current loop runs much faster, and single
  samples are noisy.
- **Even spacing.** Rows must be exactly 1 ms apart. If your logger jitters,
  resample to an even 1 kHz grid before running `calibrate`.

### No winding temperature sensor?

Log a constant, such as the room temperature, in `t_winding`. Keep the runs
short and cool (the default 90 second routine is fine), so the winding does
not heat up during the run. The fitted resistance then refers to that
temperature.

## 3. Run calibrate and read the output

`volt2motion calibrate` prints four steps:

1. **Physics fit**: resistance, flux linkage, inductance, gearbox friction
   and efficiency, next to the design values. Large differences are worth
   understanding before the next hardware revision.

   It also checks the torque constant two ways, from back-EMF and from motion
   alone. If they disagree by more than 8%, the usual causes are:
   - link or payload masses in the design file that do not match the real arm
   - the current sensor gain
   - dq scaling: about 37% apart when the firmware logs power-invariant
     currents and voltages, about 22% when only the currents are scaled

   If a sign is wrong in the log, the fit returns impossible values such as
   negative magnet flux or an efficiency above 1. `calibrate` checks for this,
   says which signals are probably flipped, and stops without saving anything.

2. **Learned correction**: torque prediction error on the validation run for
   the design model, the fitted physics, and fitted physics plus the learned
   correction. Each should be lower than the one before.

3. **Simulation against the log**: each model simulates the validation job and
   is compared with what the machine did. This is the end-to-end test of the
   simulator.

4. **Honing**: what to flash next.

Outputs, in `out/` unless you pass `--out`:

| File | What it is |
|---|---|
| `<name>_calibrated.json` | The design with fitted motor and gearbox values. Use it as the design file from now on. |
| `residual_model.npz` | The learned torque correction, for use in Python simulations. |
| `calibration_check.png` | The log against each model's simulation. |

## 4. Hone the firmware

The firmware's feedforward (the torque it adds ahead of the position error)
uses the motor and gearbox values. Copy these from `<name>_calibrated.json`
into the firmware: `motor.flux_linkage` (torque constant
`1.5 x pole_pairs x flux_linkage`), `gearbox.coulomb`, `gearbox.viscous` and
`gearbox.efficiency`. Then rerun the job, log it, and run `calibrate` again
with the new log to confirm tracking improved.

## Using the learned correction in Python

```python
from volt2motion import JointDesign, simulate, pick_and_place
from volt2motion.calibrate import ResidualModel

design = JointDesign.load("out/baseline_calibrated.json")
residual = ResidualModel.load("out/residual_model.npz")
log = simulate(design, pick_and_place(cycles=4), residual=residual)
```

The correction is switched off near standstill (below 0.02 rad/s), where
MuJoCo's friction model handles sticking, and its output is limited to the
range it saw in training.
