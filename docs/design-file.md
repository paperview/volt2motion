# The design file

A design file is a JSON file that describes one joint: motor, driver board,
gearbox, battery, heat paths, the arm it moves and the control loop.
`volt2motion design`, `import-kicad` and `calibrate` all read one.

For the full list of keys with units and defaults, see
[parameters.md](parameters.md) or run `volt2motion params`.

## Getting one

```bash
volt2motion init my_joint        # writes my_joint/baseline.json with every parameter
```

Copy it for each variant you want to compare (`motor_a.json`, `motor_b.json`)
and change the `name` field, which names the output files.

## Editing rules

- **Units are SI**: ohm, henry, weber, kg, m, seconds, radians, and degrees C
  for temperatures. `120e-6` is 120 microhenry.
- **Typos are errors.** An unknown key stops the run and suggests the closest
  match:
  `unknown design parameter 'gearbox.ratoi'. Did you mean 'gearbox.ratio'?`
- **Missing keys take their defaults.** A file can hold just the values you
  changed:
  ```json
  { "name": "heavy", "arm": { "payload_mass": 10 } }
  ```
- **Comments**: JSON has none, so any key starting with `_` is ignored. Use it
  for notes:
  ```json
  "gearbox": { "_source": "Harmonic CSF-20 datasheet, rev 3", "ratio": 100 }
  ```
- **Try a change without editing the file** with `--set`, repeated as needed:
  ```bash
  volt2motion design baseline.json --set gearbox.ratio=80 --set arm.payload_mass=8
  ```
- **Check what a file holds** with `volt2motion params --design file.json`,
  optionally filtered: `volt2motion params --design file.json motor`.

## Where the values come from

### Motor

The model is per phase, in the star (wye) equivalent. This is where most
mistakes happen, because datasheets usually quote values measured between two
motor leads.

| Key | How to get it |
|---|---|
| `pole_pairs` | Half the magnet count. A "12N14P" motor has 14 magnets, so 7 pole pairs. |
| `resistance` | **Half** the line-to-line resistance. Measure between any two leads with a milliohm meter (or a 4-wire measurement) at room temperature, then halve it. For a delta-wound motor this still gives the correct star equivalent. |
| `ld`, `lq` | **Half** the line-to-line inductance. Use the same value for both unless the motor has interior magnets and the datasheet gives Ld and Lq. |
| `flux_linkage` | See below. |
| `rotor_inertia` | Datasheet, or the rotor's CAD model. |
| `iron_loss_hyst`, `iron_loss_eddy` | Optional. Spin the motor unloaded at two speeds and record input power. Subtract the copper loss, then fit `P = hyst x w + eddy x w^2` with `w` in rad/s at the motor shaft. They only affect heat, so the defaults are fine to start. |
| `cogging_amp`, `cogging_periods` | Optional. Periods are the least common multiple of slots and magnets (12 and 14 give 84). |
| `kt_curve_current`, `kt_curve_scale` | Optional. The torque constant falling off as the iron saturates, from FEA (FEMM, Elmer, Maxwell). Two lists of the same length: phase current points and the fraction of the unsaturated torque constant at each. |

**Flux linkage.** It sets both torque per amp and back-EMF. The model uses
amplitude-invariant dq currents (phase peak amps), so torque is
`1.5 x pole_pairs x flux_linkage x iq`. Pick whichever source you have:

| You have | Flux linkage in Wb |
|---|---|
| Your own measurement (best) | Spin the motor with a drill and scope two leads. `V_LL,peak / (1.732 x pole_pairs x w)`, with `w` the shaft speed in rad/s. |
| Ke in V (line to line, rms) per 1000 rpm | `Ke x 1.414 / 104.7 / (1.732 x pole_pairs)` |
| Ke in V (line to line, peak) per rad/s | `Ke / (1.732 x pole_pairs)` |
| Kt in Nm per A rms | `Kt / (1.414 x 1.5 x pole_pairs)` |
| Kt in Nm per A peak | `Kt / (1.5 x pole_pairs)` |
| Kv in rpm/V (hobby motors) | `60 / (6.283 x Kv x 1.732 x pole_pairs)`. Kv ratings are often loose, so measure when you can. |

Worked example: a 12N14P motor rated 100 rpm/V gives
`60 / (6.283 x 100 x 1.732 x 7) = 0.0079 Wb`, so Kt is
`1.5 x 7 x 0.0079 = 0.083 Nm` per phase peak amp.

If this is wrong, the torque constant cross-check in `calibrate` will catch it
once the joint is built.

### Driver board

`volt2motion import-kicad` fills in the board values for you (see the
[KiCad guide](kicad-board.md)): `driver.rds_on`, `rds_tempco`, `switch_time`,
`dead_time`, `current_sense_lsb`, `current_sense_noise`,
`battery.bus_capacitance`, `battery.brake_resistance`,
`thermal.r_fet_ambient` and `thermal.fet_capacity`.

The rest are firmware settings and must match what you flash: `pwm_freq`,
`current_limit`, `current_loop_bw_hz`, `max_modulation`, `encoder_bits`, and
the four derating temperatures.

### Gearbox

| Key | How to get it |
|---|---|
| `ratio` | Datasheet. |
| `efficiency` | Datasheet efficiency at rated load. Harmonic drives are often 0.6 to 0.8, planetary stages about 0.9 to 0.95 each. |
| `rated_peak_torque` | The repeated peak torque rating, not the momentary (crash) rating. |
| `coulomb`, `viscous` | Hard to know before building. A starting guess for `coulomb` is the datasheet's no-load running torque at the input, times the ratio. `calibrate` fits both from the real joint. |
| `stribeck_*`, `position_friction_amp` | Leave at 0. They exist so the virtual test machine can have effects the design model leaves out. |

### Battery and bus

| Key | How to get it |
|---|---|
| `cells_series`, `capacity_ah` | Pack spec. The model assumes Li-ion cells (about 3.0 V empty to 4.2 V full). |
| `internal_resistance` | Cell resistance x series count / parallel count, plus wiring and connectors. |
| `accepts_regen` | Set `false` when testing on a bench supply that cannot absorb current. Braking energy then has to go into the brake resistor. |
| `brake_on_v`, `brake_off_v`, `overvoltage_v`, `undervoltage_v` | Firmware thresholds. Keep them in this order: full pack voltage < `brake_off_v` < `brake_on_v` < `overvoltage_v`, and `undervoltage_v` below the empty pack voltage (13 cells x 3.0 V = 39 V). |
| `aux_load_w` | Logic, encoder and communications power. |

### Heat

| Key | How to get it |
|---|---|
| `winding_capacity` | Copper mass in kg x 385. |
| `housing_capacity` | Stator plus housing mass in kg x about 460 (steel) or 900 (aluminium). |
| `r_winding_housing`, `r_housing_ambient` | Motor datasheet thermal resistances. If it gives only winding to ambient, split it roughly 40 / 60. `r_housing_ambient` depends on what the motor is bolted to. |
| `winding_limit_c` | The insulation class limit (class F is 155 C, class H 180 C) minus a margin, and below the magnets' maximum temperature. |
| `ambient_c` | The warmest air the joint will see in service. |

The heat model is not fitted by `calibrate` yet. To check it, run the joint
through a 30 to 60 minute job, log winding temperature, and compare with
`volt2motion design`'s shift run.

### Arm

The joint drives one link about a horizontal axis. Angle 0 is horizontal and
a positive angle lifts the link. The link is a uniform rod from the joint to
the payload, and the payload is a point mass at its end.

For a real arm with more joints, use the worst case for this joint: the
longest reach and the heaviest load beyond it. Set `link_mass` and
`link_length` so the gravity torque matches,
`9.81 x (link_mass x link_length / 2 + payload_mass x link_length)`.

### Control

`bandwidth_hz` is how quickly the position loop corrects errors. Higher values
track tighter but need more current, and a flexible structure will start to
shake. 5 to 15 Hz is typical for a geared arm joint.

## Reading the design check

`volt2motion design` prints a pass or fail for each check. When one fails:

| Check | What it means | What to try |
|---|---|---|
| No driver fault | The bus went over or under voltage and the driver shut down. | See the two bus checks below. |
| Tracking error under 1 deg | The arm lags its target. | Raise `control.bandwidth_hz`, or fix whichever current or voltage check is also failing. |
| Current limit rarely hit | The move needs more torque than `current_limit` allows. | Higher `gearbox.ratio`, a stronger motor (`flux_linkage`), a higher `current_limit` (check the board and heat), or slower moves (`--move-time`). |
| Voltage headroom | Back-EMF at top speed uses up the bus voltage. | Lower `gearbox.ratio`, a higher-Kv winding (lower `flux_linkage`, at the cost of more current per Nm), more `cells_series`, or slower moves. |
| Gearbox peak torque within rating | Peaks exceed `rated_peak_torque`. | A larger gearbox, a lighter payload, or slower moves. |
| Bus stays below brake threshold | Braking pushed energy back onto the bus and the brake resistor had to switch on. Fine if intended. | To avoid it: let the battery take regen (`accepts_regen`), add `bus_capacitance`, or slow the deceleration. If the bus also faults over voltage, lower `brake_resistance` so the brake absorbs energy faster. |
| Bus stays above undervoltage | Current peaks sag the bus. | Lower `internal_resistance` (thicker wiring, more cells in parallel), or check the run at a realistic `soc_init`. |
| Winding steady state under limit | The motor overheats over a shift. | Lower copper loss (higher ratio so less current, a motor with lower `resistance`), better cooling (`r_housing_ambient`), or a lighter duty cycle. |
| MOSFET steady state under limit | The driver overheats. | A heatsink or fan (`import-kicad --convection`, `SIM_HEATSINK_RTH`), lower-resistance MOSFETs, or more copper around them. |

The steady-state temperatures are where the joint settles if the job repeats
forever, which is why they can fail even when the run itself stayed cool.
