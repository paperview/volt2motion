# Preparing a KiCad board

`volt2motion import-kicad` reads a three-phase motor driver from its KiCad
files and fills in the driver, bus and MOSFET cooling values of a design. It
works out the circuit from the board itself, so you do not need special net
names or a particular layout. It does need a few things to be set up the way
KiCad's standard libraries do them.

```bash
volt2motion import-kicad path/to/project_folder --base baseline.json
volt2motion import-kicad path/to/board.kicad_pcb --run      # a single file works too
```

The values it cannot read from the board come from the `--base` design (motor,
gearbox, arm, firmware settings). Every value it sets is printed with its
source.

Tested with KiCad 8 and 10 files.

## Checklist

### 1. Files

- Point it at the project folder (with one `.kicad_pcb`) or at the
  `.kicad_pcb` file itself.
- Keep the `.kicad_sch` files next to it. Symbol fields are read from the
  schematic, including sub-sheets in the same folder.
- After editing the schematic, run **Tools > Update PCB from Schematic** (F8)
  in the PCB editor and save. The importer reads pins and copper from the
  board file, so it only sees changes that have reached the board.

### 2. MOSFET pin names

The importer finds the bridge by looking for transistors whose pins are named
`G`, `D` and `S` (or `GATE`, `DRAIN`, `SOURCE`). The symbols in KiCad's
`Transistor_FET` library already use these names.

If you made your own MOSFET symbol, open it in the symbol editor and name its
pins `G`, `D` and `S`. Pin numbers alone are not enough. A part with several
source pins can have several pins named `S`.

If the import prints `No half-bridge found`, this is almost always the cause.

### 3. Reference designators

| Part | Reference must start with |
|---|---|
| Resistors (shunts, gate resistors, brake resistor) | `R` |
| Capacitors | `C` |
| Transistors and connectors | anything (found by pin names and copper instead) |

The motor connector is found by following each phase net away from the
MOSFETs to the farthest pad that is not a resistor, capacitor or transistor.

### 4. The ground net

Name the power ground `GND`, `PGND`, `AGND`, `DGND` or `VSS`. This is the only
net name the importer relies on. The bus and phase nets are found from how the
MOSFETs connect, whatever they are called.

### 5. Fields to add

Add these in the schematic: double-click a symbol, then use the **+** button
under the fields table to add a field. Untick "Show" to keep it off the
drawing.

**Needed:**

| Field | On | Example | Used for |
|---|---|---|---|
| `MPN` | each bridge MOSFET | `IPT015N10N5` | Looking up datasheet values in [parts.json](parts-table.md). |
| `Voltage` | each bus capacitor | `63V` | The capacitor voltage rating check. Or put it in the Value, like `220u 63V`. |

**Optional.** Each one replaces a stated default, and the import tells you
when it used a default.

| Field | On | Example | Default if missing |
|---|---|---|---|
| `SIM_GAIN` | current sense amplifier | `20` | no current sense values |
| `SIM_ADC_BITS` | microcontroller | `12` | 12 |
| `SIM_ADC_VREF` | microcontroller | `3.3` | 3.3 V |
| `SIM_VDRIVE` | gate driver | `12` | 10 V |
| `SIM_ISOURCE` | gate driver | `1.0` | 1 A peak |
| `SIM_ISINK` | gate driver | `2.0` | 2 A peak |
| `SIM_DEADTIME_NS` | gate driver or microcontroller | `250` | design file value |
| `SIM_DERATE` | ceramic bus capacitors | `0.45` | 1.0 (no derating) |
| `SIM_HEATSINK_RTH` | any part, usually a MOSFET | `2.5` | no heatsink |
| `SIM_ROLE` | any resistor or capacitor | `shunt` | guessed (see below) |

- `SIM_DERATE` is the fraction of a ceramic capacitor's capacitance left at
  your bus voltage. Ceramics lose a lot under DC bias. The maker's
  "capacitance vs DC bias" curve gives it.
- `SIM_HEATSINK_RTH` is the heatsink's thermal resistance to air in K/W. It is
  added in parallel with the cooling the board's copper provides.
- The current sense range assumes a bidirectional amplifier biased at half the
  ADC reference.

### 6. How parts are recognized

| Role | Recognized when | Override |
|---|---|---|
| Shunt | a resistor of 50 milliohm or less | `SIM_ROLE` = `shunt` |
| Gate resistor | a resistor under 1 kohm on a MOSFET gate net | `SIM_ROLE` = `gate_resistor` |
| Brake resistor | a 1 to 100 ohm resistor on the bus net | `SIM_ROLE` = `brake_resistor` |
| Bus capacitor | a capacitor between the bus and ground | `SIM_ROLE` = `bus_cap` |

### 7. Copper and cooling

- Draw the phase, bus and ground copper as tracks or zones. The importer reads
  copper thickness from **Board Setup > Physical Stackup**, so set it there
  (2 oz copper is 0.070 mm).
- The MOSFET cooling estimate uses the copper area around the bridge and
  still air. With a fan, pass `--convection 40` to `--convection 80`.

## Reading the import report

The report has four parts:

1. **Board topology**: which parts it decided are the phases, high and low
   side MOSFETs, shunts, gate resistors, bus capacitors and brake resistor.
   Check this first. If something is misclassified, add a `SIM_ROLE` field.
2. **Parameters from the board**: each value, the base design's value, and
   whether it came from the board, is an estimate, or was kept from the base.
3. **Board checks**: MOSFET and capacitor ratings against the bus voltage,
   current sensing range, and whether the copper is small next to the MOSFET
   resistance.
4. **Notes**: defaults it used and anything it could not find.

Then run `volt2motion design out/<name>.json`, or pass `--run`, to see how the
whole joint behaves with this board.

## Known limits

- One board per folder, one three-phase bridge per board.
- All six MOSFETs are assumed to be the same type.
- The MOSFET temperature is an average over the bridge. At stall, one MOSFET
  can carry peak current continuously and run hotter than reported.
- Switching transients and gate ringing are not modeled. Use LTspice for those.
