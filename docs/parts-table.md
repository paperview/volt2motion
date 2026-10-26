# The parts table

KiCad files know which MOSFET is where, but not its datasheet values. Those go
in `parts.json`. `volt2motion import-kicad` uses `parts.json` in the current
folder, or the file you pass with `--parts`. Without either it falls back to a
bundled example table, whose parts are made up.

`volt2motion init` writes a copy of the example to start from.

## Format

```json
{
  "mosfets": {
    "YOUR-MPN-HERE": {
      "_source": "datasheet rev 2.1, page 3",
      "vds_max_v": 80,
      "id_max_a": 120,
      "rds_on_25c_ohm": 0.0025,
      "rds_tempco_per_k": 0.0065,
      "qgs_c": 15e-9,
      "qgd_c": 10e-9,
      "v_plateau_v": 4.5,
      "rg_internal_ohm": 1.0,
      "rth_jc_k_per_w": 0.9
    }
  }
}
```

- **The key must match the part on the board exactly.** The importer looks
  for an `MPN` field on the symbol (also accepted: `Manufacturer Part Number`,
  `MFR_PN`, `Part Number`, `PN`). If there is none, it uses the symbol's
  `Value`.
- **Every field is required and must be a number.** A missing or misspelled
  field stops the import and names it.
- Keys starting with `_` are comments.
- Use SI units throughout. Gate charge is in coulombs, so 15 nC is `15e-9`.

## Where each value is on the datasheet

| Field | Datasheet name | Notes |
|---|---|---|
| `vds_max_v` | V(BR)DSS, or VDS in the absolute maximum table | The importer checks it is at least 1.25x the bus overvoltage trip. |
| `id_max_a` | ID, continuous drain current | Use the lower of the silicon and package limits if both are given. |
| `rds_on_25c_ohm` | RDS(on) | Use the **maximum** (not typical) at 25 C, at the gate voltage your driver supplies (usually VGS = 10 V). |
| `rds_tempco_per_k` | "Normalized on-resistance vs temperature" graph | Read the ratio at 125 C, subtract 1, divide by 100. A ratio of 1.65 gives 0.0065. |
| `qgs_c` | Qgs, gate-to-source charge | From the gate charge table. Use the hot (100 C) column if the datasheet has one. |
| `qgd_c` | Qgd, gate-to-drain (Miller) charge | Same table. This one sets most of the switching loss. |
| `v_plateau_v` | Vplateau, or the flat part of the gate charge curve | Read the gate voltage on the flat section of the "gate charge" graph. |
| `rg_internal_ohm` | RG, internal gate resistance | Often in the dynamic characteristics table. Use 1 if it is not given. |
| `rth_jc_k_per_w` | RthJC, junction to case | Thermal resistance from the die to the drain tab. |

If the board uses more than one MOSFET type in the bridge, the importer uses
the first type for all six and says so in its notes.
